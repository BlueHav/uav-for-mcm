"""Receive Unity textures directly into CUDA tensors and expose small previews."""
import ctypes
import os
from pathlib import Path

import numpy as np
import torch
from scipy.spatial.transform import Rotation

CAMERAS=('front','up','down')


class CudaDepthFrame:
    def __init__(self,pixels):
        self.cuda_pixels=pixels
        self.shape=tuple(pixels.shape)

    def copy(self):
        # Each received frame owns its allocation and is immutable.
        return self

    def preview(self,width,height):
        depth=self.cuda_pixels
        y=torch.arange(height,device=depth.device)*depth.shape[0]//height
        x=torch.arange(width,device=depth.device)*depth.shape[1]//width
        image=depth[y[:,None],x]
        return torch.nan_to_num(image,nan=0.,posinf=20.,neginf=0.).clamp_(.04,20.)/20.

    def rays(self,yy,xx,pose,rotation,fx,fy,cx,cy,peers):
        depth=self.cuda_pixels
        y=torch.as_tensor(yy.reshape(-1),device=depth.device)
        x=torch.as_tensor(xx.reshape(-1),device=depth.device)
        z=depth[y,x]
        valid=torch.isfinite(z)&(z>.05)
        y,x,z=y[valid],x[valid],z[valid].to(torch.float64)
        body=torch.stack((z,-(x-cx)*z/fx,-(y-cy)*z/fy),dim=1)
        peer_hit=torch.zeros(len(z),device=depth.device,dtype=torch.bool)
        for center,radius in peers:
            delta=body-torch.as_tensor(center,device=depth.device)
            peer_hit|=delta.square().sum(dim=1)<=radius**2
        rays=body@torch.as_tensor(rotation,device=depth.device,dtype=torch.float64).T
        rays=rays.contiguous()
        endpoints=rays+torch.as_tensor(pose,device=depth.device,dtype=torch.float64)
        # Return geometry needed by the CPU map, never the source depth image.
        endpoints_and_peer=torch.cat((endpoints,peer_hit[:,None]),dim=1).cpu().numpy()
        return rays,endpoints_and_peer[:,:3],endpoints_and_peer[:,3].astype(bool)


class CudaDepthSource:
    def __init__(self,identity):
        library=os.environ['YOPO_CUDA_DEPTH_LIBRARY']
        self.lib=ctypes.CDLL(str(Path(library)))
        self.lib.yopo_depth_open.argtypes=[ctypes.c_int]
        self.lib.yopo_depth_open.restype=ctypes.c_void_p
        self.lib.yopo_depth_read.argtypes=[ctypes.c_void_p,ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint64),ctypes.POINTER(ctypes.c_float)]
        self.lib.yopo_depth_read.restype=ctypes.c_int
        self.lib.yopo_depth_close.argtypes=[ctypes.c_void_p]
        self.identity=identity
        self.handles=[None]*3
        self.buffers=[None]*3
        self.buffer_consumed=[None]*3
        self.stamps=[ctypes.c_uint64() for _ in CAMERAS]
        self.poses=[(ctypes.c_float*7)() for _ in CAMERAS]

    def take(self):
        frames={}
        for i,camera in enumerate(CAMERAS):
            handle=self.handles[i]
            if not handle:
                handle=self.handles[i]=self.lib.yopo_depth_open(3*self.identity+i)
            if not handle:
                continue
            raw=self.buffers[i]
            if raw is None:
                raw=self.buffers[i]=torch.empty((270,480),device='cuda',dtype=torch.float32)
                self.buffer_consumed[i]=torch.cuda.Event()
            else:
                # The IPC library copies on its own CUDA stream. A PyTorch
                # allocation and its last read on the current stream alone
                # do not order writes on that external stream.
                self.buffer_consumed[i].synchronize()
            if self.lib.yopo_depth_read(handle,raw.data_ptr(),ctypes.byref(self.stamps[i]),self.poses[i]):
                pose=self.poses[i]
                position=np.asarray(pose[:3],dtype=float)
                rotation=Rotation.from_quat(pose[3:]).as_matrix()
                pixels=raw.flip((0,))
                pixels=torch.where(torch.isfinite(pixels)&(pixels>0),pixels.clamp(max=80.),80.)
                self.buffer_consumed[i].record()
                # Planning and the preview publisher use different threads.
                # Publish a frame only after its immutable tensor is complete.
                self.buffer_consumed[i].synchronize()
                frames[camera]=(self.stamps[i].value*1e-9,CudaDepthFrame(pixels),(position,rotation))
        return frames

    def close(self):
        for handle in self.handles:
            if handle:self.lib.yopo_depth_close(handle)
        self.handles=[None]*3

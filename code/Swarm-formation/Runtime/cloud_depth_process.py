"""Private, bounded synthetic sensor IPC. This module never loads scene geometry."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import numpy as np
from visual_transport import SnapshotMailbox

CAMERAS=('front','up','down')

def validate_frame(frame,own_id,map_hash,last_sequence,now,monotonic):
    """Reject invalid input before it can refresh flight sensor timestamps."""
    if not isinstance(frame,dict):raise ValueError('invalid sensor envelope')
    if frame.get('own_id')!=own_id or frame.get('map_hash')!=map_hash:raise ValueError('sensor identity/hash mismatch')
    sequence=frame.get('sequence')
    if type(sequence) is not int or sequence<=last_sequence:raise ValueError('nonmonotonic sequence')
    stamp=float(frame['stamp']);start=float(frame['started']);finish=float(frame['finished'])
    if not np.isfinite([stamp,start,finish]).all():raise ValueError('nonfinite sensor times')
    if not -.1<=now-stamp<=.6:raise ValueError('stale/future pose')
    if not 0<=finish-start<=.3 or not -.1<=monotonic-finish<=.6:raise ValueError('stale/slow generation')
    p=np.asarray(frame['position'])
    if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('invalid position')
    if set(frame['images'])!=set(CAMERAS) or set(frame['rotations'])!=set(CAMERAS):raise ValueError('incomplete camera set')
    masks=frame.get('peer_masks')
    if masks is not None and (not isinstance(masks,dict) or set(masks)!=set(CAMERAS)):raise ValueError('incomplete peer source masks')
    for camera in CAMERAS:
        image=frame['images'][camera];rotation=np.asarray(frame['rotations'][camera])
        if not isinstance(image,np.ndarray) or image.shape!=(270,480) or image.dtype!=np.float32:raise ValueError('invalid depth shape/type')
        if not np.isfinite(image).all() or np.any(image<0) or np.any(image>80):raise ValueError('invalid depth values')
        if masks is not None:
            mask=masks[camera]
            if not isinstance(mask,np.ndarray) or mask.shape!=image.shape or mask.dtype!=np.bool_:raise ValueError('invalid peer source mask')
        if rotation.shape!=(3,3) or not np.isfinite(rotation).all():raise ValueError('invalid rotation')
        if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(rotation),1,atol=1e-5):raise ValueError('nonrigid rotation')
    return frame

class CloudDepthProcess:
    def __init__(self,drone_id,count,map_hash=None,worker_command=None,clock=time.time,monotonic=time.monotonic):
        if count!=10 or not 0<=drone_id<count:raise ValueError('pointcloud depth requires ten valid drone identities')
        if map_hash is None:
            with open(('__UAV_PROJECT__/config/point.json'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"]))) as stream:map_hash=json.load(stream)['map_fingerprints']['world']
        self.id=drone_id;self.map_hash=map_hash;self.clock=clock;self.monotonic=monotonic
        self.sequence=0;self.last_stamp=-float('inf');self.accepted=0;self.rejected=0;self.error=None;self.closed=False
        self.worker_status=dict(state='starting',started=monotonic())
        self.directory=tempfile.TemporaryDirectory(prefix='yopo-cloud-depth-',dir='/dev/shm');self.root=self.directory.name
        self.mailbox=None;self.health=None;self.process=None
        try:
            self.mailbox=SnapshotMailbox(self.root+'/frames',create=True,capacity=2*1024*1024)
            self.health=SnapshotMailbox(self.root+'/health',create=True,capacity=64*1024)
            command=worker_command or [sys.executable,str(Path(__file__).with_name('cloud_depth_worker.py')),self.root,str(drone_id),str(count),str(os.getpid()),map_hash]
            # Inherit the mission process group: launcher teardown also owns this worker.
            self.process=subprocess.Popen(command,close_fds=True)
        except BaseException:
            self.close();raise

    def take(self):
        if self.closed:return None
        if self.process.poll() is not None:self.error='sensor worker exited';return None
        try:
            value=self.mailbox.take()
            if value is None:return None
            validate_frame(value,self.id,self.map_hash,self.sequence,self.clock(),self.monotonic())
            if value['stamp']<=self.last_stamp:raise ValueError('repeated/regressing pose stamp')
            self.sequence=value['sequence'];self.last_stamp=value['stamp'];self.accepted+=1;self.error=None
            return value
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            self.rejected+=1;self.error=str(exc);return None

    def status(self):
        if not self.closed:
            value=self.health.take()
            if value is not None:self.worker_status=value
        return dict(pid=self.process.pid if self.process else None,alive=self.process is not None and self.process.poll() is None,
                    accepted=self.accepted,rejected=self.rejected,error=self.error,sequence=self.sequence,worker=self.worker_status)

    def close(self):
        if self.closed:return
        self.closed=True
        if self.process is not None and self.process.poll() is None:
            self.process.kill();self.process.wait(timeout=2)
        if self.health is not None:self.health.close()
        if self.mailbox is not None:self.mailbox.close()
        self.directory.cleanup()

    def __enter__(self):return self
    def __exit__(self,*args):self.close()

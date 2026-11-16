using System;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Rendering;

namespace Yopo.TrackerTest {
 public partial class TrackerField {
  [StructLayout(LayoutKind.Sequential)]
  struct CudaDepthSubmission {
   public int pending,camera;
   public IntPtr texture,renderTarget;
   public ulong stampNs;
   public float x,y,z,qx,qy,qz,qw;
  }
  [DllImport("yopo_cuda_depth_ipc",CallingConvention=CallingConvention.Cdecl)]
  static extern IntPtr yopo_depth_capture_callback();

  readonly IntPtr[] cudaDepthSubmissions=new IntPtr[30];
  readonly IntPtr[] cudaDepthTextures=new IntPtr[30];
  readonly IntPtr[] cudaDepthRenderTargets=new IntPtr[30];
  readonly RenderTexture[] cudaDepthSnapshots=new RenderTexture[30];
  IntPtr cudaDepthCallback;
  CommandBuffer cudaDepthCommands;
  bool cudaDepthChecked;
  readonly bool cudaDepthOnly=Environment.GetEnvironmentVariable("YOPO_CUDA_DEPTH_ONLY")=="1";

  void SendCudaDepth(Drone drone, RenderTexture texture, bool front, bool up,
                     ulong stampNs, Vector3 position, Quaternion orientation) {
   if(!cudaDepthChecked) {
    cudaDepthChecked=true;
    var directory=Environment.GetEnvironmentVariable("YOPO_CUDA_DEPTH_IPC_DIR");
    if(!Application.isEditor&&!string.IsNullOrEmpty(directory)&&
       SystemInfo.graphicsDeviceType==GraphicsDeviceType.OpenGLCore) {
     try { cudaDepthCallback=yopo_depth_capture_callback(); }
     catch(Exception error) { Debug.LogWarning("CUDA depth diagnostics unavailable: "+error.Message); }
    }
   }
   if(cudaDepthCallback==IntPtr.Zero) {
    if(cudaDepthOnly)throw new InvalidOperationException("CUDA-only depth requires the OpenGL CUDA plugin and an IPC directory.");
    return;
   }
   int index=drone.id*3+(front?0:up?1:2);
   if(cudaDepthSubmissions[index]==IntPtr.Zero) {
    var snapshot=cudaDepthSnapshots[index];
    if(snapshot==null) {
     snapshot=new RenderTexture(texture.descriptor){name="YOPO CUDA depth snapshot "+index};
     if(!snapshot.Create()){Destroy(snapshot);return;}
     cudaDepthSnapshots[index]=snapshot;
    }
    cudaDepthTextures[index]=snapshot.GetNativeTexturePtr();
    cudaDepthRenderTargets[index]=texture.GetNativeTexturePtr();
    if(cudaDepthTextures[index]==IntPtr.Zero)return;
    cudaDepthSubmissions[index]=Marshal.AllocHGlobal(Marshal.SizeOf<CudaDepthSubmission>());
    Marshal.WriteInt32(cudaDepthSubmissions[index],0);
   }
   // A bounded block per camera lives through queued render-thread events.
   // Never overwrite a submission while its native callback owns it.
   var submission=cudaDepthSubmissions[index];
   if(Marshal.ReadInt32(submission)!=0)return;
   Marshal.StructureToPtr(new CudaDepthSubmission {
    pending=1,camera=index,texture=cudaDepthTextures[index],renderTarget=cudaDepthRenderTargets[index],stampNs=stampNs,
    x=position.x,y=position.y,z=position.z,qx=orientation.x,qy=orientation.y,
    qz=orientation.z,qw=orientation.w
   },submission,false);
   if(cudaDepthCommands==null)cudaDepthCommands=new CommandBuffer {name="YOPO CUDA depth capture"};
   cudaDepthCommands.Clear();
   // Snapshot and export share a render command buffer. The source camera can
   // render again without touching the texture registered with CUDA.
   cudaDepthCommands.CopyTexture(texture,cudaDepthSnapshots[index]);
   cudaDepthCommands.IssuePluginEventAndData(cudaDepthCallback,index,submission);
   Graphics.ExecuteCommandBuffer(cudaDepthCommands);
  }
 }
}

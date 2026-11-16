using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
namespace Yopo.TrackerTest {
 public partial class TrackerField {
  readonly HashSet<Camera> asyncDepthBusy=new HashSet<Camera>();
  int completedFrontFrames;
  bool depthClosing;
  readonly HashSet<string> parityChecked=new HashSet<string>();
  void PublishAsyncDepth(Drone d,Camera camera,RenderTexture target,string name,float[] values){
   if(depthClosing||asyncDepthBusy.Contains(camera))return;
   asyncDepthBusy.Add(camera);
   double captureStarted=SensorClock();
   bool front=name=="front",up=name=="up";
   var header=Header("drone_"+d.id+"/"+(front?"camera_frame":name+"_camera_frame"));
   var position=WorldGeometry.UnityToRos(camera.transform.position);
   var q=front?Quaternion.LookRotation(WorldGeometry.UnityToRos(camera.transform.up),-WorldGeometry.UnityToRos(camera.transform.right)):new Quaternion(0,up?-Mathf.Sqrt(.5f):Mathf.Sqrt(.5f),0,Mathf.Sqrt(.5f));
   var pose=new PoseStampedMsg{header=new HeaderMsg{stamp=header.stamp,frame_id="world"}};
   pose.pose.position=new PointMsg(position.x,position.y,position.z);pose.pose.orientation=new QuaternionMsg(q.x,q.y,q.z,q.w);
   bool[] enabled=d.selfDepthLayer>=0?null:new bool[d.renderers.Length];
   if(enabled!=null)for(int i=0;i<enabled.Length;i++){enabled[i]=d.renderers[i].enabled;d.renderers[i].enabled=false;}
   try{camera.RenderWithShader(depthShader,"");}
   catch{asyncDepthBusy.Remove(camera);throw;}
   finally{if(enabled!=null)for(int i=0;i<enabled.Length;i++)d.renderers[i].enabled=enabled[i];}
   SensorTiming("render",captureStarted);
   SendCudaDepth(d,target,front,up,(ulong)header.stamp.sec*1000000000UL+header.stamp.nanosec,position,q);
   if(cudaDepthOnly) {
    asyncDepthBusy.Remove(camera);
    return;
   }
   float[] parity=null;
   if(d.id==2&&parityChecked.Add(name)){
    var old=RenderTexture.active;var check=new Texture2D(Width,Height,TextureFormat.RFloat,false,true);
    try{RenderTexture.active=target;check.ReadPixels(new Rect(0,0,Width,Height),0,0,false);parity=check.GetRawTextureData<float>().ToArray();}
    finally{RenderTexture.active=old;Destroy(check);}
   }
   try{AsyncGPUReadback.Request(target,0,TextureFormat.RFloat,request=>{
    try{
     if(depthClosing)return;
     double callbackStarted=SensorClock();
     SensorTiming("capture_to_readback",captureStarted);double conversionStarted=SensorClock();
     if(request.hasError){Debug.LogWarning("Depth readback failed: "+camera.name);return;}
     var source=request.GetData<float>();
     if(parity!=null){double same=0,flipped=0;for(int r=0;r<Height;r++)for(int c=0;c<Width;c++){int i=r*Width+c;double a=source[i]-parity[i],b=source[i]-parity[(Height-r-1)*Width+c];same+=a*a;flipped+=b*b;}Debug.Log("ASYNC_DEPTH_PARITY "+name+" same_rmse="+Math.Sqrt(same/source.Length)+" flipped_rmse="+Math.Sqrt(flipped/source.Length));}
     var slot=depthTransport.Pool.Acquire(d.id*3+(front?0:up?1:2));
     if(slot==null)return;
     try{
      DepthFramePacking.PackInto(source,values,slot.Data,Width,Height);
      slot.Sec=header.stamp.sec;slot.Nsec=header.stamp.nanosec;
      slot.X=position.x;slot.Y=position.y;slot.Z=position.z;
      slot.Qx=q.x;slot.Qy=q.y;slot.Qz=q.z;slot.Qw=q.w;
     }catch{depthTransport.Pool.Release(slot);throw;}
     SensorTiming("conversion",conversionStarted);
     double publishStarted=SensorClock();
     depthTransport.Submit(slot);
     if(front)completedFrontFrames++;
     SensorTiming("publish",publishStarted);SensorTiming("readback_callback",callbackStarted);
    }finally{asyncDepthBusy.Remove(camera);}
   });}catch(Exception error){asyncDepthBusy.Remove(camera);Debug.LogError("Depth request failed: "+error.Message);}
  }
 }
}

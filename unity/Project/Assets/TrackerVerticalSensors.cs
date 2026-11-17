using System;
using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
namespace Yopo.TrackerTest {
 public partial class TrackerField {
  class VerticalSensor {public Camera camera;public RenderTexture target;public Texture2D read;public string name;public float[] scratch;}
  readonly System.Collections.Generic.Dictionary<int,VerticalSensor[]> vertical=new System.Collections.Generic.Dictionary<int,VerticalSensor[]>();
  void SetupVertical(Drone d){var sensors=new VerticalSensor[2];for(int i=0;i<2;i++){
   string name=i==0?"up":"down";var s=new VerticalSensor{name=name};s.camera=new GameObject("Drone_"+d.id+"_"+name).AddComponent<Camera>();s.camera.CopyFrom(d.depthCamera);s.camera.fieldOfView=120;s.camera.enabled=false;
   s.target=new RenderTexture(Width,Height,24,RenderTextureFormat.RFloat,RenderTextureReadWrite.Linear);s.target.Create();s.camera.targetTexture=s.target;s.read=new Texture2D(Width,Height,TextureFormat.RFloat,false,true);sensors[i]=s;
   ros.RegisterPublisher<ImageMsg>(d.Topic(name+"_depth"),1);ros.RegisterPublisher<CameraInfoMsg>(d.Topic(name+"_camera_info"),1);ros.RegisterPublisher<PoseStampedMsg>(d.Topic(name+"_camera_pose"),1);
  }vertical[d.id]=sensors;}
  void PublishVertical(Drone d){foreach(var s in vertical[d.id]){
   bool up=s.name=="up";s.camera.transform.position=d.body.position;s.camera.transform.rotation=Quaternion.LookRotation(up?Vector3.up:Vector3.down,up?Vector3.back:Vector3.forward);
   if(drones.Count==10&&SystemInfo.supportsAsyncGPUReadback){if(s.scratch==null)s.scratch=new float[Width*Height];PublishAsyncDepth(d,s.camera,s.target,s.name,s.scratch);continue;}
   var old=RenderTexture.active;var enabled=new bool[d.renderers.Length];for(int i=0;i<enabled.Length;i++){enabled[i]=d.renderers[i].enabled;d.renderers[i].enabled=false;}
   try{s.camera.RenderWithShader(depthShader,"");RenderTexture.active=s.target;s.read.ReadPixels(new Rect(0,0,Width,Height),0,0,false);}finally{RenderTexture.active=old;for(int i=0;i<enabled.Length;i++)d.renderers[i].enabled=enabled[i];}
   var source=s.read.GetRawTextureData<float>();if(s.scratch==null)s.scratch=new float[Width*Height];var values=s.scratch;for(int r=0;r<Height;r++)for(int c=0;c<Width;c++){float m=source[(Height-r-1)*Width+c];values[r*Width+c]=float.IsNaN(m)||float.IsInfinity(m)||m<=0?80:Mathf.Min(m,80);}
   var bytes=new byte[values.Length*4];Buffer.BlockCopy(values,0,bytes,0,bytes.Length);var header=Header("drone_"+d.id+"/"+s.name+"_camera_frame");ros.Publish(d.Topic(s.name+"_depth"),new ImageMsg(header,Height,Width,"32FC1",0,Width*4,bytes));
   ros.Publish(d.Topic(s.name+"_camera_info"),new CameraInfoMsg{header=header,width=Width,height=Height,distortion_model="plumb_bob",k=new double[]{77.94228634059948,0,240,0,77.94228634059948,135,0,0,1},r=new double[]{1,0,0,0,1,0,0,0,1},p=new double[]{77.94228634059948,0,240,0,0,77.94228634059948,135,0,0,0,1,0}});
   // ROS FLU orientation: yaw=0, pitch=-90 up / +90 down.
   double q=Math.Sqrt(.5);var pose=new PoseStampedMsg{header=new RosMessageTypes.Std.HeaderMsg{stamp=header.stamp,frame_id="world"}};pose.pose.position=new PointMsg(d.p.x,d.p.y,d.p.z);pose.pose.orientation=new QuaternionMsg(0,up?-q:q,0,q);ros.Publish(d.Topic(s.name+"_camera_pose"),pose);
  }}
  void ReleaseVertical(Drone d){if(!vertical.TryGetValue(d.id,out var sensors))return;foreach(var s in sensors){s.target.Release();Destroy(s.read);Destroy(s.camera.gameObject);}}
 }
}


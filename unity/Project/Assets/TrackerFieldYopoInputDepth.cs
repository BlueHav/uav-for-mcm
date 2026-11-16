using System;
using UnityEngine;
using RosMessageTypes.Sensor;
namespace Yopo.TrackerTest {
 public partial class TrackerField {
  YopoInputDepthFrame[] inputDepthFrames;
  Texture2D inputDepthTexture;
  Color32[] inputDepthPixels;
  int inputDepthTextureId=-1,inputDepthDisplayId=-1;
  long inputDepthTextureVersion=-1;
  bool inputDepthVisible;
  string inputDepthState="WAITING FOR ACTUAL INFERENCE INPUT",inputDepthStamp="stamp --",inputDepthAges="source age -- | receive age --",inputDepthSize="normalized 0..1";
  static string InputDepthTopic(int id)=>"/drone_"+id+"/yopo/input_depth";
  void InitYopoInputDepth(){
   inputDepthFrames=new YopoInputDepthFrame[10];
   for(int i=0;i<10;i++){var frame=new YopoInputDepthFrame(i);inputDepthFrames[i]=frame;ros.Subscribe<ImageMsg>(InputDepthTopic(i),message=>frame.Receive(message,Time.realtimeSinceStartupAsDouble));}
  }
  void UpdateYopoInputDepth(){
   if(inputDepthFrames==null)return;
   inputDepthDisplayId=selected;
   var frame=inputDepthFrames[selected];double now=Time.realtimeSinceStartupAsDouble,utc=DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()*.001;
   inputDepthVisible=false;
   inputDepthSize="normalized 0..1"+(frame.HasFrame?" | "+frame.Width+" x "+frame.Height:"");
   inputDepthStamp="stamp "+frame.StampText;
   inputDepthAges=frame.HasFrame?"source age "+(utc-frame.SourceSeconds).ToString("F2")+" s | receive age "+(now-frame.ReceivedAt).ToString("F2")+" s":"source age -- | receive age --";
   if(!frame.HasFrame){inputDepthState=frame.Error==null?"WAITING FOR ACTUAL INFERENCE INPUT":"REJECTED: "+frame.Error;return;}
   if(!frame.IsFresh(now,utc)){inputDepthState=utc-frame.SourceSeconds<-.5?"CLOCK MISMATCH / INPUT HIDDEN":"STALE INPUT / IMAGE HIDDEN";return;}
   if(inputDepthTexture==null||inputDepthTexture.width!=frame.Width||inputDepthTexture.height!=frame.Height){
    if(inputDepthTexture!=null)Destroy(inputDepthTexture);
    inputDepthTexture=new Texture2D(frame.Width,frame.Height,TextureFormat.RGBA32,false,true){name="ActualYopoInferenceInput",filterMode=FilterMode.Point,wrapMode=TextureWrapMode.Clamp};
    inputDepthPixels=new Color32[frame.Width*frame.Height];inputDepthTextureVersion=-1;
   }
   if(inputDepthTextureId!=selected||inputDepthTextureVersion!=frame.Version){
    frame.CopyGrayscale(inputDepthPixels);inputDepthTexture.SetPixels32(inputDepthPixels);inputDepthTexture.Apply(false,false);
    inputDepthTextureId=selected;inputDepthTextureVersion=frame.Version;
   }
   inputDepthVisible=true;inputDepthState="ACTUAL INFERENCE INPUT";
  }
  void DrawYopoInputDepth(float screenWidth){
   bool current=inputDepthDisplayId==selected;
   float x=screenWidth-352;GUI.DrawTexture(new Rect(x,18,334,314),hudBackground);
   GUI.Label(new Rect(x+12,21,310,25),"YOPO INPUT / UAV "+selected,hudTitle);
   GUI.Label(new Rect(x+12,47,310,22),current?inputDepthSize:"normalized 0..1",hudMuted);
   GUI.Label(new Rect(x+12,69,310,22),current?inputDepthStamp:"stamp --",hudMuted);
   FitLabel(new Rect(x+12,91,310,22),current?inputDepthAges:"source age -- | receive age --",hudMuted);
   Rect image=new Rect(x+7,116,320,192);
   // A selection changes during OnGUI, after LateUpdate: hide the old ID until
   // the next main-thread upload rather than briefly labelling another UAV's input.
   if(current&&inputDepthVisible&&inputDepthTextureId==selected)GUI.DrawTexture(image,inputDepthTexture,ScaleMode.ScaleToFit,false);
   else FitLabel(new Rect(image.x+7,image.y+60,image.width-14,60),!current?"WAITING FOR SELECTED UAV INPUT":inputDepthState,hudMuted);
  }
  void ReleaseYopoInputDepth(){
   if(inputDepthFrames!=null&&ros!=null)for(int i=0;i<inputDepthFrames.Length;i++)ros.Unsubscribe(InputDepthTopic(i));
   inputDepthFrames=null;inputDepthPixels=null;inputDepthVisible=false;
   if(inputDepthTexture!=null){Destroy(inputDepthTexture);inputDepthTexture=null;}
  }
 }
}

using System;
using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
public static class YopoInputDepthAudit {
 static void Check(bool pass,string reason){if(!pass)throw new Exception("YOPO_INPUT_DEPTH_AUDIT_FAILED "+reason);}
 static ImageMsg Frame(){var data=new byte[16];Buffer.BlockCopy(new[]{0f,.25f,.75f,1f},0,data,0,16);return new ImageMsg(new HeaderMsg{stamp=new TimeMsg(100,250000000),frame_id="drone_7/yopo_input"},2,2,"32FC1",0,8,data);}
 public static void Run(){
  var type=typeof(Yopo.TrackerTest.TrackerField).Assembly.GetType("Yopo.TrackerTest.YopoInputDepthFrame");
  Check(type!=null,"actual input frame receiver is missing");
  var cache=Activator.CreateInstance(type,new object[]{7});
  var receive=(Func<ImageMsg,double,bool>)Delegate.CreateDelegate(typeof(Func<ImageMsg,double,bool>),cache,type.GetMethod("Receive"));
  var fresh=(Func<double,double,bool>)Delegate.CreateDelegate(typeof(Func<double,double,bool>),cache,type.GetMethod("IsFresh"));
  var copy=(Action<Color32[]>)Delegate.CreateDelegate(typeof(Action<Color32[]>),cache,type.GetMethod("CopyGrayscale"));
  Func<string,object> get=name=>type.GetProperty(name).GetValue(cache);
  Check(!fresh(0,100.3),"missing frame is not fresh");
  Check(receive(Frame(),4),"valid actual input accepted");Check((int)get("Width")==2&&(int)get("Height")==2,"dimensions retained");
  Check((string)get("StampText")=="100.250000000","source stamp retained");Check(fresh(4.1,100.35),"fresh frame visible");
  Check(!fresh(5.3,100.35),"expired receive hidden");Check(!fresh(4.1,101.6),"expired source hidden");Check(!fresh(4.1,99),"future source hidden");
  var colors=new Color32[4];copy(colors);Check(colors[0].r==191&&colors[1].r==255&&colors[2].r==0&&colors[3].r==64,"grayscale pixels and top-left ROS orientation");
  Check(colors[0].r==colors[0].g&&colors[0].g==colors[0].b&&colors[0].a==255,"opaque grayscale");
  Action<string,Action<ImageMsg>> reject=(name,mutate)=>{var m=Frame();mutate(m);Check(!receive(m,5),name+" rejected");Check(!fresh(5,100.3),name+" cannot retain misleading image");};
  reject("encoding",m=>m.encoding="16UC1");reject("endianness",m=>m.is_bigendian=1);
  reject("zero width",m=>m.width=0);reject("zero height",m=>m.height=0);reject("wide",m=>m.width=481);reject("tall",m=>m.height=271);
  reject("row padding",m=>m.step=12);reject("short row",m=>m.step=4);reject("short bytes",m=>m.data=new byte[15]);reject("extra bytes",m=>m.data=new byte[17]);reject("null bytes",m=>m.data=null);
  reject("null header",m=>m.header=null);reject("wrong id",m=>m.header.frame_id="drone_6/yopo_input");reject("bad nanos",m=>m.header.stamp.nanosec=1000000000);reject("zero stamp",m=>m.header.stamp=new TimeMsg(0,0));
  foreach(float value in new[]{float.NaN,float.PositiveInfinity,float.NegativeInfinity,-.01f,1.01f})reject("non-normalized "+value,m=>Buffer.BlockCopy(new[]{value},0,m.data,0,4));
  var max=Frame();max.width=480;max.height=270;max.step=1920;max.data=new byte[480*270*4];Check(receive(max,6),"bounded maximum accepted");
  Check(receive(Frame(),7),"latest replaces previous");Check((int)get("Width")==2&&(int)get("Height")==2,"latest dimensions used");
  Check((long)get("Version")>1,"version tracks replacement");
  Debug.Log("YOPO_INPUT_DEPTH_AUDIT_OK validation freshness latest-cache grayscale-orientation");
 }
}

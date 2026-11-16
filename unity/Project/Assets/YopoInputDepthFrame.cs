using System;
using UnityEngine;
using RosMessageTypes.Sensor;
namespace Yopo.TrackerTest {
 // ROSConnection dispatches subscribers from its main-thread Update. Each ID owns
 // only its latest validated ROS message; no image queue or per-GUI conversion.
 public sealed class YopoInputDepthFrame {
  public const double MaxAgeSeconds=1.2;
  readonly string expectedFrameId;
  ImageMsg latest;
  public int Width {get;private set;}
  public int Height {get;private set;}
  public double ReceivedAt {get;private set;}
  public double SourceSeconds {get;private set;}
  public string StampText {get;private set;}="--";
  public string Error {get;private set;}
  public long Version {get;private set;}
  public bool HasFrame => latest!=null;
  public YopoInputDepthFrame(int id){expectedFrameId="drone_"+id+"/yopo_input";}
  bool Reject(string reason){latest=null;Width=Height=0;StampText="--";Error=reason;Version++;return false;}
  public bool Receive(ImageMsg image,double receivedAt){
   if(image==null)return Reject("null image");
   if(image.encoding!="32FC1"||image.is_bigendian!=0||!BitConverter.IsLittleEndian)return Reject("requires little-endian 32FC1");
   if(image.width==0||image.height==0||image.width>480||image.height>270)return Reject("dimensions outside 1..480 x 1..270");
   int width=(int)image.width,height=(int)image.height;
   if(image.step!=(uint)(width*4)||image.data==null||image.data.Length!=width*height*4)return Reject("invalid row step or byte count");
   if(image.header==null||image.header.stamp==null||image.header.frame_id!=expectedFrameId)return Reject("wrong source frame ID");
   var stamp=image.header.stamp;
   if(stamp.nanosec>=1000000000||(stamp.sec==0&&stamp.nanosec==0))return Reject("invalid source stamp");
   for(int offset=0;offset<image.data.Length;offset+=4){float value=BitConverter.ToSingle(image.data,offset);if(float.IsNaN(value)||float.IsInfinity(value)||value<0||value>1)return Reject("pixels must be finite normalized 0..1");}
   latest=image;Width=width;Height=height;ReceivedAt=receivedAt;SourceSeconds=stamp.sec+stamp.nanosec*1e-9;
   StampText=stamp.sec+"."+stamp.nanosec.ToString("D9");Error=null;Version++;return true;
  }
  public bool IsFresh(double now,double utcSeconds){
   double sourceAge=utcSeconds-SourceSeconds;
   return latest!=null&&now>=ReceivedAt&&now-ReceivedAt<=MaxAgeSeconds&&sourceAge>=-.5&&sourceAge<=MaxAgeSeconds;
  }
  public void CopyGrayscale(Color32[] pixels){
   if(latest==null||pixels==null||pixels.Length!=Width*Height)throw new ArgumentException("A valid frame and exactly sized pixel buffer are required");
   // ROS row 0 is top-left; Unity texture row 0 is bottom-left. No interpolation,
   // metric rescaling or substitute camera is used for these normalized pixels.
   for(int y=0;y<Height;y++)for(int x=0;x<Width;x++){
    float value=BitConverter.ToSingle(latest.data,(y*Width+x)*4);byte gray=(byte)Math.Round(value*255,MidpointRounding.AwayFromZero);
    pixels[(Height-1-y)*Width+x]=new Color32(gray,gray,gray,255);
   }
  }
 }
}

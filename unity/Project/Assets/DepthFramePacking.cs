using System;
using Unity.Collections;
namespace Yopo.TrackerTest {
 public static class DepthFramePacking {
  public static byte[] Pack(NativeArray<float> source,float[] scratch,int width,int height){
   var bytes=new byte[checked(width*height*4)];PackInto(source,scratch,bytes,width,height);return bytes;
  }
  public static void PackInto(NativeArray<float> source,float[] scratch,byte[] bytes,int width,int height){
   if(width<=0||height<=0||source.Length!=width*height||scratch.Length!=source.Length)throw new ArgumentException("Depth frame shape mismatch");
   if(bytes.Length!=checked(width*height*4))throw new ArgumentException("Depth destination shape mismatch");
   // Copy once across the NativeArray boundary instead of checking it per pixel.
   source.CopyTo(scratch);
   // Ordered comparisons also reject NaN, both infinities and nonpositive values.
   for(int i=0;i<scratch.Length;i++){float m=scratch[i];scratch[i]=m>0&&m<80?m:80;}
   int rowBytes=width*4;
   for(int r=0;r<height;r++)Buffer.BlockCopy(scratch,(height-r-1)*rowBytes,bytes,r*rowBytes,rowBytes);
  }
 }
}

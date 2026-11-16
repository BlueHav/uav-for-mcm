using System;
using System.Diagnostics;
using Unity.Collections;
using Yopo.TrackerTest;
public static class DepthPackingChecks {
 public static void Run(){
  foreach(var shape in new[]{new[]{3,5},new[]{480,270}}){
   int w=shape[0],h=shape[1];var input=new float[w*h];var random=new Random(7901);
   for(int i=0;i<input.Length;i++)input[i]=(float)(random.NextDouble()*100);
   var special=new[]{float.NaN,float.PositiveInfinity,float.NegativeInfinity,-1f,0f,80f,90f,float.Epsilon};Array.Copy(special,input,special.Length);
   using(var native=new NativeArray<float>(input,Allocator.Temp)){
    var actual=DepthFramePacking.Pack(native,new float[input.Length],w,h);
    var expected=new float[input.Length];for(int r=0;r<h;r++)for(int c=0;c<w;c++){float m=input[(h-1-r)*w+c];expected[r*w+c]=float.IsNaN(m)||float.IsInfinity(m)||m<=0?80:Math.Min(m,80);}
    var expectedBytes=new byte[input.Length*4];Buffer.BlockCopy(expected,0,expectedBytes,0,expectedBytes.Length);
    for(int i=0;i<actual.Length;i++)if(actual[i]!=expectedBytes[i])throw new Exception("Depth byte mismatch at "+i);
    var reused=new byte[actual.Length];var reuseScratch=new float[input.Length];
    for(int iteration=0;iteration<5;iteration++){
     DepthFramePacking.PackInto(native,reuseScratch,reused,w,h);
     for(int i=0;i<reused.Length;i++)if(reused[i]!=expectedBytes[i])throw new Exception("Reused depth byte mismatch at "+i);
    }
    if(w==480){var scratch=new float[input.Length];for(int i=0;i<3;i++)DepthFramePacking.Pack(native,scratch,w,h);
     double best=double.MaxValue;for(int batch=0;batch<3;batch++){var clock=Stopwatch.StartNew();for(int i=0;i<20;i++)DepthFramePacking.Pack(native,scratch,w,h);best=Math.Min(best,clock.Elapsed.TotalMilliseconds/20);}
     UnityEngine.Debug.Log("DEPTH_PACKING_MS "+best);if(best>3)throw new Exception("Depth conversion exceeds 3ms budget: "+best);
    }
   }
  }
  UnityEngine.Debug.Log("DEPTH_PACKING_CHECKS_OK exact_bytes=true invalid_depths=true non_square=true");
 }
}

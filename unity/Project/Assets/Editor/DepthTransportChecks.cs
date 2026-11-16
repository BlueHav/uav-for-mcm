using System;
using System.IO;
using System.Threading;
using Yopo.TrackerTest;
public static class DepthTransportChecks {
 static void Check(bool value,string message){if(!value)throw new Exception(message);}
 public static void Run(){
  DepthPackingChecks.Run();
  var pool=new DepthFramePool();
  var first=pool.Acquire(0);first.Data[0]=11;pool.Submit(first);
  var sending=pool.Take();Check(Object.ReferenceEquals(first,sending),"first lease");
  for(int i=0;i<100;i++){var next=pool.Acquire(0);Check(next!=null,"pool saturation");next.Data[0]=(byte)i;pool.Submit(next);}
  Check(sending.Data[0]==11,"in-flight bytes overwritten");
  var latest=pool.Take();Check(latest.Data[0]==99,"latest replacement");
  pool.Release(latest);pool.Release(sending);
  var a=pool.Acquire(0);var b=pool.Acquire(0);var c=pool.Acquire(0);
  Check(pool.Acquire(0)==null,"filling leases reused");
  pool.Release(a);pool.Release(b);pool.Release(c);
  for(int i=0;i<30;i++){var slot=pool.Acquire(i);pool.Submit(slot);}
  var seen=new bool[30];for(int i=0;i<30;i++){var slot=pool.Take();Check(!seen[slot.Stream],"sender fairness");seen[slot.Stream]=true;pool.Release(slot);}
  var codec=new DepthWireEncoder();var frame=pool.Acquire(6);
  frame.Sec=100;frame.Nsec=1;frame.X=1;frame.Y=2;frame.Z=3;frame.Qw=1;frame.Sequence=1;
  for(int i=0;i<frame.Data.Length;i+=4){frame.Data[i+2]=160;frame.Data[i+3]=66;} // 80.0f
  int length;var bytes=codec.Encode(frame,out length);Check(bytes!=frame.Data&&length<frame.Data.Length,"compress constant");
  var dir=Path.Combine(Directory.GetCurrentDirectory(),"Reports");Directory.CreateDirectory(dir);
  using(var output=File.Create(Path.Combine(dir,"depth-csharp-lz4.bin"))){output.Write(codec.Header,0,128);output.Write(bytes,0,length);}
  var random=new Random(17);random.NextBytes(frame.Data);bytes=codec.Encode(frame,out length);
  Check(Object.ReferenceEquals(bytes,frame.Data)&&length==frame.Data.Length,"raw fallback");
  using(var output=File.Create(Path.Combine(dir,"depth-csharp-raw.bin"))){output.Write(codec.Header,0,128);output.Write(bytes,0,length);}
  File.WriteAllBytes(Path.Combine(dir,"depth-csharp-raw.expected"),frame.Data);
  pool.Release(frame);
  // A real concurrent consumer holds sending leases while producer overwrites queued frames.
  Exception threadFailure=null;bool producerDone=false;int consumed=0;long dropsBefore=pool.Dropped;
  var consumer=new Thread(()=>{try{
   while(!Volatile.Read(ref producerDone)||consumed==0){
    var item=pool.Take();if(item==null){Thread.Yield();continue;}
    byte marker=item.Data[0];Thread.Sleep(1);
    for(int k=0;k<item.Data.Length;k+=4096)Check(item.Data[k]==marker,"concurrent lease corruption");
    pool.Release(item);Interlocked.Increment(ref consumed);
   }
  }catch(Exception error){threadFailure=error;}});consumer.Start();
  for(int i=0;i<50000;i++){
   var item=pool.Acquire(i%30);if(item==null)continue;
   for(int k=0;k<item.Data.Length;k+=4096)item.Data[k]=(byte)i;
   pool.Submit(item);
  }
  Volatile.Write(ref producerDone,true);Check(consumer.Join(3000),"consumer stuck");
  if(threadFailure!=null)throw threadFailure;
  Check(consumed>0,"concurrent consumer made no progress");
  Check(pool.Dropped==dropsBefore,"ordinary producer/consumer lock contention dropped frames");
  UnityEngine.Debug.Log("DEPTH_TRANSPORT_CHECKS_OK");
 }
}

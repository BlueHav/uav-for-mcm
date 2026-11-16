using System;
using System.IO;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;
namespace Yopo.TrackerTest {
 public sealed class DepthFrameSlot {
  public const int Bytes=480*270*4;
  public readonly byte[] Data=new byte[Bytes];
  public readonly int Stream;
  internal int State; // 0 free, 1 filling, 2 queued, 3 sending
  public ulong Sequence; public uint Sec,Nsec;
  public double X,Y,Z,Qx,Qy,Qz,Qw;
  internal DepthFrameSlot(int stream){Stream=stream;}
 }
 public sealed class DepthFramePool {
  readonly object[] gates=new object[30];
  readonly DepthFrameSlot[][] slots=new DepthFrameSlot[30][];
  readonly DepthFrameSlot[] queued=new DepthFrameSlot[30];
  readonly ulong[] sequences=new ulong[30];
  int cursor;
  public long Replaced,Dropped;
  public DepthFramePool(){for(int s=0;s<30;s++){gates[s]=new object();slots[s]=new DepthFrameSlot[3];for(int i=0;i<3;i++)slots[s][i]=new DepthFrameSlot(s);}}
  public DepthFrameSlot Acquire(int stream){
   if(stream<0||stream>=30)throw new ArgumentOutOfRangeException("stream");
   // Per-camera critical sections only change slot ownership; they never copy,
   // compress, write sockets or wait for I/O. Contention must not discard images.
   lock(gates[stream]){
    foreach(var slot in slots[stream])if(slot.State==0){slot.State=1;slot.Sequence=++sequences[stream];return slot;}
    var old=queued[stream];if(old!=null){queued[stream]=null;old.State=1;old.Sequence=++sequences[stream];Replaced++;return old;}
    Dropped++;return null;
   }
  }
  public void Submit(DepthFrameSlot slot){lock(gates[slot.Stream]){
   if(slot.State!=1)throw new InvalidOperationException("not a filling lease");
   var old=queued[slot.Stream];if(old!=null){old.State=0;Replaced++;}
   slot.State=2;queued[slot.Stream]=slot;
  }}
  public DepthFrameSlot Take(){
   for(int n=0;n<30;n++){int s=cursor;cursor=(cursor+1)%30;lock(gates[s]){var slot=queued[s];if(slot==null)continue;queued[s]=null;slot.State=3;return slot;}}
   return null;
  }
  public void Release(DepthFrameSlot slot){lock(gates[slot.Stream]){
   if(slot.State!=1&&slot.State!=3)throw new InvalidOperationException("invalid release");slot.State=0;
  }}
 }
 public sealed class DepthWireEncoder {
  [DllImport("liblz4.so.1",CallingConvention=CallingConvention.Cdecl)] static extern int LZ4_compressBound(int inputSize);
  [DllImport("liblz4.so.1",CallingConvention=CallingConvention.Cdecl)] static extern int LZ4_compress_default(byte[] source,byte[] dest,int sourceSize,int destCapacity);
  [DllImport("libz.so.1",CallingConvention=CallingConvention.Cdecl)] static extern UIntPtr crc32(UIntPtr crc,byte[] buffer,uint length);
  readonly byte[] compressed=new byte[LZ4_compressBound(DepthFrameSlot.Bytes)];
  public readonly byte[] Header=new byte[128];
  readonly MemoryStream stream;
  readonly BinaryWriter writer;
  public DepthWireEncoder(){if(!BitConverter.IsLittleEndian)throw new PlatformNotSupportedException("little endian required");stream=new MemoryStream(Header);writer=new BinaryWriter(stream);}
  public byte[] Encode(DepthFrameSlot slot,out int size){
   int packed=LZ4_compress_default(slot.Data,compressed,slot.Data.Length,compressed.Length);
   bool use=packed>0&&packed<slot.Data.Length;size=use?packed:slot.Data.Length;
   uint crc=(uint)crc32(UIntPtr.Zero,slot.Data,(uint)slot.Data.Length).ToUInt64();
   stream.Position=0;writer.Write(0x5A4C4459u);writer.Write((byte)1);writer.Write((byte)(use?1:0));
   writer.Write((byte)(slot.Stream/3));writer.Write((byte)(slot.Stream%3));writer.Write(slot.Sequence);
   writer.Write(slot.Sec);writer.Write(slot.Nsec);writer.Write(480u);writer.Write(270u);
   writer.Write((uint)slot.Data.Length);writer.Write((uint)size);writer.Write(crc);writer.Write(0u);
   writer.Write(slot.X);writer.Write(slot.Y);writer.Write(slot.Z);writer.Write(slot.Qx);writer.Write(slot.Qy);writer.Write(slot.Qz);writer.Write(slot.Qw);
   return use?compressed:slot.Data;
  }
 }
 public sealed class DepthTransport:IDisposable {
  public readonly DepthFramePool Pool=new DepthFramePool();
  readonly DepthWireEncoder encoder=new DepthWireEncoder();
  readonly AutoResetEvent wake=new AutoResetEvent(false);
  readonly Thread worker;
  volatile bool closing;
  volatile TcpClient client;
  public long Sent,Errors,Expired,WireBytes,RawBytes;
  public string LastError="";
  public DepthTransport(){worker=new Thread(SendLoop){IsBackground=true,Name="LosslessDepthSender"};worker.Start();}
  public void Submit(DepthFrameSlot slot){Pool.Submit(slot);wake.Set();}
  void Disconnect(){var current=client;client=null;if(current!=null)try{current.Close();}catch{}}
  void SendLoop(){
   try{while(!closing){
    var slot=Pool.Take();if(slot==null){wake.WaitOne(20);continue;}
    try{
     double age=DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()*.001-(slot.Sec+slot.Nsec*1e-9);
     if(age>.5){Interlocked.Increment(ref Expired);continue;}
     if(client==null){var next=new TcpClient();client=next;next.NoDelay=true;next.SendTimeout=200;next.SendBufferSize=1024*1024;
      // Loopback connect fails immediately if bridge is absent; never block Unity main.
      next.Connect("127.0.0.1",10013);
     }
     int size;var bytes=encoder.Encode(slot,out size);var network=client.GetStream();
     network.Write(encoder.Header,0,encoder.Header.Length);network.Write(bytes,0,size);
     Interlocked.Increment(ref Sent);Interlocked.Add(ref WireBytes,size+128);Interlocked.Add(ref RawBytes,slot.Data.Length);
    }catch(Exception error){Interlocked.Increment(ref Errors);LastError=error.GetType().Name+": "+error.Message;Disconnect();wake.WaitOne(50);}
    finally{Pool.Release(slot);}
   }}finally{Disconnect();}
  }
  public void Dispose(){closing=true;Disconnect();wake.Set();if(worker.Join(1000))wake.Dispose();}
 }
}

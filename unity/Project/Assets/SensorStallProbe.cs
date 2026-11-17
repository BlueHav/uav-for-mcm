// Opt-in diagnostics only. Never changes sensors, flight commands or timeouts.
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using UnityEngine;
using Unity.Profiling;
using Unity.Profiling.LowLevel.Unsafe;

public sealed class SensorStallProbe : MonoBehaviour {
    readonly Dictionary<string,ProfilerRecorder> recorders=new Dictionary<string,ProfilerRecorder>();
    StreamWriter output; double last,next; int gcLast; bool discovered;
    static double Clock(){return (double)Stopwatch.GetTimestamp()/Stopwatch.Frequency;}
    [Serializable] class Metric {public string name;public long value;}
    [Serializable] class Sample {public double stamp,monotonic,frame_gap_ms;public int frame,gc_delta,gc_total;public Metric[] metrics;}
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install(){if(Environment.GetEnvironmentVariable("YOPO_SENSOR_PROBE")=="1")new GameObject("SensorStallProbe").AddComponent<SensorStallProbe>();}
    void Start(){
        string root=Environment.GetEnvironmentVariable("MC_REPORTS")??Application.persistentDataPath;
        output=new StreamWriter(Path.Combine(root,"unity-stalls.jsonl"),false,System.Text.Encoding.UTF8,65536);
        last=Clock();gcLast=GC.CollectionCount(0);
    }
    void Discover(){
        var handles=new List<ProfilerRecorderHandle>();ProfilerRecorderHandle.GetAvailable(handles);
        using(var inventory=new StreamWriter(Path.Combine(Environment.GetEnvironmentVariable("MC_REPORTS")??Application.persistentDataPath,"unity-profiler-markers.txt"))){
            foreach(var handle in handles){var d=ProfilerRecorderHandle.GetDescription(handle);inventory.WriteLine(d.Category.Name+" | "+d.Name+" | "+d.UnitType);
                string n=d.Name;
                if(n=="Main Thread"||n=="Render Thread"||n=="PlayerLoop"||n=="GC Allocated In Frame"||n=="GC Reserved Memory"||n=="GC Used Memory"||d.Category.Name=="GC"||n.Contains("CollectIncremental")||n.Contains("WaitForPresent")||n.Contains("WaitForRenderThread")||n.Contains("WaitForTargetFPS")||n.Contains("WaitForLastPresentation")||n.Contains("ScriptRunBehaviour")||n.Contains("FinishFrameRendering")||n=="Gfx.PresentFrame"||n=="Camera.Render"||n.Contains("AsyncReadback")){
                    if(!recorders.ContainsKey(n))recorders[n]=ProfilerRecorder.StartNew(d.Category,n,1);
                }
            }
        }
    }
    void Update(){
        double now=Clock(),gap=(now-last)*1000;last=now;int gc=GC.CollectionCount(0),delta=gc-gcLast;gcLast=gc;
        if(!discovered&&Time.frameCount>180){Discover();discovered=true;}
        var values=new List<Metric>();foreach(var item in recorders)values.Add(new Metric{name=item.Key,value=item.Value.LastValue});
        output.WriteLine(JsonUtility.ToJson(new Sample{stamp=DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()/1000.0,monotonic=now,frame=Time.frameCount,frame_gap_ms=gap,gc_delta=delta,gc_total=gc,metrics=values.ToArray()}));
        if(now>=next){output.Flush();next=now+1;}
    }
    void OnDestroy(){foreach(var recorder in recorders.Values)recorder.Dispose();output?.Dispose();}
}

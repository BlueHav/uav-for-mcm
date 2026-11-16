using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using RosMessageTypes.Std;
namespace Yopo.TrackerTest {
 public partial class TrackerField {
  readonly Dictionary<string,List<double>> sensorTimings=new Dictionary<string,List<double>>();
  double nextTimingReport;
  static double SensorClock(){return (double)Stopwatch.GetTimestamp()/Stopwatch.Frequency;}
  void SensorTiming(string name,double start){
   if(!sensorTimings.TryGetValue(name,out var samples)){samples=new List<double>();sensorTimings[name]=samples;}
   if(samples.Count<10000)samples.Add((SensorClock()-start)*1000);
  }
  [Serializable] class SensorTimingMetric {public string name;public int count;public double mean_ms,p95_ms,max_ms;}
  [Serializable] class SensorTimingReport {public SensorTimingMetric[] metrics;public int gc_collections,pending_readbacks;public long managed_bytes;}
  void PublishSensorTimings(){
   if(SensorClock()<nextTimingReport)return;nextTimingReport=SensorClock()+1;
   var metrics=new List<SensorTimingMetric>();
   foreach(var pair in sensorTimings){var values=pair.Value;if(values.Count==0)continue;values.Sort();double sum=0;foreach(double v in values)sum+=v;
    metrics.Add(new SensorTimingMetric{name=pair.Key,count=values.Count,mean_ms=sum/values.Count,p95_ms=values[(int)((values.Count-1)*.95)],max_ms=values[values.Count-1]});values.Clear();}
   ros.Publish("/scene/sensor_timing",new StringMsg(JsonUtility.ToJson(new SensorTimingReport{metrics=metrics.ToArray(),gc_collections=GC.CollectionCount(0),pending_readbacks=asyncDepthBusy.Count,managed_bytes=GC.GetTotalMemory(false)})));
  }
 }
}

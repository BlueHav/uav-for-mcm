using System;
using UnityEngine;
using Yopo.TrackerTest;
public static class DepthSourceChecks {
 public static void Run(){
  if(DepthSourcePolicy.Parse(null)!=false||DepthSourcePolicy.Parse("unity")!=false||DepthSourcePolicy.Parse("pointcloud")!=true)throw new Exception("Depth source routing mismatch");
  foreach(var value in new[]{"","UNITY","cloud","pointcloud "}){
   bool rejected=false;try{DepthSourcePolicy.Parse(value);}catch(ArgumentException){rejected=true;}
   if(!rejected)throw new Exception("Invalid depth source accepted");
  }
  Debug.Log("DEPTH_SOURCE_CHECKS_OK");
 }
}

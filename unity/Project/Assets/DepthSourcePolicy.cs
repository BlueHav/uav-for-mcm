using System;
namespace Yopo.TrackerTest {
 public static class DepthSourcePolicy {
  public static bool Parse(string source){
   if(source==null||source=="unity")return false;
   if(source=="pointcloud")return true;
   throw new ArgumentException("YOPO_DEPTH_SOURCE must be unity or pointcloud");
  }
 }
}

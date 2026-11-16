using System;
using System.IO;
using System.Security.Cryptography;
using UnityEditor;
using UnityEditor.Build.Reporting;
public static class SceneHeartbeatBuild {
 [Serializable] class SourceHash {public string path,sha256;}
 [Serializable] class Contract {
  public int schema_version=1,sensor_service_contract=1;
  public string yopo_input_topic="/drone_{id}/yopo/input_depth",terrain_source="map_terrain_sample_not_radar";
  public string frame_field="scene_frame",sensor_frame_field="frames",dll_sha256;
  public SourceHash[] source_hashes;
 }
 static string Hash(string path){using(var sha=SHA256.Create())return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-","").ToLowerInvariant();}
 public static void Build(){
  string output=Environment.GetEnvironmentVariable("YOPO_HEARTBEAT_BUILD_OUTPUT");
  if(string.IsNullOrEmpty(output)||!Path.IsPathRooted(output)||Directory.Exists(output)||File.Exists(output))throw new Exception("A new absolute build directory is required");
  YopoInputDepthAudit.Run();
  var paths=new[]{"Assets/TrackerField.cs","Assets/TrackerFieldSensors.cs","Assets/TrackerVerticalSensors.cs","Assets/TrackerAsyncDepth.cs","Assets/FleetHud.cs","Assets/MapMarkers.cs","Assets/YopoInputDepthFrame.cs","Assets/TrackerFieldYopoInputDepth.cs","Assets/SixLineAudit.cs","Assets/Editor/YopoInputDepthAudit.cs","Assets/Editor/SceneHeartbeatBuild.cs"};
  var hashes=new SourceHash[paths.Length];for(int i=0;i<paths.Length;i++)hashes[i]=new SourceHash{path=paths[i],sha256=Hash(paths[i])};
  var report=BuildPipeline.BuildPlayer(new[]{"Assets/Scenes/PointMission.unity"},Path.Combine(output,"MountainCity.x86_64"),BuildTarget.StandaloneLinux64,BuildOptions.None);
  if(report.summary.result!=BuildResult.Succeeded)throw new Exception("SCENE_HEARTBEAT_BUILD_FAILED "+report.summary.result);
  var contract=new Contract{dll_sha256=Hash(Path.Combine(output,"MountainCity_Data/Managed/Assembly-CSharp.dll")),source_hashes=hashes};
  File.WriteAllText(Path.Combine(output,"scene-heartbeat-build.json"),UnityEngine.JsonUtility.ToJson(contract,true));
  UnityEngine.Debug.Log("SCENE_HEARTBEAT_BUILD_OK "+output);
 }
}

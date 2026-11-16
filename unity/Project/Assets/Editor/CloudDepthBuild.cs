using System;
using System.IO;
using UnityEngine;
using UnityEditor;
using UnityEditor.Build.Reporting;
public static class CloudDepthBuild {
 [Serializable] class Capabilities {public int schema=1;public string[] sources=new[]{"unity","pointcloud"};public string player_sha256,assembly_sha256;}
 static string HashFile(string path){using(var sha=System.Security.Cryptography.SHA256.Create())using(var file=File.OpenRead(path))return BitConverter.ToString(sha.ComputeHash(file)).Replace("-","").ToLowerInvariant();}
 public static void Build(){
  DepthSourceChecks.Run();
  const string output="BuildCloudDepthMission/MountainCity.x86_64";
  var report=BuildPipeline.BuildPlayer(new[]{"Assets/Scenes/PointMission.unity"},output,BuildTarget.StandaloneLinux64,BuildOptions.None);
  if(report.summary.result!=BuildResult.Succeeded)throw new Exception("Cloud depth build failed");
  File.WriteAllText("BuildCloudDepthMission/depth-source-capabilities.json",JsonUtility.ToJson(new Capabilities{player_sha256=HashFile(output),assembly_sha256=HashFile("BuildCloudDepthMission/MountainCity_Data/Managed/Assembly-CSharp.dll")},true));
  Debug.Log("CLOUD_DEPTH_BUILD_OK");
 }
}

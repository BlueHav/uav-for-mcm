using UnityEditor;
using UnityEditor.Build.Reporting;
public static class PointMissionRebuild {
 public static void Build(){
  DepthTransportChecks.Run();
  var result=BuildPipeline.BuildPlayer(new[]{"Assets/Scenes/PointMission.unity"},"BuildPointMission/MountainCity.x86_64",BuildTarget.StandaloneLinux64,BuildOptions.None);
  if(result.summary.result!=BuildResult.Succeeded)throw new System.Exception("Build failed");
  UnityEngine.Debug.Log("POINT_TCP_REBUILD_OK");
 }
}

using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
public static class HudBuild {
 public static void Build(){
  PlayerSettings.resizableWindow=true;
  var scenes=EditorBuildSettings.scenes.Where(s=>s.enabled).Select(s=>s.path).ToArray();
  var report=BuildPipeline.BuildPlayer(scenes,"BuildPreview/MountainCity.x86_64",BuildTarget.StandaloneLinux64,BuildOptions.None);
  if(report.summary.result!=BuildResult.Succeeded)throw new System.Exception("HUD_BUILD_FAILED "+report.summary.result);
  UnityEngine.Debug.Log("HUD_BUILD_OK");
 }
}

using UnityEditor;
using UnityEditor.Build.Reporting;
public static class SensorDiagnosticBuild {
    public static void Build(){
        var report=BuildPipeline.BuildPlayer(new[]{"Assets/Scenes/PointMission.unity"},"BuildSensorDiagnostic/MountainCity.x86_64",BuildTarget.StandaloneLinux64,BuildOptions.Development);
        if(report.summary.result!=BuildResult.Succeeded)throw new System.Exception("Sensor diagnostic build failed");
        UnityEngine.Debug.Log("SENSOR_DIAGNOSTIC_BUILD_OK");
    }
}

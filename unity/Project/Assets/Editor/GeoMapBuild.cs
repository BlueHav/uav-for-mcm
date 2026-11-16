using System;using System.IO;using UnityEngine;using UnityEditor;using UnityEditor.SceneManagement;using UnityEditor.Build.Reporting;using Yopo.TrackerTest;
public static class GeoMapBuild {
 public static void Build(){
  Directory.CreateDirectory("Reports");Directory.CreateDirectory("Assets/GeoGenerated");Directory.CreateDirectory("Build");
  var l=WorldLayout.Load();var scene=EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);var root=WorldGeometry.Generate(l);
  var ground=root.transform.Find("Ground_3000x500");var shader=Shader.Find("Unlit/Texture");if(shader==null)throw new Exception("Texture shader missing");
  var mat=new Material(shader);mat.mainTexture=Resources.Load<Texture2D>("GeoSatellite");mat.mainTextureScale=new Vector2(1,-1);mat.mainTextureOffset=new Vector2(0,1);ground.GetComponent<Renderer>().sharedMaterial=mat;
  int mi=0;foreach(var r in root.GetComponentsInChildren<Renderer>())foreach(var m in r.sharedMaterials)if(!AssetDatabase.Contains(m))AssetDatabase.CreateAsset(m,"Assets/GeoGenerated/Material"+(mi++)+".mat");
  foreach(var f in root.GetComponentsInChildren<MeshFilter>())if(!AssetDatabase.Contains(f.sharedMesh))AssetDatabase.CreateAsset(f.sharedMesh,"Assets/GeoGenerated/Mesh"+(mi++)+".asset");
  Physics.SyncTransforms();float error=0;int checks=0;
  var collider=ground.GetComponent<Collider>();for(int i=1;i<50;i++)for(int j=1;j<50;j++){float x=Mathf.Lerp(l.bounds.x_min,l.bounds.x_max,i/50f),y=Mathf.Lerp(l.bounds.y_min,l.bounds.y_max,j/50f);RaycastHit hit;if(!collider.Raycast(new Ray(WorldGeometry.RosToUnity(x,y,2000),Vector3.down),out hit,3000))throw new Exception("Terrain hole");error=Mathf.Max(error,Mathf.Abs(hit.point.y-TerrainSampler.Height(l,x,y)));checks++;}
  if(error>.02f)throw new Exception("Terrain mismatch "+error);
  foreach(var o in l.objects){var g=root.transform.Find(o.id);RaycastHit hit;if(!g.GetComponent<Collider>().Raycast(new Ray(WorldGeometry.RosToUnity(o.x,o.y,2000),Vector3.down),out hit,3000))throw new Exception("Building missed "+o.id);}
  var rig=new GameObject("GeoMapRuntime").AddComponent<TrackerField>();rig.enablePerson=false;rig.depthShader=AssetDatabase.LoadAssetAtPath<Shader>("Assets/LinearEyeDepth.shader");rig.gameObject.AddComponent<GoalOverlay>();rig.gameObject.AddComponent<GeoSensorAudit>().depthShader=rig.depthShader;
  var path="Assets/Scenes/GeoMap10km.unity";EditorSceneManager.SaveScene(scene,path);AssetDatabase.SaveAssets();
  File.WriteAllText("Reports/geometry-check.json","{\"terrain_rays\":"+checks+",\"max_error_m\":"+error.ToString(System.Globalization.CultureInfo.InvariantCulture)+",\"building_rays\":"+l.building_count+"}");
  PlayerSettings.productName="GeoMap10km";PlayerSettings.runInBackground=true;PlayerSettings.resizableWindow=true;
  var result=BuildPipeline.BuildPlayer(new[]{path},"BuildAbsolute/MountainCity.x86_64",BuildTarget.StandaloneLinux64,BuildOptions.None);if(result.summary.result!=BuildResult.Succeeded)throw new Exception("Build failed");Debug.Log("GEOMAP_BUILD_OK");
 }
}

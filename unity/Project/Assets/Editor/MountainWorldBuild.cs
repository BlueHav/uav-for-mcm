using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEditor.Build.Reporting;
using UnityEngine.SceneManagement;
namespace Yopo.TrackerTest.Editor {
 public static class MountainWorldBuild {
  [Serializable] class Report {public int buildings,trees,chunks,colliders,renderers;public float top,terrainRayError;public string[] scenes;}
  static void SaveAssets(GameObject root){
   foreach(var r in root.GetComponentsInChildren<Renderer>())foreach(var m in r.sharedMaterials){
    if(m==null||m.shader==null||!m.shader.isSupported)throw new Exception("Unsupported material on "+r.name);
    if(!AssetDatabase.Contains(m))AssetDatabase.CreateAsset(m,"Assets/GeneratedMaterials/"+m.name+".mat");
   }
   foreach(var f in root.GetComponentsInChildren<MeshFilter>())if(f.sharedMesh!=null&&!AssetDatabase.Contains(f.sharedMesh))AssetDatabase.CreateAsset(f.sharedMesh,"Assets/GeneratedGeometry/"+f.sharedMesh.name+".asset");
  }
  public static void Build(){
   if(!Directory.GetCurrentDirectory().Contains("MountainCity2000x1000"))throw new Exception("Wrong project");
   Directory.CreateDirectory("Assets/Scenes/MountainCity");Directory.CreateDirectory("Reports");AssetDatabase.Refresh();
   var layout=WorldLayout.Load();if(layout.bounds.x_max!=2000||layout.bounds.y_min!=-500||layout.bounds.y_max!=500)throw new Exception("Map bounds");
   var baseScene=EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
   var root=WorldGeometry.Generate(layout);NativeVegetation.Apply(root,layout);
   // Keep vegetation visible even to a high overview camera; physical geometry never unloads.
   foreach(var group in root.GetComponentsInChildren<LODGroup>()){var lod=group.GetLODs();if(lod.Length>0){lod[lod.Length-1].screenRelativeTransitionHeight=.00001f;group.SetLODs(lod);}}
   SaveAssets(root);AssetDatabase.SaveAssets();
   foreach(var mc in root.GetComponentsInChildren<MeshCollider>()){var mesh=mc.GetComponent<MeshFilter>().sharedMesh;mc.sharedMesh=null;mc.sharedMesh=mesh;}
   Physics.SyncTransforms();
   var ground=root.transform.Find("Ground_3000x500").GetComponent<Collider>();float maxErr=0;
   for(float x=2.5f;x<2000;x+=25)for(float y=-497.5f;y<500;y+=25){if(!ground.Raycast(new Ray(WorldGeometry.RosToUnity(x,y,220),Vector3.down),out var hit,230))throw new Exception("Terrain hole at "+x+","+y+" collider bounds="+ground.bounds+" vertices="+ground.GetComponent<MeshFilter>().sharedMesh.vertexCount);maxErr=Mathf.Max(maxErr,Mathf.Abs(hit.point.y-TerrainSampler.Height(layout,x,y)));}
   if(maxErr>.01f)throw new Exception("Terrain mismatch");
   foreach(var p in layout.pads)for(float x=p.x-14;x<=p.x+14;x+=4)for(float y=p.y-14;y<=p.y+14;y+=4){if(!Physics.Raycast(WorldGeometry.RosToUnity(x,y,220),Vector3.down,out var hit,230,1<<8)||Mathf.Abs(hit.point.y)>.005f)throw new Exception("Platform obstructed "+p.name);}
   var renderers=root.GetComponentsInChildren<Renderer>().Where(r=>r.enabled).ToArray();float top=renderers.Max(r=>r.bounds.max.y);
   if(top>=140)throw new Exception("Height limit "+top);
   foreach(var r in renderers){var b=r.bounds;if(b.min.z<-.05f||b.max.z>2000.05f||b.min.x<-500.05f||b.max.x>500.05f)throw new Exception("Object outside map "+r.name);}
   foreach(var o in layout.objects.Where(o=>o.kind=="building")){
    var go=root.transform.Find(o.id);if(go==null)throw new Exception("Building missing "+o.id);
    var box=go.GetComponent<BoxCollider>();if(box==null)throw new Exception("Building collider missing");
    foreach(var mf in go.GetComponentsInChildren<MeshFilter>())foreach(var v in mf.sharedMesh.vertices){var p=go.InverseTransformPoint(mf.transform.TransformPoint(v));if(Mathf.Abs(p.x-box.center.x)>box.size.x/2+.02f||Mathf.Abs(p.y-box.center.y)>box.size.y/2+.02f||Mathf.Abs(p.z-box.center.z)>box.size.z/2+.02f)throw new Exception("Visible mesh outside collider "+o.id);}
   }
   var report=new Report{buildings=layout.building_count,trees=layout.tree_count,top=top,terrainRayError=maxErr,colliders=root.GetComponentsInChildren<Collider>().Length,renderers=renderers.Length};
   string basePath="Assets/Scenes/MountainCity/WorldBase.unity";if(!EditorSceneManager.SaveScene(baseScene,basePath))throw new Exception("Base initial save failed");
   var cells=new Dictionary<string,Scene>();var objects=layout.objects.ToDictionary(o=>o.id,o=>o);
   foreach(Transform child in root.transform.Cast<Transform>().ToArray()){
    string id=child.name;WorldObject obj=null;
    if(!objects.TryGetValue(id,out obj))foreach(string suffix in new[]{"_trunk","_canopy","_terrain_mesh"})if(id.EndsWith(suffix)&&objects.TryGetValue(id.Substring(0,id.Length-suffix.Length),out obj))break;
    if(obj==null)continue;
    string key="Cell_"+Mathf.Clamp((int)(obj.x/250),0,7)+"_"+Mathf.Clamp((int)((obj.y+500)/250),0,3);
    if(!cells.ContainsKey(key)){cells[key]=EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Additive);EditorSceneManager.SaveScene(cells[key],"Assets/Scenes/MountainCity/"+key+".unity");}
    child.SetParent(null,true);SceneManager.MoveGameObjectToScene(child.gameObject,cells[key]);
   }
   if(!EditorSceneManager.SaveScene(baseScene,basePath))throw new Exception("Base save failed");
   var loadPaths=new List<string>{basePath};
   foreach(var pair in cells.OrderBy(k=>k.Key)){string path="Assets/Scenes/MountainCity/"+pair.Key+".unity";if(!EditorSceneManager.SaveScene(pair.Value,path))throw new Exception("Cell save failed");loadPaths.Add(path);}
   var boot=EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);var loader=new GameObject("MountainCityLoader").AddComponent<MountainBootstrap>();loader.scenes=loadPaths.ToArray();loader.depthShader=AssetDatabase.LoadAssetAtPath<Shader>("Assets/LinearEyeDepth.shader");
   string bootPath="Assets/Scenes/MountainCity/MountainCity2000x1000.unity";EditorSceneManager.SaveScene(boot,bootPath);
   var all=new[]{bootPath}.Concat(loadPaths).ToArray();EditorBuildSettings.scenes=all.Select(p=>new EditorBuildSettingsScene(p,true)).ToArray();report.chunks=cells.Count;report.scenes=all;
   File.WriteAllText("Reports/build-geometry.json",JsonUtility.ToJson(report,true));
   PlayerSettings.productName="MountainCity2000x1000";PlayerSettings.companyName="YOPO";PlayerSettings.runInBackground=true;PlayerSettings.defaultScreenWidth=1440;PlayerSettings.defaultScreenHeight=900;PlayerSettings.fullScreenMode=FullScreenMode.Windowed;
   PlayerSettings.SetGraphicsAPIs(BuildTarget.StandaloneLinux64,new[]{UnityEngine.Rendering.GraphicsDeviceType.Vulkan,UnityEngine.Rendering.GraphicsDeviceType.OpenGLCore});
   Directory.CreateDirectory("Build");AssetDatabase.SaveAssets();
   var result=BuildPipeline.BuildPlayer(new BuildPlayerOptions{scenes=all,locationPathName=Path.GetFullPath("Build/MountainCity.x86_64"),target=BuildTarget.StandaloneLinux64,options=BuildOptions.CompressWithLz4});
   if(result.summary.result!=BuildResult.Succeeded)throw new Exception("Build failed "+result.summary.result);
   Debug.Log("MOUNTAIN_BUILD_PASS bytes="+result.summary.totalSize+" chunks="+cells.Count+" buildings="+report.buildings+" trees="+report.trees);
  }
 }
}


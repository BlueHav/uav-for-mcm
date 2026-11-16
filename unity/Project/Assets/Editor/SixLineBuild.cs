using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEditor.Build.Reporting;
using Yopo.TrackerTest;
using Yopo.TrackerTest.Editor;
public static class SixLineBuild {
 [Serializable] class Points {public Point start,landing,end,end2;}
 [Serializable] class Point {public float x,y,ground_z;}
 [Serializable] class PointFile {public Points points;}
 [Serializable] class TreeList {public WorldObject[] objects;}
 public static void Build(){
  if(File.Exists("Reports/scene-check.json")){BuildOnly();return;}
  Directory.CreateDirectory("Reports");
  var scene=EditorSceneManager.OpenScene("Assets/Scenes/PointMission.unity");
  var original=WorldLayout.Load();var ground=GameObject.Find("Ground_3000x500");
  if(ground==null)throw new Exception("Original ground missing");
  var originalMesh=ground.GetComponent<MeshFilter>().sharedMesh;
  string meshPath=AssetDatabase.GetAssetPath(originalMesh);string meshHash=Hash(meshPath);
  var points=JsonUtility.FromJson<PointFile>(File.ReadAllText("../../config/point.json")).points;
  var root=new GameObject("AddedTerminalForests_20261108");var list=new List<WorldObject>();
  var bark=WorldGeometry.Material("AddedBark",new Color(.32f,.20f,.12f));var leaves=WorldGeometry.Material("AddedLeaves",new Color(.14f,.37f,.17f));
  AssetDatabase.CreateAsset(bark,"Assets/AddedBark.mat");AssetDatabase.CreateAsset(leaves,"Assets/AddedLeaves.mat");
  for(int group=0;group<2;group++){
   var end=group==0?points.end:points.end2;Vector2 start=new Vector2(points.landing.x,points.landing.y),finish=new Vector2(end.x,end.y),direction=(finish-start).normalized,side=new Vector2(-direction.y,direction.x);int added=0;
   for(int k=0;k<120&&added<24;k++){
    float fraction=.23f+(k%12)*.047f;float lateral=(k%2==0?1:-1)*(25+(k/12)*9);
    Vector2 p=Vector2.Lerp(start,finish,fraction)+side*lateral;
    if(original.objects.Any(o=>Vector2.Distance(p,new Vector2(o.x,o.y))<Mathf.Max(10,Mathf.Sqrt(o.width*o.width+o.depth*o.depth)/2+8)))continue;
    if(list.Any(o=>Vector2.Distance(p,new Vector2(o.x,o.y))<9))continue;
    float z=TerrainSampler.Height(original,p.x,p.y);
    var o2=new WorldObject{kind="tree",id="TerminalForest_G"+(group+1)+"_"+added,x=p.x,y=p.y,base_z=z,bottom_z=z,height=10+(added%4),radius=3.2f};
    list.Add(o2);added++;
    Make(PrimitiveType.Cylinder,o2.id+"_trunk",WorldGeometry.RosToUnity(p.x,p.y,z+o2.height*.275f),new Vector3(.55f,o2.height*.275f,.55f),bark,root.transform);
    Make(PrimitiveType.Sphere,o2.id+"_canopy",WorldGeometry.RosToUnity(p.x,p.y,z+o2.height*.7f),new Vector3(o2.radius*2,o2.height*.6f,o2.radius*2),leaves,root.transform);
   }
   if(added!=24)throw new Exception("Cannot place forest group "+group+": "+added);
  }
  NativeVegetation.Apply(root,new WorldLayout{objects=list.ToArray(),tree_count=list.Count});
  Physics.SyncTransforms();foreach(var o in list){var trunk=root.transform.Find(o.id+"_trunk");if(trunk==null||trunk.GetComponent<Collider>()==null||trunk.gameObject.layer!=WorldGeometry.ObstacleLayer)throw new Exception("Tree collider missing");}
  string worldPath="Assets/Resources/TrackerWorld.json";string json=File.ReadAllText(worldPath);
  string entries=string.Join(",",list.Select(o=>JsonUtility.ToJson(o)).ToArray());
  var pattern=new Regex("(\"objects\"\\s*:\\s*\\[)");if(pattern.Matches(json).Count!=1)throw new Exception("Ambiguous world objects");
  json=pattern.Replace(json,m=>m.Value+entries+",",1);
  json=Regex.Replace(json,"(\"tree_count\"\\s*:\\s*)[0-9]+",m=>m.Groups[1].Value+(original.tree_count+list.Count));
  File.WriteAllText(worldPath,json);File.WriteAllText("Reports/added-trees.json",JsonUtility.ToJson(new TreeList{objects=list.ToArray()},true));
  EditorSceneManager.SaveScene(scene,"Assets/Scenes/PointMission.unity");AssetDatabase.SaveAssets();AssetDatabase.Refresh();
  if(Hash(meshPath)!=meshHash)throw new Exception("Original ground modified");
  File.WriteAllText("Reports/scene-check.json","{\"added_trees\":48,\"groups\":2,\"terrain_sha256\":\""+meshHash+"\",\"terrain_unchanged\":true,\"original_objects\":"+original.objects.Length+"}");
  BuildOnly();
 }
 public static void BuildOnly(){
  var report=BuildPipeline.BuildPlayer(new[]{"Assets/Scenes/PointMission.unity"},"BuildSixLine/MountainCity.x86_64",BuildTarget.StandaloneLinux64,BuildOptions.None);
  if(report.summary.result!=BuildResult.Succeeded)throw new Exception("Build failed "+report.summary.result);
  File.WriteAllText("BuildSixLine/depth-source-capabilities.json","{\"schema\":1,\"sources\":[\"unity\",\"pointcloud\"],\"player_sha256\":\""+Hash("BuildSixLine/MountainCity.x86_64")+"\",\"assembly_sha256\":\""+Hash("BuildSixLine/MountainCity_Data/Managed/Assembly-CSharp.dll")+"\"}");
  Debug.Log("SIX_LINE_BUILD_OK");
 }
 static void Make(PrimitiveType type,string name,Vector3 p,Vector3 scale,Material material,Transform parent){var g=GameObject.CreatePrimitive(type);g.name=name;g.transform.SetParent(parent);g.transform.position=p;g.transform.localScale=scale;g.layer=WorldGeometry.ObstacleLayer;g.GetComponent<Renderer>().sharedMaterial=material;}
 static string Hash(string path){using(var sha=System.Security.Cryptography.SHA256.Create())using(var stream=File.OpenRead(path))return BitConverter.ToString(sha.ComputeHash(stream)).Replace("-","").ToLowerInvariant();}
}

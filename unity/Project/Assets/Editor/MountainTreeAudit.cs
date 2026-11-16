using System;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEditor.SceneManagement;
namespace Yopo.TrackerTest.Editor {
 public static class MountainTreeAudit {
  [Serializable] class Result {public int trees,trunk_colliders,foliage_colliders,surface_triangles;public float surface_max_error_m;}
  public static void Check(){
   EditorSceneManager.OpenScene("Assets/Scenes/MountainCity/WorldBase.unity",OpenSceneMode.Single);
   foreach(string p in Directory.GetFiles("Assets/Scenes/MountainCity","Cell_*.unity"))EditorSceneManager.OpenScene(p,OpenSceneMode.Additive);
   Physics.SyncTransforms();var d=WorldLayout.Load();var result=new Result();
   foreach(var o in d.objects.Where(o=>o.kind=="tree")){
    var root=GameObject.Find(o.id+"_terrain_mesh");var trunk=GameObject.Find(o.id+"_trunk").GetComponent<Collider>();var canopy=GameObject.Find(o.id+"_canopy");result.trees++;
    if(trunk==null||!trunk.enabled)throw new Exception("Missing trunk collider "+o.id);
    if(!trunk.Raycast(new Ray(trunk.bounds.center+Vector3.up*(trunk.bounds.extents.y+5),Vector3.down),out var hit,trunk.bounds.size.y+10))throw new Exception("Trunk ray miss "+o.id);
    result.trunk_colliders++;result.foliage_colliders+=canopy.GetComponents<Collider>().Length+root.GetComponentsInChildren<Collider>().Length;
   }
   for(int layer=0;layer<4;layer++){
    var mesh=GameObject.Find("CityRoadSurface_"+layer).GetComponent<MeshFilter>().sharedMesh;var vertices=mesh.vertices;var triangles=mesh.triangles;float lift=layer==0?.015f:layer==1?.025f:layer==2?.04f:.06f;
    for(int i=0;i<triangles.Length;i+=3){var point=(vertices[triangles[i]]+vertices[triangles[i+1]]+vertices[triangles[i+2]])/3;var ros=WorldGeometry.UnityToRos(point);float error=Mathf.Abs(ros.z-TerrainSampler.Height(d,ros.x,ros.y)-lift);result.surface_max_error_m=Mathf.Max(result.surface_max_error_m,error);result.surface_triangles++;}
   }
   File.WriteAllText("Reports/tree-coverage.json",JsonUtility.ToJson(result,true));Debug.Log("TREE_COVERAGE "+JsonUtility.ToJson(result));if(result.foliage_colliders>0)throw new Exception("Foliage should not collide: "+result.foliage_colliders);
   if(result.surface_max_error_m>.003f)throw new Exception("Surface off terrain "+result.surface_max_error_m);
  }
 }
}


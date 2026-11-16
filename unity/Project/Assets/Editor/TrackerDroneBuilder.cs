using System;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEditor;
namespace Yopo.TrackerTest.Editor {
 public static class TrackerDroneBuilder {
  [Serializable] class Part {public float[] color,vertices,normals;public int[] triangles;}
  [Serializable] class Data {public Part[] parts;public float max_radius_m;public int triangles;}
  public static void Build(){
   var data=JsonUtility.FromJson<Data>(File.ReadAllText("Assets/SourceModels/TrackerDroneMesh.json"));
   Directory.CreateDirectory("Assets/GeneratedGeometry");Directory.CreateDirectory("Assets/Resources");AssetDatabase.Refresh();
   var root=new GameObject("DroneVisual_Tracker");root.layer=10;int index=0,total=0;float maxRadius=0;
   foreach(var part in data.parts){
    var vertices=new Vector3[part.vertices.Length/3];var normals=new Vector3[vertices.Length];
    for(int i=0;i<vertices.Length;i++){
     vertices[i]=new Vector3(part.vertices[i*3],part.vertices[i*3+1],part.vertices[i*3+2]);
     normals[i]=new Vector3(part.normals[i*3],part.normals[i*3+1],part.normals[i*3+2]);maxRadius=Mathf.Max(maxRadius,vertices[i].magnitude);
    }
    var mesh=new Mesh{name="TrackerDrone_"+index,indexFormat=UnityEngine.Rendering.IndexFormat.UInt32};
    mesh.vertices=vertices;mesh.normals=normals;mesh.triangles=part.triangles;mesh.RecalculateBounds();total+=part.triangles.Length/3;
    string path="Assets/GeneratedGeometry/TrackerDrone_"+index+".asset";AssetDatabase.DeleteAsset(path);AssetDatabase.CreateAsset(mesh,path);
    var material=WorldGeometry.Material("TrackerDrone_"+index,new Color(part.color[0],part.color[1],part.color[2],part.color[3]));material.SetFloat("_Cull",0);
    path="Assets/GeneratedMaterials/TrackerDrone_"+index+".mat";AssetDatabase.DeleteAsset(path);AssetDatabase.CreateAsset(material,path);
    var g=new GameObject("TrackerMesh_"+index++);g.layer=10;g.transform.SetParent(root.transform,false);g.AddComponent<MeshFilter>().sharedMesh=mesh;g.AddComponent<MeshRenderer>().sharedMaterial=material;
   }
   if(total!=data.triangles||maxRadius>.35f||root.GetComponentsInChildren<Collider>().Length!=0)throw new Exception("Drone mesh safety check failed");
   PrefabUtility.SaveAsPrefabAsset(root,"Assets/Resources/TrackerDrone.prefab");
   Debug.Log("TRACKER_DRONE_VALIDATED triangles="+total+" materials="+index+" maxRadius="+maxRadius+" layer=10 colliders=0");
   UnityEngine.Object.DestroyImmediate(root);
  }
 }
}

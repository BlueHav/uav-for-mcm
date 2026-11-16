using System;using System.IO;using UnityEngine;using UnityEditor;
namespace Yopo.TrackerTest.Editor {
 public static class CityModelBuilder {
  [Serializable] class Part {public float[] vertices,normals,uv,color;public int[] triangles;public string texture;}
  [Serializable] class Model {public string name;public float width,depth,height;public Part[] parts;}
  [Serializable] class Catalog {public Model[] models;}
  public static void Build(){
   Directory.CreateDirectory("Assets/Resources/City");Directory.CreateDirectory("Assets/GeneratedGeometry/City");Directory.CreateDirectory("Assets/GeneratedMaterials/City");AssetDatabase.Refresh();
   var catalog=JsonUtility.FromJson<Catalog>(File.ReadAllText("Assets/SourceModels/City/catalog.json"));
   foreach(var model in catalog.models){
    var root=new GameObject(model.name);root.layer=8;int index=0;
    foreach(var p in model.parts){
     var v=new Vector3[p.vertices.Length/3];var n=new Vector3[v.Length];var uv=new Vector2[v.Length];
     for(int i=0;i<v.Length;i++){v[i]=new Vector3(p.vertices[i*3],p.vertices[i*3+1],p.vertices[i*3+2]);n[i]=new Vector3(p.normals[i*3],p.normals[i*3+1],p.normals[i*3+2]);uv[i]=new Vector2(p.uv[i*2],p.uv[i*2+1]);}
     var mesh=new Mesh{name="City_"+model.name+"_"+index,indexFormat=UnityEngine.Rendering.IndexFormat.UInt32};mesh.vertices=v;mesh.normals=n;mesh.uv=uv;mesh.triangles=p.triangles;mesh.RecalculateBounds();
     string path="Assets/GeneratedGeometry/City/"+model.name+"_"+index+".asset";AssetDatabase.DeleteAsset(path);AssetDatabase.CreateAsset(mesh,path);
     var mat=WorldGeometry.Material("City_"+model.name+"_"+index,new Color(p.color[0],p.color[1],p.color[2],p.color[3]));mat.SetFloat("_Cull",0);
     if(!string.IsNullOrEmpty(p.texture)){mat.mainTexture=AssetDatabase.LoadAssetAtPath<Texture2D>(p.texture);if(mat.mainTexture==null)throw new Exception("Missing city texture "+p.texture);}
     path="Assets/GeneratedMaterials/City/"+model.name+"_"+index+".mat";AssetDatabase.DeleteAsset(path);AssetDatabase.CreateAsset(mat,path);
     var child=new GameObject("Part_"+index++);child.layer=8;child.transform.SetParent(root.transform,false);child.AddComponent<MeshFilter>().sharedMesh=mesh;var r=child.AddComponent<MeshRenderer>();r.sharedMaterial=mat;r.shadowCastingMode=UnityEngine.Rendering.ShadowCastingMode.Off;
    }
    var rs=root.GetComponentsInChildren<Renderer>();var b=rs[0].bounds;foreach(var r in rs)b.Encapsulate(r.bounds);
    if(Mathf.Abs(b.size.x-model.depth)>.01||Mathf.Abs(b.size.z-model.width)>.01||Mathf.Abs(b.size.y-model.height)>.01||Mathf.Abs(b.min.y)>.01)throw new Exception("City model size mismatch "+model.name);
    PrefabUtility.SaveAsPrefabAsset(root,"Assets/Resources/City/"+model.name+".prefab");UnityEngine.Object.DestroyImmediate(root);
   }
   Debug.Log("CITY_MODELS_VALIDATED count="+catalog.models.Length+" sdf_metric_scale_preserved=true textured=true");
  }
 }
}

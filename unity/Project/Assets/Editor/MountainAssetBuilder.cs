using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
namespace Yopo.TrackerTest.Editor {
 public static class MountainAssetBuilder {
  [Serializable] public class Entry {public string name,source;public float width,depth,height;public int triangles,materials;}
  [Serializable] public class Catalog {public Entry[] models;}
  static Dictionary<Material,Material> materials=new Dictionary<Material,Material>();
  static Material Convert(Material src){
   if(src==null)throw new Exception("Missing source material");
   if(materials.TryGetValue(src,out var known))return known;
   var m=new Material(Shader.Find("Yopo/MountainSurface")){name="MC_"+src.name,enableInstancing=true};
   var so=new SerializedObject(src);var tex=so.FindProperty("m_SavedProperties.m_TexEnvs");
   Texture albedo=null,normal=null,emission=null;Vector2 scale=Vector2.one,offset=Vector2.zero;
   for(int i=0;i<tex.arraySize;i++){
    var pair=tex.GetArrayElementAtIndex(i);string key=pair.FindPropertyRelative("first").stringValue;
    var value=pair.FindPropertyRelative("second.m_Texture").objectReferenceValue as Texture;
    if(value==null)continue;
    if(key=="_BaseMap"||key=="_MainTex"||key=="_Diffuse_Map"||key=="_DiffuseMap"||key=="_Base_Map"||(albedo==null&&(key.ToLower().Contains("albedo")||key.ToLower().Contains("basecolor")))){albedo=value;scale=pair.FindPropertyRelative("second.m_Scale").vector2Value;offset=pair.FindPropertyRelative("second.m_Offset").vector2Value;}
    if(key=="_BumpMap"||key=="_Normal_Map"||key=="_NormalMap")normal=value;if(key=="_EmissionMap"||key=="_Emissive_Map")emission=value;
   }
   Color color=Color.white,em=Color.black;var colors=so.FindProperty("m_SavedProperties.m_Colors");
   for(int i=0;i<colors.arraySize;i++){var p=colors.GetArrayElementAtIndex(i);var key=p.FindPropertyRelative("first").stringValue;if(key=="_BaseColor"||key=="_Color"||key=="_Recolor")color=p.FindPropertyRelative("second").colorValue;if(key=="_EmissionColor"||key=="_Emissive_Recolor")em=p.FindPropertyRelative("second").colorValue;}
   float emissionStrength=.15f;var floats=so.FindProperty("m_SavedProperties.m_Floats");for(int i=0;i<floats.arraySize;i++){var pair=floats.GetArrayElementAtIndex(i);if(pair.FindPropertyRelative("first").stringValue=="_Emissive_Strength")emissionStrength=Mathf.Clamp(pair.FindPropertyRelative("second").floatValue,0,.3f);}
   color.a=1;m.color=color;m.mainTexture=albedo;m.mainTextureScale=scale;m.mainTextureOffset=offset;
   m.SetTexture("_BumpMap",normal);m.SetFloat("_NormalStrength",normal==null?0:.35f);m.SetTexture("_EmissionMap",emission);m.SetColor("_EmissionColor",emission==null?Color.black:em*emissionStrength);m.SetFloat("_Cull",0);
   string guid=AssetDatabase.AssetPathToGUID(AssetDatabase.GetAssetPath(src));
   string path="Assets/GeneratedMaterials/MC_"+guid+".mat";var existing=AssetDatabase.LoadAssetAtPath<Material>(path);
   if(existing!=null){EditorUtility.CopySerialized(m,existing);UnityEngine.Object.DestroyImmediate(m);m=existing;EditorUtility.SetDirty(m);}else AssetDatabase.CreateAsset(m,path);materials[src]=m;return m;
  }
  public static void RepairAndBuild(){
   foreach(string path in Directory.GetFiles("Assets/GeneratedMaterials","MC_*.mat")){string guid=Path.GetFileNameWithoutExtension(path).Substring(3);var src=AssetDatabase.LoadAssetAtPath<Material>(AssetDatabase.GUIDToAssetPath(guid));if(src!=null)Convert(src);}
   AssetDatabase.SaveAssets();Debug.Log("MOUNTAIN_MATERIAL_REPAIR count="+materials.Count);MountainWorldBuild.Build();
  }
  public static void Build(){
   Directory.CreateDirectory("Assets/GeneratedGeometry/ModernCity");Directory.CreateDirectory("Assets/GeneratedMaterials");Directory.CreateDirectory("Assets/Resources/City");Directory.CreateDirectory("Reports");AssetDatabase.Refresh();
   var entries=new List<Entry>();
   string basePath="Assets/LeartesSudios/Stylized Sci-Fi Modern City/Art/Blueprints";
   foreach(string path in Directory.GetFiles(basePath,"BP_*.prefab").OrderBy(x=>x)){
    string basename=Path.GetFileNameWithoutExtension(path);
    if(!(basename.StartsWith("BP_B1_")||basename.StartsWith("BP_B2_")||basename.StartsWith("BP_B3_")||basename.StartsWith("BP_B4_")||basename.StartsWith("BP_SS")))continue;
    var prefab=AssetDatabase.LoadAssetAtPath<GameObject>(path);if(prefab==null)throw new Exception("Prefab missing "+path);
    var instance=UnityEngine.Object.Instantiate(prefab);instance.transform.SetPositionAndRotation(Vector3.zero,Quaternion.identity);
    var rs=instance.GetComponentsInChildren<MeshRenderer>().Where(r=>r.enabled&&r.GetComponent<MeshFilter>()!=null&&r.GetComponent<MeshFilter>().sharedMesh!=null).ToArray();
    if(rs.Length==0)throw new Exception("No building meshes "+path);
    var bounds=rs[0].bounds;foreach(var r in rs.Skip(1))bounds.Encapsulate(r.bounds);
    var shift=new Vector3(-bounds.center.x,-bounds.min.y,-bounds.center.z);
    // One meter/unit normalization: cap oversized skyscrapers while retaining proportions.
    float factor=Mathf.Min(1,65/bounds.size.y,48/Mathf.Max(bounds.size.x,bounds.size.z));
    var groups=new Dictionary<Material,List<CombineInstance>>();
    foreach(var r in rs){var mesh=r.GetComponent<MeshFilter>().sharedMesh;for(int j=0;j<mesh.subMeshCount;j++){
     var mat=Convert(r.sharedMaterials[Mathf.Min(j,r.sharedMaterials.Length-1)]);
     if(!groups.ContainsKey(mat))groups[mat]=new List<CombineInstance>();
     groups[mat].Add(new CombineInstance{mesh=mesh,subMeshIndex=j,transform=Matrix4x4.Scale(Vector3.one*factor)*Matrix4x4.Translate(shift)*r.transform.localToWorldMatrix});
    }}
    string name="new_"+basename;var root=new GameObject(name);root.layer=8;int count=0,tri=0;
    foreach(var group in groups){var mesh=new Mesh{name=name+"_"+count,indexFormat=UnityEngine.Rendering.IndexFormat.UInt32};mesh.CombineMeshes(group.Value.ToArray(),true,true);mesh.RecalculateBounds();tri+=mesh.triangles.Length/3;
     AssetDatabase.CreateAsset(mesh,"Assets/GeneratedGeometry/ModernCity/"+mesh.name+".asset");
     var part=new GameObject("Part"+count++);part.layer=8;part.transform.SetParent(root.transform,false);part.AddComponent<MeshFilter>().sharedMesh=mesh;var mr=part.AddComponent<MeshRenderer>();mr.sharedMaterial=group.Key;mr.shadowCastingMode=UnityEngine.Rendering.ShadowCastingMode.Off;
    }
    var actual=root.GetComponentsInChildren<Renderer>();var b=actual[0].bounds;foreach(var r in actual.Skip(1))b.Encapsulate(r.bounds);
    if(Mathf.Abs(b.min.y)>.02f||b.size.y<=1||b.size.x<=1||b.size.z<=1)throw new Exception("Invalid normalized bounds "+name);
    PrefabUtility.SaveAsPrefabAsset(root,"Assets/Resources/City/"+name+".prefab");
    entries.Add(new Entry{name=name,source=path,width=b.size.z,depth=b.size.x,height=b.size.y,triangles=tri,materials=count});
    UnityEngine.Object.DestroyImmediate(instance);UnityEngine.Object.DestroyImmediate(root);
   }
   if(entries.Count<10)throw new Exception("Insufficient city models");
   File.WriteAllText("Reports/catalog.json",JsonUtility.ToJson(new Catalog{models=entries.ToArray()},true));AssetDatabase.SaveAssets();Debug.Log("MOUNTAIN_CATALOG_PASS models="+entries.Count+" materials="+materials.Count);
  }
 }
}


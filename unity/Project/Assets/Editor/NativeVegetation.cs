using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
namespace Yopo.TrackerTest.Editor
{
    public static class NativeVegetation
    {
        static Dictionary<Material,Material> cache;
        static Material Convert(Material source)
        {
            if(cache.TryGetValue(source,out var existing))return existing;
            var material=WorldGeometry.Material("Terrain_"+source.name,Color.white);
            material.SetFloat("_Cull",0);material.SetFloat("_Cutoff",.3f);
            var serialized=new SerializedObject(source);
            var textures=serialized.FindProperty("m_SavedProperties.m_TexEnvs");
            for(int i=0;i<textures.arraySize;i++)
            {
                var pair=textures.GetArrayElementAtIndex(i);
                if(pair.FindPropertyRelative("first").stringValue=="_MainTex")
                {
                    material.mainTexture=pair.FindPropertyRelative("second.m_Texture").objectReferenceValue as Texture;
                    material.mainTextureScale=pair.FindPropertyRelative("second.m_Scale").vector2Value;
                    material.mainTextureOffset=pair.FindPropertyRelative("second.m_Offset").vector2Value;
                }
            }
            var colors=serialized.FindProperty("m_SavedProperties.m_Colors");
            for(int i=0;i<colors.arraySize;i++)
            {
                var pair=colors.GetArrayElementAtIndex(i);
                if(pair.FindPropertyRelative("first").stringValue=="_Color")material.color=pair.FindPropertyRelative("second").colorValue;
            }
            if(material.mainTexture==null)throw new Exception("Tree albedo texture not resolved: "+source.name);
            var guid=AssetDatabase.AssetPathToGUID(AssetDatabase.GetAssetPath(source));
            string name="Assets/GeneratedMaterials/Terrain_"+source.name.Replace("/","_")+"_"+guid.Substring(0,8)+".mat";
            AssetDatabase.DeleteAsset(name);AssetDatabase.CreateAsset(material,name);cache[source]=material;
            return material;
        }
        static GameObject Template(string variant)
        {
            string dir="Assets/TerrainDemoScene_URP/Prefabs/Trees/Pines/Pine_"+variant+"/Pine_"+variant;
            var source=AssetDatabase.LoadAssetAtPath<GameObject>(dir+".prefab");
            if(source==null)source=AssetDatabase.LoadAssetAtPath<GameObject>(dir+".st");
            if(source==null)throw new Exception("Pine asset not imported: "+dir);
            var t=UnityEngine.Object.Instantiate(source);t.name="Template_"+variant;
            t.transform.position=Vector3.zero;t.transform.rotation=Quaternion.identity;t.transform.localScale=Vector3.one;
            foreach(var c in t.GetComponentsInChildren<Collider>())UnityEngine.Object.DestroyImmediate(c);
            foreach(var m in t.GetComponentsInChildren<MonoBehaviour>())if(m!=null)UnityEngine.Object.DestroyImmediate(m);
            var group=t.GetComponentInChildren<LODGroup>();
            Renderer[] keep=group!=null ? group.GetLODs()[Math.Min(1,group.GetLODs().Length-1)].renderers : t.GetComponentsInChildren<MeshRenderer>();
            if(keep.Length==0||keep.Any(r=>!(r is MeshRenderer)))throw new Exception("No usable tree mesh LOD: "+variant);
            var kept=new HashSet<Renderer>(keep);
            foreach(var r in t.GetComponentsInChildren<Renderer>())
            {
                if(!kept.Contains(r)){var unusedMesh=r.GetComponent<MeshFilter>();UnityEngine.Object.DestroyImmediate(r);if(unusedMesh!=null)UnityEngine.Object.DestroyImmediate(unusedMesh);continue;}
                r.sharedMaterials=r.sharedMaterials.Select(Convert).ToArray();
                r.shadowCastingMode=UnityEngine.Rendering.ShadowCastingMode.Off;r.enabled=true;
            }
            foreach(var old in t.GetComponentsInChildren<LODGroup>())UnityEngine.Object.DestroyImmediate(old);
            group=t.AddComponent<LODGroup>();group.SetLODs(new[]{new LOD(.04f,keep)});group.RecalculateBounds();
            return t;
        }
        public static void Apply(GameObject root,WorldLayout layout)
        {
            cache=new Dictionary<Material,Material>();
            var templates=new[]{"A","B","C","D"}.Select(Template).ToArray();int count=0;
            foreach(var o in layout.objects.Where(x=>x.kind=="tree"))
            {
                var parent=new GameObject(o.id+"_terrain_mesh");parent.transform.parent=root.transform;
                parent.transform.position=Vector3.zero;
                var t=UnityEngine.Object.Instantiate(templates[count%4],parent.transform);t.name="Pine";
                var renderers=t.GetComponentsInChildren<Renderer>();
                var b=renderers[0].bounds;foreach(var r in renderers.Skip(1))b.Encapsulate(r.bounds);
                float radius=Mathf.Sqrt(b.extents.x*b.extents.x+b.extents.z*b.extents.z);
                float scale=Mathf.Min(o.height/b.size.y,o.radius/radius);
                t.transform.localScale=Vector3.one*scale;
                t.transform.localPosition=new Vector3(-b.center.x*scale,-b.min.y*scale,-b.center.z*scale);
                parent.transform.rotation=Quaternion.Euler(0,(count*137.508f)%360,0);
                parent.transform.position=WorldGeometry.RosToUnity(o.x,o.y,o.base_z);
                float actualHeight=b.size.y*scale,actualRadius=radius*scale;
                var trunk=root.transform.Find(o.id+"_trunk");var canopy=root.transform.Find(o.id+"_canopy");
                trunk.GetComponent<Renderer>().enabled=false;canopy.GetComponent<Renderer>().enabled=false;
                trunk.localScale=new Vector3(.55f,actualHeight*.275f,.55f);trunk.position=WorldGeometry.RosToUnity(o.x,o.y,o.base_z+actualHeight*.275f);
                // User-requested trunk-only collision.
                UnityEngine.Object.DestroyImmediate(canopy.GetComponent<Collider>());
                canopy.localScale=Vector3.one;canopy.position=WorldGeometry.RosToUnity(o.x,o.y,o.base_z+actualHeight*.5f);
                // Foliage is visual only; keep trunk as the solid obstacle.
                count++;
            }
            foreach(var t in templates)UnityEngine.Object.DestroyImmediate(t);
            if(count!=layout.tree_count)throw new Exception("Tree upgrade count mismatch");
            Debug.Log("TERRAIN_TREES_APPLIED count="+count+" variants=4 converted_materials="+cache.Count);
        }
    }
}


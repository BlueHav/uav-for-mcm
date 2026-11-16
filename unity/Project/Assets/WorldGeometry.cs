using System;
using System.Collections.Generic;
using UnityEngine;

namespace Yopo.TrackerTest
{
    [Serializable] public class WorldObject
    {
        public string kind,id,model,district;
        public float yaw;
        public float x,y,height,width,depth,radius,base_z,bottom_z;
    }
    [Serializable] public class Plaza {public string name;public float x,y,radius;}
    [Serializable] public class PadData {public string name;public float x,y,z,width,depth;}
    [Serializable] public class NavigationData { public int width,height; public float cell; public string blocked; }
    [Serializable] public class FlightBounds { public float x_min,x_max,y_min,y_max,z_min,z_max; }
    [Serializable] public class WorldLayout
    {
        public int schema,seed,tree_count,building_count;
        public float spawn_clearance;
        public FlightBounds bounds;
        public WorldObject[] objects;
        public NavigationData navigation;
        public HeightField terrain;
        public SpawnPoint spawn;
        public PadData[] pads;
        public DistrictData[] districts;public RoadData[] roads;public Plaza[] plazas;
        public Vector3 Spawn => spawn==null?new Vector3(45,-50,2):new Vector3(spawn.x,spawn.y,spawn.z);
        public static WorldLayout Load() => JsonUtility.FromJson<WorldLayout>(Resources.Load<TextAsset>("TrackerWorld").text);
    }
    public static class WorldGeometry
    {
        public const int ObstacleLayer=8;
        public static Vector3 RosToUnity(float x,float y,float z) => new Vector3(-y,z,x);
        public static Vector3 UnityToRos(Vector3 p) => new Vector3(p.z,-p.x,p.y);
        public static Material Material(string name,Color color)
        {
            var shader=Shader.Find("Yopo/WorldColor");
            if(shader==null) throw new Exception("WorldColor shader missing");
            var m=new Material(shader){name=name,enableInstancing=true};
            m.color=color; return m;
        }
        public static GameObject Primitive(PrimitiveType type,string name,Vector3 p,Vector3 scale,Material mat,Transform parent,int layer=ObstacleLayer)
        {
            var g=GameObject.CreatePrimitive(type);g.name=name;g.transform.SetParent(parent,false);
            g.transform.localPosition=p;g.transform.localScale=scale;g.layer=layer;
            if(type==PrimitiveType.Sphere){ UnityEngine.Object.DestroyImmediate(g.GetComponent<Collider>()); g.AddComponent<MeshCollider>().sharedMesh=g.GetComponent<MeshFilter>().sharedMesh; }
            var r=g.GetComponent<Renderer>();r.sharedMaterial=mat;
            r.shadowCastingMode=UnityEngine.Rendering.ShadowCastingMode.Off;
            return g;
        }
        public static GameObject Generate(WorldLayout layout)
        {
            MountainPalette.Clear();var root=new GameObject("World_3000x500_Altitude_0_200");
            var elevationShader=Shader.Find("Yopo/ElevationColor");if(elevationShader==null)throw new Exception("Elevation shader missing");
            var ground=new Material(elevationShader){name="GroundElevation"};
            var trunk=Material("Bark",new Color(.32f,.20f,.12f));
            var leaves=Material("Leaves",new Color(.14f,.37f,.17f));
            var wall=Material("Buildings",new Color(.55f,.58f,.62f));
            var roof=Material("Roofs",new Color(.30f,.29f,.30f));
            if(layout.terrain!=null) HeightFieldMesh.Create(layout.terrain,layout.bounds,root.transform,ground);
            else Primitive(PrimitiveType.Cube,"Ground_3000x500",RosToUnity((layout.bounds.x_min+layout.bounds.x_max)/2,(layout.bounds.y_min+layout.bounds.y_max)/2,-.5f),new Vector3(layout.bounds.y_max-layout.bounds.y_min,1,layout.bounds.x_max-layout.bounds.x_min),ground,root.transform);
            foreach(var o in layout.objects)
            {
                var p=RosToUnity(o.x,o.y,o.base_z);
                if(o.kind=="tree")
                {
                    float stem=o.height*.55f;
                    Primitive(PrimitiveType.Cylinder,o.id+"_trunk",p+Vector3.up*stem*.5f,
                        new Vector3(.55f,stem*.5f,.55f),trunk,root.transform);
                    // Non-uniform sphere collider follows the same ellipsoid transform as the canopy.
                    Primitive(PrimitiveType.Sphere,o.id+"_canopy",p+Vector3.up*(o.height*.70f),
                        new Vector3(o.radius*2,o.height*.60f,o.radius*2),leaves,root.transform);
                }
                else if(!string.IsNullOrEmpty(o.model)){
                    var prefab=Resources.Load<GameObject>("City/"+o.model);if(prefab==null)throw new Exception("City prefab missing: "+o.model);
                    var visual=UnityEngine.Object.Instantiate(prefab,root.transform,false);visual.name=o.id;visual.transform.position=p;visual.transform.rotation=Quaternion.Euler(0,-o.yaw,0);
                    MountainPalette.Apply(visual,o.district);float foundation=Mathf.Max(0,o.base_z-o.bottom_z);
                    if(foundation>.02f){var slab=Primitive(PrimitiveType.Cube,o.id+"_foundation",Vector3.zero,Vector3.one,wall,visual.transform);UnityEngine.Object.DestroyImmediate(slab.GetComponent<Collider>());slab.transform.localPosition=new Vector3(0,-foundation/2,0);slab.transform.localScale=new Vector3(o.depth,foundation,o.width);}
                    var collider=visual.AddComponent<BoxCollider>();collider.center=new Vector3(0,(o.height-foundation)/2,0);collider.size=new Vector3(o.depth,o.height+foundation,o.width);
                }
                else
                {
                    Primitive(PrimitiveType.Cube,o.id,RosToUnity(o.x,o.y,(o.bottom_z+o.base_z+o.height)*.5f),
                        new Vector3(o.depth,o.base_z+o.height-o.bottom_z,o.width),wall,root.transform);
                    Primitive(PrimitiveType.Cube,o.id+"_roof",p+Vector3.up*(o.height+.15f),
                        new Vector3(o.depth+.4f,.3f,o.width+.4f),roof,root.transform);
                }
            }
            CityRoads.Generate(layout,root.transform);
            if(layout.pads!=null)foreach(var pad in layout.pads){
                var mat=Material(pad.name+"Marking",pad.name=="Takeoff"?new Color(.95f,.85f,.15f):new Color(.15f,.75f,.95f));
                foreach(float side in new[]{-1f,1f}){
                    var a=Primitive(PrimitiveType.Cube,pad.name+"BorderX",RosToUnity(pad.x+side*(pad.width*.5f-.15f),pad.y,pad.z+.01f),new Vector3(pad.depth,.02f,.3f),mat,root.transform,10);
                    var b=Primitive(PrimitiveType.Cube,pad.name+"BorderY",RosToUnity(pad.x,pad.y+side*(pad.depth*.5f-.15f),pad.z+.01f),new Vector3(.3f,.02f,pad.width),mat,root.transform,10);
                    UnityEngine.Object.DestroyImmediate(a.GetComponent<Collider>());UnityEngine.Object.DestroyImmediate(b.GetComponent<Collider>());
                }
            }
            return root;
        }
    }
}


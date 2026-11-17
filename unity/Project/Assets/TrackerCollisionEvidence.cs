using System;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Std;
namespace Yopo.TrackerTest {
 public partial class TrackerField {
  [Serializable] class ColliderEvidence {public string name,type;public Vector3 world_center_ros,unity_aabb_size;public float closest_distance_m;}
  [Serializable] class CollisionEvidence {public int drone_id,event_number;public Vector3 p,previous_ros;public float ground,body_contact_offset_m;public string kind;public ColliderEvidence[] colliders;public int[] peers;}
  void PublishCollisionDetail(Drone d,float ground){
   var found=new List<ColliderEvidence>();foreach(var c in Physics.OverlapSphere(d.body.position,.30f,1<<WorldGeometry.ObstacleLayer))if(c.name!="Ground_3000x500")found.Add(new ColliderEvidence{name=c.name,type=c.GetType().Name,world_center_ros=WorldGeometry.UnityToRos(c.bounds.center),unity_aabb_size=c.bounds.size,closest_distance_m=Vector3.Distance(c.ClosestPoint(d.body.position),d.body.position)});
   Vector3 travel=d.body.position-d.previous;if(travel.magnitude>.001f&&travel.magnitude<10)foreach(var hit in Physics.SphereCastAll(d.previous,.30f,travel.normalized,travel.magnitude,1<<WorldGeometry.ObstacleLayer)){var c=hit.collider;if(c.name!="Ground_3000x500"&&!found.Exists(v=>v.name==c.name))found.Add(new ColliderEvidence{name=c.name,type=c.GetType().Name,world_center_ros=WorldGeometry.UnityToRos(c.bounds.center),unity_aabb_size=c.bounds.size,closest_distance_m=Vector3.Distance(c.ClosestPoint(d.body.position),d.body.position)});}
   var peers=new List<int>();foreach(var other in drones)if(other!=d&&Vector3.Distance(d.body.position,other.body.position)<.60f)peers.Add(other.id);
   var evidence=new CollisionEvidence{drone_id=d.id,event_number=d.collisionEvents,p=d.p,previous_ros=WorldGeometry.UnityToRos(d.previous),ground=ground,body_contact_offset_m=d.bodyOffset,kind=d.p.z-d.bodyOffset<ground-.025f?"terrain_penetration":"obstacle_or_peer",colliders=found.ToArray(),peers=peers.ToArray()};string json=JsonUtility.ToJson(evidence);ros.Publish("/scene/collision_detail",new StringMsg(json));Debug.Log("COLLISION_DETAIL "+json);
  }
  void CreateDepthEnvelopes(){
   var world=GameObject.Find("World_3000x500_Altitude_0_200");var root=new GameObject("DepthOnlyPhysicsEnvelopes");var material=WorldGeometry.Material("DepthPhysicsEnvelope",Color.white);material.SetFloat("_Cull",0);int count=0;
   foreach(var c in UnityEngine.Object.FindObjectsOfType<Collider>()){
    if(c.name=="Ground_3000x500"||c.gameObject.layer!=8)continue;GameObject proxy=null;
    if(c is MeshCollider mesh){proxy=new GameObject("DepthEnvelope_"+c.name);proxy.AddComponent<MeshFilter>().sharedMesh=mesh.sharedMesh;proxy.AddComponent<MeshRenderer>();proxy.transform.SetParent(c.transform,false);}
    else if(c is BoxCollider box){proxy=GameObject.CreatePrimitive(PrimitiveType.Cube);proxy.transform.SetParent(c.transform,false);proxy.transform.localPosition=box.center;proxy.transform.localScale=box.size;}
    else if(c is SphereCollider sphere){proxy=GameObject.CreatePrimitive(PrimitiveType.Sphere);proxy.transform.SetParent(c.transform,false);proxy.transform.localPosition=sphere.center;proxy.transform.localScale=Vector3.one*sphere.radius*2;}
    else if(c is CapsuleCollider){proxy=GameObject.CreatePrimitive(PrimitiveType.Cube);proxy.transform.SetParent(root.transform,false);proxy.transform.position=c.bounds.center;proxy.transform.localScale=c.bounds.size;}
    if(proxy==null)throw new Exception("No depth envelope for "+c.GetType().Name+" "+c.name);
    var added=proxy.GetComponent<Collider>();if(added!=null)Destroy(added);proxy.name="DepthEnvelope_"+c.name;proxy.layer=12;var renderer=proxy.GetComponent<MeshRenderer>();renderer.sharedMaterial=material;renderer.shadowCastingMode=UnityEngine.Rendering.ShadowCastingMode.Off;count++;
   }
   // Proxies inherit each physics transform and introduce no persistent scene
   // changes. RGB/main camera exclude layer12; all depth cameras include it.
   Debug.Log("DEPTH_PHYSICS_ENVELOPES count="+count+" depth_only_layer=12 collider_thresholds_unchanged=true");
  }
 }
}


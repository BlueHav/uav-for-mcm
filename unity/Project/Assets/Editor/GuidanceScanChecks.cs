using System;
using System.Reflection;
using UnityEditor.SceneManagement;
using UnityEngine;
using Yopo.TrackerTest;

public static class GuidanceScanChecks {
 static void Check(bool ok,string message){if(!ok)throw new Exception("GUIDANCE_SCAN_CHECK_FAILED "+message);}
 public static void Run(){
  var type=typeof(TrackerField).Assembly.GetType("Yopo.TrackerTest.GuidanceScan");
  Check(type!=null,"Missing 1km measured raycast channel");
  var trace=type.GetMethod("Trace",BindingFlags.Public|BindingFlags.Static);
  Check(trace!=null,"Trace interface missing");
  EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
  var cube=GameObject.CreatePrimitive(PrimitiveType.Cube);cube.layer=30;
  cube.transform.localScale=new Vector3(2,10,10);cube.transform.position=new Vector3(900,10,0);Physics.SyncTransforms();
  var origin=new Vector3(0,10,0);
  Func<Vector4> cast=()=> (Vector4)trace.Invoke(null,new object[]{origin,Vector3.right,1<<30});
  var p=cast();Check(p.w==1&&Mathf.Abs(p.x-899)<.01f,"detect obstacle at 900m");
  cube.transform.position=new Vector3(1010,10,0);Physics.SyncTransforms();p=cast();
  Check(p.w==0&&Mathf.Abs(p.x-1000)<.01f,"hard 1000m range limit");
  cube.transform.position=new Vector3(500,10,0);Physics.SyncTransforms();p=cast();
  Check(p.w==1&&Mathf.Abs(p.x-499)<.01f,"nearest occluding surface");
  bool rejected=false;try{trace.Invoke(null,new object[]{origin,Vector3.zero,1<<30});}catch(TargetInvocationException ex){rejected=ex.InnerException is ArgumentException;}
  Check(rejected,"zero direction rejected");
  UnityEngine.Object.DestroyImmediate(cube);
  Debug.Log("GUIDANCE_SCAN_CHECKS_OK detect900 limit1000 occlusion finite_inputs");
 }
}

using System;
using System.IO;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
namespace Yopo.TrackerTest {
 public class MountainRuntimeAudit:MonoBehaviour {
  [Serializable] class View {public string name;public float fps,p95_ms;}
  [Serializable] class Result {public bool ready;public int loaded_scenes,buildings,trees,building_collider_checks;public float preload_seconds;public long allocated_memory;public View[] views;}
  IEnumerator Start(){
   yield return new WaitForSecondsRealtime(8);
   string dir=Environment.GetEnvironmentVariable("MC_REPORTS")??System.IO.Path.GetFullPath(System.IO.Path.Combine(UnityEngine.Application.dataPath, "../../Reports"));Directory.CreateDirectory(dir);
   var cam=Camera.main;var viewer=cam.GetComponent<SceneViewController>();viewer.enabled=false;
   var layout=WorldLayout.Load();var list=new List<View>();
   foreach(string name in new[]{"overview","city1","city2","city3","city4","extension","street"}){
    cam.orthographic=name=="overview";
    if(name=="overview"){cam.orthographicSize=620;cam.transform.position=WorldGeometry.RosToUnity(1000,0,2200);cam.transform.rotation=Quaternion.Euler(90,90,0);}
    else if(name.StartsWith("city")){int i=int.Parse(name.Substring(4))-1;var c=layout.districts[i];float x=(c.x_min+c.x_max)/2,y=(c.y_min+c.y_max)/2;cam.transform.position=WorldGeometry.RosToUnity(x-175,y-235,c.z+190);cam.transform.LookAt(WorldGeometry.RosToUnity(x,y,c.z+10));}
    else if(name=="extension"){cam.transform.position=WorldGeometry.RosToUnity(1850,-470,250);cam.transform.LookAt(WorldGeometry.RosToUnity(1600,150,20));}
    else{var r=layout.roads[layout.roads.Length/2];float x=r.x0,y=r.y0;cam.transform.position=WorldGeometry.RosToUnity(x,y,TerrainSampler.Height(layout,x,y)+8);cam.transform.LookAt(WorldGeometry.RosToUnity(x+60,y+25,TerrainSampler.Height(layout,x,y)+12));}
    yield return new WaitForSecondsRealtime(1.5f);
    var times=new List<float>();float start=Time.realtimeSinceStartup;while(Time.realtimeSinceStartup-start<3){times.Add(Time.unscaledDeltaTime*1000);yield return null;}
    times.Sort();list.Add(new View{name=name,fps=1000/times.Average(),p95_ms=times[(int)((times.Count-1)*.95f)]});
    var target=new RenderTexture(1600,1000,24);var tex=new Texture2D(1600,1000,TextureFormat.RGB24,false);var prev=RenderTexture.active;cam.targetTexture=target;cam.Render();RenderTexture.active=target;tex.ReadPixels(new Rect(0,0,1600,1000),0,0);tex.Apply();File.WriteAllBytes(Path.Combine(dir,name+".png"),tex.EncodeToPNG());cam.targetTexture=null;RenderTexture.active=prev;target.Release();Destroy(target);Destroy(tex);
   }
   int checkedBuildings=0;foreach(var o in layout.objects.Where(o=>o.kind=="building")){var g=GameObject.Find(o.id);var c=g==null?null:g.GetComponent<BoxCollider>();if(c==null||!c.enabled)throw new Exception("Runtime building collider missing "+o.id);if(!c.Raycast(new Ray(WorldGeometry.RosToUnity(o.x,o.y,o.base_z+o.height+10),Vector3.down),out var hit,o.height+20))throw new Exception("Runtime collider ray miss "+o.id);checkedBuildings++;}
   var result=new Result{ready=MountainBootstrap.Ready,loaded_scenes=SceneManager.sceneCount,buildings=layout.building_count,trees=layout.tree_count,building_collider_checks=checkedBuildings,preload_seconds=MountainBootstrap.LoadSeconds,allocated_memory=UnityEngine.Profiling.Profiler.GetTotalAllocatedMemoryLong(),views=list.ToArray()};
   File.WriteAllText(Path.Combine(dir,"runtime-validation.json"),JsonUtility.ToJson(result,true));Debug.Log("MOUNTAIN_RUNTIME_AUDIT_PASS");
   cam.orthographic=true;cam.orthographicSize=620;cam.transform.position=WorldGeometry.RosToUnity(1000,0,2200);cam.transform.rotation=Quaternion.Euler(90,90,0);viewer.enabled=true;Destroy(this);
  }
 }
}


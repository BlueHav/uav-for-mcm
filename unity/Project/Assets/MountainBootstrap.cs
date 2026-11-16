using System;
using System.IO;
using System.Collections;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;
namespace Yopo.TrackerTest {
 public class MountainBootstrap:MonoBehaviour {
  public string[] scenes;public Shader depthShader;
  public static bool Ready {get;private set;}
  public static float LoadSeconds {get;private set;}
  public static string Failure {get;private set;}
  float progress;int loaded;Camera loadingCamera;
  IEnumerator Start(){
   Ready=false;Failure=null;float start=Time.realtimeSinceStartup;
   loadingCamera=new GameObject("LoadingCamera").AddComponent<Camera>();loadingCamera.clearFlags=CameraClearFlags.SolidColor;loadingCamera.backgroundColor=new Color(.055f,.08f,.12f);
   if(scenes==null||scenes.Length==0){Failure="No scene chunks configured";Debug.LogError(Failure);yield break;}
   foreach(var scene in scenes){
    AsyncOperation op=null;try{op=SceneManager.LoadSceneAsync(scene,LoadSceneMode.Additive);}catch(Exception e){Failure=e.Message;}
    if(op==null){Failure=Failure??("Cannot load "+scene);Debug.LogError(Failure);yield break;}
    float deadline=Time.realtimeSinceStartup+180;
    while(!op.isDone){progress=(loaded+Mathf.Clamp01(op.progress/.9f))/scenes.Length;if(Time.realtimeSinceStartup>deadline){Failure="Loading timed out: "+scene;Debug.LogError(Failure);yield break;}yield return null;}
    if(!SceneManager.GetSceneByPath(scene).isLoaded){Failure="Scene not resident: "+scene;Debug.LogError(Failure);yield break;}
    loaded++;progress=(float)loaded/scenes.Length;yield return null;
   }
   Physics.SyncTransforms();LoadSeconds=Time.realtimeSinceStartup-start;Ready=true;
   Destroy(loadingCamera.gameObject);
   var ros=ROSConnection.GetOrCreateInstance();ros.ConnectOnStart=false;ros.RosPort=int.TryParse(Environment.GetEnvironmentVariable("YOPO_TCP_PORT"),out var port)?port:10012;
   var rig=new GameObject("MountainCityRuntime").AddComponent<TrackerField>();rig.enablePerson=false;rig.depthShader=depthShader;rig.gameObject.AddComponent<GoalOverlay>();
   if(Environment.GetEnvironmentVariable("MC_CAPTURE")=="1")rig.gameObject.AddComponent<MountainRuntimeAudit>();
   Debug.Log("MOUNTAIN_PRELOAD_READY chunks="+loaded+" seconds="+LoadSeconds);
  }
  void OnGUI(){if(Ready)return;GUI.Box(new Rect(Screen.width/2-230,Screen.height/2-55,460,110),"Mountain City | 2 km x 1 km");GUI.Label(new Rect(Screen.width/2-210,Screen.height/2-20,420,70),Failure==null?"Preparing terrain, streets and buildings... "+Mathf.RoundToInt(progress*100)+"%\n"+loaded+" / "+(scenes==null?0:scenes.Length)+" areas":Failure);}
 }
}


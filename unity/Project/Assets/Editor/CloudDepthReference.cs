using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using Yopo.TrackerTest;
public static class CloudDepthReference {
 [Serializable] class Frame {public string file,camera;public Vector3 position;public Quaternion rotation;}
 [Serializable] class Manifest {public string scope="static scene only; dynamic drone proxies tested separately";public Frame[] frames;}
 public static void Export(){
  if(SystemInfo.graphicsDeviceType==UnityEngine.Rendering.GraphicsDeviceType.Null)throw new Exception("Reference needs a real graphics device");
  EditorSceneManager.OpenScene("Assets/Scenes/PointMission.unity");
  string output="Reports/cloud-depth-reference";Directory.CreateDirectory(output);
  var world=WorldLayout.Load();var building=world.objects[0];
  var positions=new[]{new Vector3(-2765.95f,-5772.89f,212.6f),new Vector3(0,0,270),new Vector3(408.26f,2682.47f,16.25f),new Vector3(building.x-building.width/2-12,building.y,building.base_z+building.height/2),new Vector3(building.x,building.y,building.base_z+building.height+10)};
  var camera=new GameObject("CloudReferenceCamera").AddComponent<Camera>();camera.enabled=false;camera.nearClipPlane=.05f;camera.farClipPlane=80;camera.cullingMask=~((1<<10)|(1<<11));camera.clearFlags=CameraClearFlags.SolidColor;camera.backgroundColor=new Color(80,0,0,1);camera.allowMSAA=false;
  var target=new RenderTexture(480,270,24,RenderTextureFormat.RFloat,RenderTextureReadWrite.Linear);target.Create();camera.targetTexture=target;
  var read=new Texture2D(480,270,TextureFormat.RFloat,false,true);var shader=AssetDatabase.LoadAssetAtPath<Shader>("Assets/LinearEyeDepth.shader");var frames=new List<Frame>();
  var old=RenderTexture.active;
  try{
   for(int i=0;i<positions.Length;i++)foreach(var name in new[]{"front","up","down"}){
    float a=Mathf.Sqrt(.5f);var q=name=="front"?Quaternion.identity:new Quaternion(0,name=="up"?-a:a,0,a);
    camera.transform.position=TrackerField.ToUnity(positions[i]);camera.transform.rotation=Quaternion.LookRotation(TrackerField.ToUnity(q*Vector3.right),TrackerField.ToUnity(q*Vector3.forward));
    camera.fieldOfView=name=="front"?2*Mathf.Atan(270f/480)*Mathf.Rad2Deg:120;camera.aspect=480f/270;
    camera.RenderWithShader(shader,"");RenderTexture.active=target;read.ReadPixels(new Rect(0,0,480,270),0,0,false);
    var raw=read.GetRawTextureData<float>();var values=new float[480*270];for(int r=0;r<270;r++)for(int c=0;c<480;c++){float z=raw[(269-r)*480+c];values[r*480+c]=float.IsNaN(z)||float.IsInfinity(z)||z<=0?80:Mathf.Min(80,z);}
    var bytes=new byte[values.Length*4];Buffer.BlockCopy(values,0,bytes,0,bytes.Length);string file=i+"-"+name+".f32";File.WriteAllBytes(Path.Combine(output,file),bytes);frames.Add(new Frame{file=file,camera=name,position=positions[i],rotation=q});
   }
   File.WriteAllText(Path.Combine(output,"manifest.json"),JsonUtility.ToJson(new Manifest{frames=frames.ToArray()},true));Debug.Log("CLOUD_DEPTH_REFERENCE_OK frames="+frames.Count);
  }finally{RenderTexture.active=old;target.Release();UnityEngine.Object.DestroyImmediate(read);UnityEngine.Object.DestroyImmediate(camera.gameObject);}
 }
}

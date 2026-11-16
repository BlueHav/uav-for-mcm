using System;using System.IO;using System.Collections;using UnityEngine;
namespace Yopo.TrackerTest {public class GeoSensorAudit:MonoBehaviour{
 public Shader depthShader;
 IEnumerator Start(){if(DepthSourcePolicy.Parse(Environment.GetEnvironmentVariable("YOPO_DEPTH_SOURCE")))yield break;yield return new WaitForSeconds(10);var l=WorldLayout.Load();var o=l.objects[0];var g=GameObject.Find(o.id);var bounds=g.GetComponent<Renderer>().bounds;
 var cam=new GameObject("AuditCamera").AddComponent<Camera>();cam.enabled=false;cam.nearClipPlane=.1f;cam.farClipPlane=80;cam.cullingMask=1<<8;cam.clearFlags=CameraClearFlags.SolidColor;cam.backgroundColor=new Color(80,80,80,1);cam.transform.position=bounds.center+Vector3.up*(bounds.extents.y+20);cam.transform.rotation=Quaternion.LookRotation(Vector3.down,Vector3.forward);
 var rt=new RenderTexture(64,64,24,RenderTextureFormat.RFloat);rt.Create();cam.targetTexture=rt;cam.RenderWithShader(depthShader,"");var old=RenderTexture.active;RenderTexture.active=rt;var tex=new Texture2D(64,64,TextureFormat.RFloat,false,true);tex.ReadPixels(new Rect(0,0,64,64),0,0);tex.Apply();float measured=tex.GetRawTextureData<float>()[32*64+32];RenderTexture.active=old;
 RaycastHit hit;bool ray=Physics.Raycast(cam.transform.position,Vector3.down,out hit,80,1<<8);bool ok=ray&&measured>0&&Mathf.Abs(measured-hit.distance)<.6f;
 string report="{\"passed\":"+ok.ToString().ToLower()+",\"depth_m\":"+measured.ToString("R",System.Globalization.CultureInfo.InvariantCulture)+",\"ray_m\":"+hit.distance.ToString("R",System.Globalization.CultureInfo.InvariantCulture)+",\"hit\":\""+(ray?hit.collider.name:"")+"\"}";
 File.WriteAllText(Path.Combine(Application.dataPath,"../sensor-check.json"),report);Debug.Log("GEOMAP_SENSOR_AUDIT "+report);Destroy(cam.gameObject);rt.Release();Destroy(tex);
 }} }

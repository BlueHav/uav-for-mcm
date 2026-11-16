using System;
using System.IO;
using System.Collections;
using UnityEngine;
namespace Yopo.TrackerTest {
 public partial class TrackerField {
  [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
  static void StartSixLineAudit(){if(Environment.GetEnvironmentVariable("YOPO_UI_AUDIT")!="1")return;var field=FindObjectOfType<TrackerField>();field.StartCoroutine(field.SixLineAudit());}
  IEnumerator SixLineAudit(){
   yield return new WaitForSeconds(3);InitHud();
   float start=Time.realtimeSinceStartup;bool selectedSeven=false,captured=false,overview=false;
   while(Time.realtimeSinceStartup-start<28){
    float t=Time.realtimeSinceStartup-start;
    foreach(var d in drones){float z=TerrainSampler.Height(layout,d.p.x,d.p.y)+15+Mathf.Sin(t*.4f+d.id)*5;d.p.z=z;d.body.position=ToUnity(d.p);d.velocity=new Vector3(3+d.id*.1f,1,Mathf.Cos(t*.4f+d.id)*2);d.haveOdom=true;d.lastOdom=Time.realtimeSinceStartup;}
    fleetFormation=new FleetFormation{group_modes=new[]{"LINE","STAR"},stage="UI_TEST"};lastFormation=Time.realtimeSinceStartup;
    if(t>4&&!selectedSeven){Select(7);viewer.FollowSelected();selectedSeven=true;}
    if(t>13&&!captured){yield return CaptureUi("ui-sixline.png");captured=true;}
    if(t>18&&!overview){chase.orthographic=true;chase.orthographicSize=5800;chase.transform.position=WorldGeometry.RosToUnity(-3470,-1573,12000);chase.transform.rotation=Quaternion.Euler(90,90,0);viewer.enabled=false;overview=true;}
    if(t>21&&t<21.3f)yield return CaptureUi("ui-pins.png");
    yield return null;
   }
   bool pins=mapPoints!=null&&mapPoints.start!=null&&mapPoints.landing!=null&&mapPoints.end!=null&&mapPoints.end2!=null;
   bool linked=selected==7&&queryId=="7"&&viewer.target==drones[7].body;
   bool actualInput=inputDepthFrames!=null&&inputDepthFrames[7].HasFrame;
   int trees=0;foreach(var c in FindObjectsOfType<Collider>())if(c.name.StartsWith("TerminalForest_")&&c.name.EndsWith("_trunk")&&c.enabled&&c.gameObject.layer==8)trees++;
   bool pass=pins&&linked&&trees==48&&FiniteHud(EndDistance(drones[7]));
   string result="{\"passed\":"+pass.ToString().ToLowerInvariant()+",\"four_point_markers_loaded\":"+pins.ToString().ToLowerInvariant()+",\"selection_follow_linked\":"+linked.ToString().ToLowerInvariant()+",\"inference_input_received\":"+actualInput.ToString().ToLowerInvariant()+",\"tree_colliders\":"+trees+",\"synthetic_telemetry\":true,\"flight_test\":false}";
   File.WriteAllText(Path.Combine(Application.dataPath,"../ui-audit.json"),result);Debug.Log("SIX_LINE_AUDIT "+result);Application.Quit(pass?0:2);
  }
  IEnumerator CaptureUi(string name){yield return new WaitForEndOfFrame();var image=new Texture2D(Screen.width,Screen.height,TextureFormat.RGB24,false);image.ReadPixels(new Rect(0,0,Screen.width,Screen.height),0,0);image.Apply();File.WriteAllBytes(Path.Combine(Application.dataPath,"../"+name),image.EncodeToPNG());Destroy(image);}
 }
}

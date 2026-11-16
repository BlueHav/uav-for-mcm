using System;
using UnityEngine;
using RosMessageTypes.Std;
namespace Yopo.TrackerTest {
 public partial class TrackerField {
  string queryId="0",queryError="";
  GUIStyle hudText,hudTitle,hudMuted,hudButton,hudInput;
  Texture2D hudBackground;
  [Serializable] class FleetFormation {public string[] group_modes;public string stage;}
  FleetFormation fleetFormation;float lastFormation=-100;
  void InitHud(){
   if(hudText!=null)return;
   ros.ShowHud=false;if(ros.HUDPanel!=null)ros.HUDPanel.enabled=false;
   hudBackground=new Texture2D(1,1);hudBackground.SetPixel(0,0,new Color(.025f,.045f,.075f,.94f));hudBackground.Apply();
   hudText=new GUIStyle(GUI.skin.label){fontSize=14,wordWrap=false,clipping=TextClipping.Clip,alignment=TextAnchor.MiddleLeft};hudText.normal.textColor=new Color(.88f,.94f,1);
   hudTitle=new GUIStyle(hudText){fontSize=18,fontStyle=FontStyle.Bold};hudTitle.normal.textColor=new Color(.32f,.84f,1);
   hudMuted=new GUIStyle(hudText){fontSize=12};hudMuted.normal.textColor=new Color(.70f,.78f,.86f);
   hudButton=new GUIStyle(GUI.skin.button){fontSize=14};hudInput=new GUIStyle(GUI.skin.textField){fontSize=14,alignment=TextAnchor.MiddleLeft};
   var font=Resources.Load<Font>("HudFont");if(font!=null)foreach(var style in new[]{hudText,hudTitle,hudMuted,hudButton,hudInput})style.font=font;
   ros.Subscribe<StringMsg>("/swarm10/status",m=>{try{fleetFormation=JsonUtility.FromJson<FleetFormation>(m.data);lastFormation=Time.realtimeSinceStartup;}catch(Exception){}});

  }
  bool Fresh(Drone d){return d.haveOdom&&Time.realtimeSinceStartup-d.lastOdom<1.2f&&FiniteHud(d.velocity.x)&&FiniteHud(d.velocity.y)&&FiniteHud(d.velocity.z);}
  static bool FiniteHud(float v){return !float.IsNaN(v)&&!float.IsInfinity(v);}
  float Agl(Drone d){return d.p.z-TerrainSampler.Height(layout,d.p.x,d.p.y);}
  float EndDistance(Drone d){var p=mapPoints==null?null:(d.id<5?mapPoints.end:mapPoints.end2);return p==null?float.NaN:Vector2.Distance(new Vector2(d.p.x,d.p.y),new Vector2(p.x,p.y));}
  string Formation(int id){int group=id<5?0:1;if(fleetFormation!=null&&Time.realtimeSinceStartup-lastFormation<2&&fleetFormation.group_modes!=null&&group<fleetFormation.group_modes.Length)return fleetFormation.group_modes[group];return "--";}
  string Number(float v){return FiniteHud(v)?v.ToString("F1"):"--";}
  void FitLabel(Rect r,string value,GUIStyle style){int old=style.fontSize;float width=style.CalcSize(new GUIContent(value)).x;if(width>r.width)style.fontSize=Mathf.Max(10,Mathf.FloorToInt(old*r.width/width));GUI.Label(r,value,style);style.fontSize=old;}
  void DrawFleetHud(){
   InitHud();var matrix=GUI.matrix;float scale=Mathf.Min(1f,Screen.width/1440f,Screen.height/900f);GUI.matrix=Matrix4x4.TRS(Vector3.zero,Quaternion.identity,Vector3.one*scale);
   float sw=Screen.width/scale;const float left=18,top=18,width=1050,row=29;
   GUI.DrawTexture(new Rect(left,top,width,192),hudBackground);
   FitLabel(new Rect(30,24,1026,27),"SWARM FLIGHT MONITOR"+(Environment.GetEnvironmentVariable("YOPO_UI_AUDIT")=="1"?" / UI TEST: SYNTHETIC TELEMETRY":""),hudTitle);
   float speed=0,abs=0,agl=0,dist=0;int n=0,nd=0;
   foreach(var d in drones)if(Fresh(d)){speed+=d.velocity.magnitude;abs+=d.p.z;agl+=Agl(d);n++;float v=EndDistance(d);if(FiniteHud(v)){dist+=v;nd++;}}
   string modes="G1 "+Formation(0)+(drones.Count>5?" / G2 "+Formation(5):"");
   string fleet=n==0?"FLEET AVG  |  Waiting for telemetry":"FLEET AVG  |  "+Number(speed/n)+" m/s  |  ABS "+Number(abs/n)+" m  |  AGL "+Number(agl/n)+" m";
   FitLabel(new Rect(30,24+row,1026,27),fleet+"  |  "+modes+"  |  END XY "+Number(nd>0?dist/nd:float.NaN)+" m  |  UAV "+drones.Count+" (live "+n+")",hudText);
   float y=24+row*2;GUI.Label(new Rect(30,y,76,27),"Drone ID",hudText);GUI.SetNextControlName("DroneQuery");queryId=GUI.TextField(new Rect(106,y+1,58,25),queryId,3,hudInput);
   bool submit=GUI.Button(new Rect(174,y+1,80,25),"Select",hudButton);
   if(Event.current.type==EventType.KeyDown&&Event.current.keyCode==KeyCode.Return&&GUI.GetNameOfFocusedControl()=="DroneQuery"){submit=true;Event.current.Use();}
   if(submit){int id;if(int.TryParse(queryId,out id)&&id>=0&&id<drones.Count){Select(id);queryError="";GUI.FocusControl(null);}else queryError="ID must be 0-"+(drones.Count-1);}
   if(GUI.Button(new Rect(264,y+1,130,25),"Follow drone",hudButton)){viewer.FollowSelected();GUI.FocusControl(null);}
   var color=GUI.backgroundColor;GUI.backgroundColor=allMapMarkers?new Color(.3f,.75f,.9f):Color.gray;
   if(GUI.Button(new Rect(404,y+1,75,25),"label",hudButton))allMapMarkers=!allMapMarkers;GUI.backgroundColor=color;
   GUI.Label(new Rect(492,y,550,27),queryError.Length>0?queryError:"Selected UAV "+selected+"  |  Labels: "+(allMapMarkers?"all UAVs":"leaders"),hudMuted);
   var current=drones[selected];string details=Fresh(current)?Number(current.velocity.magnitude)+" m/s  |  ABS "+Number(current.p.z)+" m  |  AGL "+Number(Agl(current))+" m":"Waiting for fresh telemetry";
   FitLabel(new Rect(30,24+row*3,1026,27),"UAV "+selected+"  |  "+details+"  |  "+Formation(selected)+"  |  END XY "+(Fresh(current)?Number(EndDistance(current)):"--")+" m",hudText);
   GUI.Label(new Rect(30,24+row*4,1026,27),"TIME  "+DateTime.UtcNow.AddHours(8).ToString("yyyy-MM-dd HH:mm:ss")+" CST  |  ELAPSED "+Time.realtimeSinceStartup.ToString("F1")+" s  |  FPS "+viewer.Fps.ToString("F1")+"  |  VIEW "+viewer.ModeLabel,hudMuted);
   FitLabel(new Rect(30,24+row*5,1026,27),"C follow/free | V overview | F follow | 1-9/0 select | RMB look | MMB pan | WASD move | Q/E down/up | Shift faster | Wheel zoom | Arrows turn",hudMuted);
   DrawYopoInputDepth(sw);GUI.matrix=matrix;
  }
 }
}

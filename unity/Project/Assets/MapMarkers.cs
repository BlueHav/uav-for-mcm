using System;
using System.IO;
using UnityEngine;
namespace Yopo.TrackerTest {
 public partial class TrackerField {
  [Serializable] class MarkerPoint { public float x,y,ground_z; }
  [Serializable] class MarkerPoints { public MarkerPoint start,end,end2,landing; }
  [Serializable] class MarkerFile { public MarkerPoints points; }
  bool allMapMarkers=true;
  MarkerPoints mapPoints; float nextPointRead; string pointText;
  readonly System.Collections.Generic.List<Rect> pinIcons=new System.Collections.Generic.List<Rect>();
  Texture2D pinTexture; GUIStyle markerLabel; readonly System.Collections.Generic.List<Rect> markerRects=new System.Collections.Generic.List<Rect>();
  void DrawMapMarkers(){
   if(chase==null)return;

   if(Time.realtimeSinceStartup>=nextPointRead){nextPointRead=Time.realtimeSinceStartup+1;
    try {string path=Environment.GetEnvironmentVariable("YOPO_POINT_FILE");if(string.IsNullOrEmpty(path))path="/root/swarm-uav/config/point.json";
     double readStarted=SensorClock();string text=File.ReadAllText(path);SensorTiming("pin_file_read",readStarted);if(text!=pointText){var file=JsonUtility.FromJson<MarkerFile>(text);mapPoints=file.points;pointText=text;Debug.Log("MAP_MARKERS_LOADED start/end/end2/landing");}}
    catch(Exception e){if(pointText!="READ_ERROR")Debug.LogWarning("Map markers: "+e.Message);mapPoints=null;pointText="READ_ERROR";}
   }
   if(pinTexture==null){pinTexture=new Texture2D(48,64,TextureFormat.RGBA32,false);pinTexture.wrapMode=TextureWrapMode.Clamp;
    for(int y=0;y<64;y++)for(int x=0;x<48;x++){float dx=x-23.5f,dy=y-40;bool circle=dx*dx+dy*dy<22*22;bool tip=y<40&&Mathf.Abs(dx)<y*.57f;bool hole=dx*dx+dy*dy<8*8;pinTexture.SetPixel(x,y,hole?new Color(.05f,.08f,.12f,1):(circle||tip?Color.white:Color.clear));}pinTexture.Apply();
    markerLabel=new GUIStyle(GUI.skin.box){fontSize=14,fontStyle=FontStyle.Bold,alignment=TextAnchor.MiddleCenter};markerLabel.normal.textColor=Color.white;
   }
   markerRects.Clear();pinIcons.Clear();
   if(drones.Count==10){float u=Mathf.Min(1f,Screen.width/1440f,Screen.height/900f);markerRects.Add(new Rect(18*u,18*u,1050*u,192*u));markerRects.Add(new Rect(Screen.width-352*u,18*u,334*u,314*u));}
   var oldColor=GUI.color;var oldMatrix=GUI.matrix;GUI.matrix=Matrix4x4.identity;
   if(mapPoints!=null){PointPin(mapPoints.landing,"LANDING",new Color(.75f,.5f,1f));PointPin(mapPoints.start,"START",new Color(.1f,1,.45f));PointPin(mapPoints.end,"END 1",new Color(1,.3f,.25f));PointPin(mapPoints.end2,"END 2",new Color(1,.65f,.1f));}
   for(int i=0;i<drones.Count;i++){var d=drones[i];if(!allMapMarkers&&i!=2&&i!=7)continue;if(!d.haveOdom)continue;bool fresh=Time.realtimeSinceStartup-d.lastOdom<1.2f;DrawPin(d.body.position,"UAV "+i+(fresh?"":" STALE"),fresh?(i<5?Color.cyan:new Color(1,.65f,1)):Color.gray,24);}
   GUI.color=oldColor;GUI.matrix=oldMatrix;
  }
  void PointPin(MarkerPoint p,string label,Color color){if(p==null||float.IsNaN(p.x)||float.IsNaN(p.y)||float.IsNaN(p.ground_z)||float.IsInfinity(p.x)||float.IsInfinity(p.y)||float.IsInfinity(p.ground_z))return;DrawPin(WorldGeometry.RosToUnity(p.x,p.y,p.ground_z),label,color,34);}
  void DrawPin(Vector3 world,string label,Color color,float size){
   Vector3 p=chase.WorldToScreenPoint(world);if(p.z<=0||p.x<0||p.x>Screen.width||p.y<0||p.y>Screen.height)return;
   Vector2 tip=new Vector2(p.x,Screen.height-p.y);
   if(drones.Count==10){float u=Mathf.Min(1f,Screen.width/1440f,Screen.height/900f);if(tip.x<1068*u+size&&tip.y<210*u+size*4/3+25)tip.y=210*u+size*4/3+29;}
   Rect icon=new Rect(tip.x-size/2,tip.y-size*4/3,size,size*4/3);
   for(int tries=0;tries<30;tries++){bool overlap=false;foreach(var used in pinIcons)if(icon.Overlaps(used)){overlap=true;break;}if(!overlap)break;icon.x+=size+6;if(icon.xMax>Screen.width-4){icon.x=4;icon.y+=size*4/3+6;}}
   tip=new Vector2(icon.center.x,icon.yMax);pinIcons.Add(icon);markerRects.Add(icon);
   var anchor=new Vector2(p.x,Screen.height-p.y);if((anchor-tip).sqrMagnitude>4)Line(anchor,tip,color,.8f);
   GUI.color=color;GUI.DrawTexture(icon,pinTexture);
   float width=label.Length>8?112:80;Rect box=new Rect(Mathf.Clamp(tip.x-width/2,4,Screen.width-width-4),Mathf.Clamp(tip.y-size*4/3-25,4,Screen.height-28),width,24);
   for(int n=0;n<24;n++){bool overlap=false;foreach(Rect used in markerRects)if(box.Overlaps(used)){overlap=true;break;}if(!overlap)break;box.y+=26;if(box.yMax>Screen.height-4){box.y=4;box.x=Mathf.Clamp(box.x+width+4,4,Screen.width-width-4);}}
   markerRects.Add(box);Vector2 from=tip,to=box.center;float length=Vector2.Distance(from,to);var matrix=GUI.matrix;GUIUtility.RotateAroundPivot(Mathf.Atan2(to.y-from.y,to.x-from.x)*Mathf.Rad2Deg,from);GUI.DrawTexture(new Rect(from.x,from.y,length,1.5f),Texture2D.whiteTexture);GUI.matrix=matrix;
   GUI.color=new Color(.04f,.07f,.12f,.94f);GUI.DrawTexture(box,Texture2D.whiteTexture);GUI.color=color;GUI.Box(box,label,markerLabel);
  }
  void Line(Vector2 a,Vector2 b,Color color,float thickness){var matrix=GUI.matrix;var previous=GUI.color;GUI.color=color;GUIUtility.RotateAroundPivot(Mathf.Atan2(b.y-a.y,b.x-a.x)*Mathf.Rad2Deg,a);GUI.DrawTexture(new Rect(a.x,a.y,Vector2.Distance(a,b),thickness),Texture2D.whiteTexture);GUI.matrix=matrix;GUI.color=previous;}

 }
}

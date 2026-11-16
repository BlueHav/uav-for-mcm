using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

namespace Yopo.TrackerTest {
 public class SceneViewController : MonoBehaviour {
  public Transform target;
  public float Fps {get;private set;}
  public string ModeLabel {get{return mode.ToString();}}
  enum ViewMode { Follow, Free, Overview, Pad }
  ViewMode mode=ViewMode.Follow;bool followOnly;
  FlightBounds bounds;HeightField terrain;Camera cam;ROSConnection ros;Vector3 lastMouse;
  float worldTop;float moveSpeed=35,followDistance=24,elapsed;int frameCount,lookEvents;
  readonly List<float> frameTimes=new List<float>();
  [Serializable] class ViewStatus {
   public string mode,drone_model="yopo_tracker_uav_dae";
   public float fps,frame_p95_ms,camera_x,camera_y,camera_z,ground_top_m,ground_slope_deg;
   public bool person_present,overview_fits;
   public int terrain_count,look_events,view_width,view_height,drone_mesh_count;
   public bool drone_sensor_excluded,peer_visible=true;public string own_body_exclusion="render_disable";public float world_top_m;
   public float camera_pitch,camera_yaw,mouse_x,mouse_y;public bool right_mouse;
  }
  void Start(){
   followOnly=false;
   worldTop=UnityEngine.Object.FindObjectsOfType<Renderer>().Where(r=>r.enabled).Max(r=>r.bounds.max.y);
   bounds=WorldLayout.Load().bounds;terrain=WorldLayout.Load().terrain;cam=GetComponent<Camera>();cam.farClipPlane=20000;ros=ROSConnection.GetOrCreateInstance();
   ros.RegisterPublisher<StringMsg>("/scene/view_status",1);
   ros.Subscribe<StringMsg>("/scene/view_command",m=>{
    if(followOnly){SetMode(ViewMode.Follow);return;}
    switch(m.data.ToLowerInvariant()){
     case "follow":followDistance=24;SetMode(ViewMode.Follow);break;
     case "free":SetMode(ViewMode.Free);break;
     case "overview":SetMode(ViewMode.Overview);break;
     case "focus":SetMode(ViewMode.Follow);break;
     case "drone":followDistance=1.1f;SetMode(ViewMode.Follow);break;
     case "takeoff":PadView("Takeoff");break;
     case "landing":PadView("Landing");break;
     case "extension":MountainView(1850,200);break;case "mountain1":MountainView(525,40);break;case "mountain2":MountainView(990,-170);break;case "mountain3":MountainView(1495,-10);break;
     case "city1":CityView(0);break;case "city2":CityView(1);break;case "city3":CityView(2);break;case "city4":CityView(3);break;
    }
   });
   if(Environment.GetEnvironmentVariable("YOPO_DRONE_COUNT")!="10"&&GetComponent<ElevationLegend>()==null)gameObject.AddComponent<ElevationLegend>();
   lastMouse=Input.mousePosition;SetMode(ViewMode.Follow);
  }
  void MountainView(float x,float y){mode=ViewMode.Pad;cam.orthographic=false;transform.position=WorldGeometry.RosToUnity(x-180,y-225,220);transform.LookAt(WorldGeometry.RosToUnity(x,y,15));}
  void CityView(int index){
   var c=WorldLayout.Load().districts[index];mode=ViewMode.Pad;cam.orthographic=false;
   var center=WorldGeometry.RosToUnity((c.x_min+c.x_max)/2,(c.y_min+c.y_max)/2,c.z);transform.position=WorldGeometry.RosToUnity((c.x_min+c.x_max)/2-160,(c.y_min+c.y_max)/2-180,c.z+170);transform.LookAt(center);
  }
  void PadView(string name){
   var pad=WorldLayout.Load().pads.First(p=>p.name==name);mode=ViewMode.Pad;cam.orthographic=true;cam.orthographicSize=24;
   transform.position=WorldGeometry.RosToUnity(pad.x,pad.y,100);transform.rotation=Quaternion.Euler(90,90,0);
  }
  public void FollowSelected(){followDistance=24;SetMode(ViewMode.Follow);}
  void SetMode(ViewMode next){
   mode=next;cam.orthographic=mode==ViewMode.Overview;
   if(mode==ViewMode.Overview){
    cam.orthographicSize=Mathf.Max((bounds.y_max-bounds.y_min)/2,(bounds.x_max-bounds.x_min)/2/cam.aspect)*1.08f;
    transform.position=WorldGeometry.RosToUnity((bounds.x_min+bounds.x_max)/2,(bounds.y_min+bounds.y_max)/2,12000);transform.rotation=Quaternion.Euler(90,90,0);
   }else if(mode==ViewMode.Follow)FollowPose(true);
  }
  void FollowPose(bool immediate){
   if(target==null)return;
   Vector3 wanted=target.position+new Vector3(-.45f,.5f,-.75f).normalized*followDistance;
   transform.position=immediate?wanted:Vector3.Lerp(transform.position,wanted,1-Mathf.Exp(-6*Time.unscaledDeltaTime));
   transform.LookAt(target.position);
  }
  void Update(){
   Vector3 mouse=Input.mousePosition,delta=mouse-lastMouse;lastMouse=mouse;
   float dt=Mathf.Min(Time.unscaledDeltaTime,.1f);
   bool typing=GUIUtility.keyboardControl!=0;
   if(followOnly){float scroll=Input.mouseScrollDelta.y;followDistance=Mathf.Clamp(followDistance*Mathf.Exp(-scroll*.12f),4,300);elapsed+=Time.unscaledDeltaTime;frameCount++;frameTimes.Add(Time.unscaledDeltaTime*1000);if(elapsed>=1){Fps=frameCount/elapsed;Report();elapsed=0;frameCount=0;frameTimes.Clear();}return;}
   if(!typing){
    if(Input.GetKeyDown(KeyCode.C))SetMode(mode==ViewMode.Follow?ViewMode.Free:ViewMode.Follow);
    if(Input.GetKeyDown(KeyCode.V))SetMode(mode==ViewMode.Overview?ViewMode.Follow:ViewMode.Overview);
    if(Input.GetKeyDown(KeyCode.F))SetMode(ViewMode.Follow);
    float turn=(Input.GetKey(KeyCode.RightArrow)?1f:0f)-(Input.GetKey(KeyCode.LeftArrow)?1f:0f);
    if(turn!=0f){
     if(mode!=ViewMode.Free)SetMode(ViewMode.Free);
     transform.Rotate(Vector3.up,turn*75f*dt,Space.World);
    }
   }
   if(Input.GetMouseButtonDown(1)||Input.GetMouseButtonDown(2))delta=Vector3.zero;
   if(Input.GetMouseButton(1)){
    if(mode!=ViewMode.Free)SetMode(ViewMode.Free);
    if(delta.sqrMagnitude>.01f)lookEvents++;
    var e=transform.eulerAngles;float pitch=e.x>180?e.x-360:e.x;
    transform.rotation=Quaternion.Euler(Mathf.Clamp(pitch-delta.y*.18f,-89,89),e.y+delta.x*.18f,0);
   }
   if(Input.GetMouseButton(2)){
    if(mode==ViewMode.Follow)SetMode(ViewMode.Free);
    float scale=cam.orthographic?2*cam.orthographicSize/Mathf.Max(1,Screen.height):.06f;
    transform.position-=(transform.right*delta.x+transform.up*delta.y)*scale;
   }
   float wheel=Input.mouseScrollDelta.y;
   if(!typing&&Mathf.Abs(wheel)>.001f){
    if(mode==ViewMode.Overview)cam.orthographicSize=Mathf.Clamp(cam.orthographicSize*Mathf.Exp(-wheel*.12f),10,Mathf.Max(2500,Mathf.Max((bounds.y_max-bounds.y_min)/2,(bounds.x_max-bounds.x_min)/2/cam.aspect)*2));
    else if(mode==ViewMode.Follow)followDistance=Mathf.Clamp(followDistance*Mathf.Exp(-wheel*.12f),4,300);
    else transform.position+=transform.forward*wheel*moveSpeed*.25f;
   }
   if(mode==ViewMode.Free&&(!typing||Input.GetMouseButton(1))){
    float x=(Input.GetKey(KeyCode.D)?1:0)-(Input.GetKey(KeyCode.A)?1:0);
    float z=(Input.GetKey(KeyCode.W)?1:0)-(Input.GetKey(KeyCode.S)?1:0);
    float y=(Input.GetKey(KeyCode.E)?1:0)-(Input.GetKey(KeyCode.Q)?1:0);
    Vector3 direction=transform.right*x+transform.forward*z+Vector3.up*y;
    transform.position+=Vector3.ClampMagnitude(direction,1)*moveSpeed*dt*(Input.GetKey(KeyCode.LeftShift)||Input.GetKey(KeyCode.RightShift)?5:1);
   }
   elapsed+=Time.unscaledDeltaTime;frameCount++;frameTimes.Add(Time.unscaledDeltaTime*1000);
   if(elapsed>=1){
    Fps=frameCount/elapsed;Report();elapsed=0;frameCount=0;frameTimes.Clear();
   }
  }
  void LateUpdate(){if(mode==ViewMode.Follow)FollowPose(false);}
  void Report(){
   var ground=GameObject.Find("Ground_3000x500");
   var p=WorldGeometry.UnityToRos(transform.position);
   bool fits=true;
   foreach(float x in new[]{bounds.x_min,bounds.x_max})foreach(float y in new[]{bounds.y_min,bounds.y_max}){
    var v=cam.WorldToViewportPoint(WorldGeometry.RosToUnity(x,y,0));
    fits&=v.z>0&&v.x>=0&&v.x<=1&&v.y>=0&&v.y<=1;
   }
   var times=frameTimes.OrderBy(t=>t).ToArray();
   var data=new ViewStatus{mode=mode.ToString(),fps=Fps,frame_p95_ms=times.Length==0?0:times[(int)((times.Length-1)*.95f)],
    camera_x=p.x,camera_y=p.y,camera_z=p.z,person_present=GameObject.Find("MovingPerson")!=null,
    terrain_count=Terrain.activeTerrains.Length,overview_fits=fits,look_events=lookEvents,
    view_width=Screen.width,view_height=Screen.height,camera_pitch=transform.eulerAngles.x,camera_yaw=transform.eulerAngles.y,
    mouse_x=Input.mousePosition.x,mouse_y=Input.mousePosition.y,right_mouse=Input.GetMouseButton(1),
    world_top_m=worldTop,drone_mesh_count=target.GetComponentsInChildren<MeshFilter>().Length,drone_sensor_excluded=true,
    ground_top_m=ground==null?float.NaN:ground.GetComponent<Renderer>().bounds.max.y,
    ground_slope_deg=terrain!=null?terrain.max_slope_deg:(ground==null?float.NaN:Vector3.Angle(ground.transform.up,Vector3.up))};
   ros.Publish("/scene/view_status",new StringMsg(JsonUtility.ToJson(data)));
  }
 }
}


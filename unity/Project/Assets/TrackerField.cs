using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
namespace Yopo.TrackerTest {
 public partial class TrackerField:MonoBehaviour {
  public bool enablePerson=true;public Shader depthShader;public Material actorMaterial;
  WorldLayout layout;ROSConnection ros;Camera chase;SceneViewController viewer;int selected;string mapSha;
  [Serializable] public class MissionSpawns {public Vector3[] points;}
  readonly List<Drone> drones=new List<Drone>();
  DepthTransport depthTransport;
  bool pointcloudDepth;
  [Serializable] class DepthTransportStats {public long sent,errors,expired,replaced,dropped,wire_bytes,raw_bytes;public string last_error;}
  void PublishDepthTransportStats(){
   if(depthTransport==null)return;
   var t=depthTransport;
   var stats=new DepthTransportStats{sent=System.Threading.Interlocked.Read(ref t.Sent),errors=System.Threading.Interlocked.Read(ref t.Errors),expired=System.Threading.Interlocked.Read(ref t.Expired),
    replaced=System.Threading.Interlocked.Read(ref t.Pool.Replaced),dropped=System.Threading.Interlocked.Read(ref t.Pool.Dropped),wire_bytes=System.Threading.Interlocked.Read(ref t.WireBytes),raw_bytes=System.Threading.Interlocked.Read(ref t.RawBytes),last_error=t.LastError};
   ros.Publish("/scene/depth_sender",new StringMsg(JsonUtility.ToJson(stats)));
  }
  [Serializable] public class Mission {public string phase="idle",height_mode="ABS",reason="waiting";public float target_height,agl,abs,speed;}
  class Drone {
   public Quaternion rosRotation=Quaternion.identity;public int id,collisionEvents,boundsEvents;public int selfDepthLayer=-1;public Transform body;public Renderer[] renderers;public Vector3 p,velocity,previous;
   public float bodyOffset,lastOdom;public bool haveOdom,collision,outside;public Mission mission=new Mission();
   public Camera rgbCamera,depthCamera;public RenderTexture rgbTarget,depthTarget;public Texture2D rgbRead,depthRead;
   public float[] depthScratch,groundValues;public byte[] groundData;public float groundOriginX,groundOriginY;
   public string Topic(string suffix)=>"/drone_"+id+"/"+suffix;
  }
  public static Vector3 ToUnity(Vector3 p)=>WorldGeometry.RosToUnity(p.x,p.y,p.z);
  static HeaderMsg Header(string frame){long ms=DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();return new HeaderMsg{stamp=new TimeMsg((uint)(ms/1000),(uint)(ms%1000)*1000000),frame_id=frame};}
  void Start(){
   pointcloudDepth=DepthSourcePolicy.Parse(Environment.GetEnvironmentVariable("YOPO_DEPTH_SOURCE"));
   Application.targetFrameRate=60;Application.runInBackground=true;QualitySettings.vSyncCount=0;layout=WorldLayout.Load();
   using(var hash=System.Security.Cryptography.SHA256.Create())mapSha=BitConverter.ToString(hash.ComputeHash(System.Text.Encoding.UTF8.GetBytes(Resources.Load<TextAsset>("TrackerWorld").text))).Replace("-","").ToLowerInvariant();
   if(GameObject.Find("World_3000x500_Altitude_0_200")==null)WorldGeometry.Generate(layout);
   ros=ROSConnection.GetOrCreateInstance();ros.ConnectOnStart=false;
   int count=int.TryParse(Environment.GetEnvironmentVariable("YOPO_DRONE_COUNT"),out var requestedCount)?requestedCount:1;
   if(count!=1&&count!=2&&count!=5&&count!=10)throw new Exception("Supported drone counts: 1, 2, 5, 10");
   if(pointcloudDepth&&count!=10)throw new Exception("Pointcloud depth requires ten drones");
   if(count==10&&!pointcloudDepth){
    if(!SystemInfo.supportsAsyncGPUReadback)throw new Exception("Ten-drone lossless streaming requires AsyncGPUReadback");
    depthTransport=new DepthTransport();ros.ShowHud=false;if(ros.HUDPanel!=null)ros.HUDPanel.enabled=false;
   }
   var prefab=Resources.Load<GameObject>("TrackerDrone");if(prefab==null)throw new Exception("TrackerDrone missing");
   var spawnAsset=Resources.Load<TextAsset>("MissionSpawn");var spawns=spawnAsset==null?null:JsonUtility.FromJson<MissionSpawns>(spawnAsset.text);
   if(count==10&&(spawns==null||spawns.points==null||spawns.points.Length!=10))throw new Exception("Point mission requires exactly ten configured spawns");
   for(int i=0;i<count;i++){
    var d=new Drone{id=i};d.body=new GameObject("Drone_"+i).transform;
    var visual=Instantiate(prefab,d.body,false);d.renderers=visual.GetComponentsInChildren<Renderer>();
    float minY=float.PositiveInfinity;
    foreach(var mf in visual.GetComponentsInChildren<MeshFilter>())foreach(var v in mf.sharedMesh.vertices)minY=Mathf.Min(minY,d.body.InverseTransformPoint(mf.transform.TransformPoint(v)).y);
    d.bodyOffset=-minY;d.p=new Vector3(layout.spawn.x,layout.spawn.y+(count==1?0:count==2?(i==0?-8:8):count==10?-9+i*2:-12+i*6),0);d.p.z=TerrainSampler.Height(layout,d.p.x,d.p.y)+d.bodyOffset;
    if(count==10){d.p=spawns.points[i];d.p.z=TerrainSampler.Height(layout,d.p.x,d.p.y)+d.bodyOffset;}
    d.body.position=ToUnity(d.p);d.body.rotation=Quaternion.LookRotation(ToUnity(Vector3.right),ToUnity(Vector3.forward));d.previous=d.body.position;
    // Only markers retain layer 10; all drone meshes are visible to other drones.
    // In CUDA mode, the render camera owns its self mask. Avoid changing
    // renderer state around a render that may execute later on the GL thread.
    if(count==10&&cudaDepthOnly)d.selfDepthLayer=13+i;
    foreach(var t in visual.GetComponentsInChildren<Transform>())
     t.gameObject.layer=d.selfDepthLayer>=0?d.selfDepthLayer:11;
    if(!pointcloudDepth){SetupSensors(d);SetupVertical(d);}drones.Add(d);
    if(count==10&&(i==2||i==7)){
     var trace=new GameObject("CenterTrajectory_"+i);trace.transform.SetParent(d.body,false);trace.layer=10;
     var trail=trace.AddComponent<TrailRenderer>();trail.time=3600;trail.minVertexDistance=.15f;trail.startWidth=.18f;trail.endWidth=.18f;
     trail.material=new Material(Shader.Find("Sprites/Default"));trail.startColor=trail.endColor=i==2?new Color(1f,.45f,.1f):new Color(.1f,.65f,1f);
    }
    if(!pointcloudDepth){
     ros.RegisterPublisher<ImageMsg>(d.Topic("rgb"),1);ros.RegisterPublisher<ImageMsg>(d.Topic("depth"),1);ros.RegisterPublisher<CameraInfoMsg>(d.Topic("camera_info"),1);
     ros.RegisterPublisher<PoseStampedMsg>(d.Topic("depth_camera_pose"),1);ros.RegisterPublisher<PointStampedMsg>(RawSensorTopic(d,"ground"),1);ros.RegisterPublisher<PointCloud2Msg>(RawSensorTopic(d,"local_ground"),1);
    }
    ros.RegisterPublisher<StringMsg>("/scene/collision_detail",1);
    ros.RegisterPublisher<BoolMsg>(d.Topic("reset"),1);
    ros.Subscribe<OdometryMsg>(d.Topic("odom"),m=>OnOdom(d,m));
    ros.Subscribe<StringMsg>(d.Topic("mission/status"),m=>{try{d.mission=JsonUtility.FromJson<Mission>(m.data)??new Mission();}catch(Exception){}});
    Debug.Log("DRONE_BODY_CONTACT id="+i+" offset_m="+d.bodyOffset.ToString("R")+" spawn="+d.p);
   }
   if(count==10){
    ros.RegisterPublisher<StringMsg>("/swarm10/state",1);
    for(int g=0;g<2;g++){
     ros.RegisterPublisher<PointStampedMsg>("/sensors/raw/swarm10/group_"+g+"/ground",1);
     ros.RegisterPublisher<PointCloud2Msg>("/sensors/raw/swarm10/group_"+g+"/local_ground",1);
    }
   }
   if(count==10)InitYopoInputDepth();
   if(count==10&&!pointcloudDepth){ros.RegisterPublisher<PointCloud2Msg>("/unity/formation_ground",1);ros.RegisterPublisher<StringMsg>("/scene/depth_sender",1);}
   if(cloudMission)ros.Subscribe<StringMsg>("/swarm10/planning_mode",OnPlanningMode);
   chase=new GameObject("ChaseCamera").AddComponent<Camera>();chase.tag="MainCamera";chase.farClipPlane=20000;
   if(!pointcloudDepth)CreateDepthEnvelopes();chase.cullingMask=~(1<<12);chase.backgroundColor=new Color(.57f,.70f,.84f);chase.clearFlags=CameraClearFlags.SolidColor;
   viewer=chase.gameObject.AddComponent<SceneViewController>();viewer.target=drones[0].body;
   ros.RegisterPublisher<BoolMsg>("/scene/collision",1);ros.RegisterPublisher<BoolMsg>("/scene/bounds_violation",1);ros.RegisterPublisher<StringMsg>("/scene/status",1);
   ros.RegisterPublisher<StringMsg>("/scene/view_command",1);ros.RegisterPublisher<BoolMsg>("/respawn",1);ros.Subscribe<BoolMsg>("/scene/reset",m=>{if(m.data)ResetAll();});
   ros.Subscribe<Int32Msg>("/scene/select_drone",m=>{if(m.data>=0&&m.data<drones.Count)Select(m.data);});ros.Connect("127.0.0.1",int.TryParse(Environment.GetEnvironmentVariable("YOPO_TCP_PORT"),out var tcpPort)?tcpPort:10012);StartCoroutine(SensorLoop());StartCoroutine(GroundLoop());Debug.Log("TRACKER_FIELD_READY drones="+count+" terrain_preserved=true depth_source="+(pointcloudDepth?"pointcloud":"unity"));
  }
  void OnOdom(Drone d,OdometryMsg m){var p=m.pose.pose.position;if(double.IsNaN(p.x)||double.IsNaN(p.y)||double.IsNaN(p.z)||double.IsInfinity(p.x)||double.IsInfinity(p.y)||double.IsInfinity(p.z))return;
   d.p=new Vector3((float)p.x,(float)p.y,(float)p.z);d.body.position=ToUnity(d.p);
   var q=m.pose.pose.orientation;var rot=new Quaternion((float)q.x,(float)q.y,(float)q.z,(float)q.w);d.rosRotation=rot;d.body.rotation=Quaternion.LookRotation(ToUnity(rot*Vector3.right),ToUnity(rot*Vector3.forward));
   var v=m.twist.twist.linear;d.velocity=new Vector3((float)v.x,(float)v.y,(float)v.z);d.haveOdom=true;d.lastOdom=Time.realtimeSinceStartup;
  }
  void ResetAll(){foreach(var d in drones)ros.Publish(d.Topic("reset"),new BoolMsg(true));}
  float nextSceneMetricsWall;
  void LateUpdate(){
   double updateStarted=SensorClock();
   for(int i=0;i<drones.Count;i++)if(GUIUtility.keyboardControl==0&&Input.GetKeyDown(i==9?KeyCode.Alpha0:(KeyCode)((int)KeyCode.Alpha1+i)))Select(i);
   if(drones.Count!=10&&Input.GetKeyDown(KeyCode.R))ros.Publish(drones[selected].Topic("reset"),new BoolMsg(true));
   UpdateYopoInputDepth();
   foreach(var d in drones){
    if(!d.haveOdom)continue;
    // At rest the actual lowest mesh point touches terrain; terrain contact is
    // distinguished from penetration, while obstacles and peers remain collisions.
    float ground=TerrainSampler.Height(layout,d.p.x,d.p.y);
    bool hit=d.p.z-d.bodyOffset<ground-.025f;
    foreach(var c in Physics.OverlapSphere(d.body.position,.30f,1<<WorldGeometry.ObstacleLayer))if(c.name!="Ground_3000x500")hit=true;
    Vector3 travel=d.body.position-d.previous;
    if(travel.magnitude>.001f&&travel.magnitude<10)foreach(var h in Physics.SphereCastAll(d.previous,.30f,travel.normalized,travel.magnitude,1<<WorldGeometry.ObstacleLayer))if(h.collider.name!="Ground_3000x500")hit=true;
    foreach(var peer in drones)if(peer!=d&&Vector3.Distance(peer.body.position,d.body.position)<.60f)hit=true;
    if(hit&&!d.collision){d.collisionEvents++;PublishCollisionDetail(d,ground);}d.collision=hit;d.previous=d.body.position;
    bool outside=d.p.x<layout.bounds.x_min||d.p.x>layout.bounds.x_max||d.p.y<layout.bounds.y_min||d.p.y>layout.bounds.y_max||d.p.z<layout.bounds.z_min||d.p.z>layout.bounds.z_max;
    if(outside&&!d.outside)d.boundsEvents++;d.outside=outside;
   }
   SensorTiming("field_update",updateStarted);
   // Compact state follows every real main-thread collision/bounds evaluation
   // (50-60 Hz, frame-limited). The heavier evidence/HUD message stays at 5 Hz.
   PublishSwarmState();
   if(layout!=null&&ros!=null&&drones.Count>0&&Time.realtimeSinceStartup>=nextSceneMetricsWall){
    nextSceneMetricsWall=Time.realtimeSinceStartup+.2f;PublishMetrics();
   }
  }
  void Select(int id){selected=id;queryId=id.ToString();viewer.target=drones[id].body;}
  void OnGUI(){if(layout==null||drones.Count==0)return;
   double guiStarted=SensorClock();try{
   DrawMapMarkers();
   if(drones.Count==10){DrawFleetHud();return;}
   GUI.Box(new Rect(10,10,900,65+drones.Count*25),"GeoTIFF 10 km x 10 km | approximate buildings | 1-9 / 0: select drone | R: reset selected");
   for(int i=0;i<drones.Count;i++){var d=drones[i];float g=TerrainSampler.Height(layout,d.p.x,d.p.y);GUI.Label(new Rect(25,35+i*25,870,25),"Drone "+i+(i==selected?" *":"")+" | ABS "+d.p.z.ToString("F2")+" | ground "+g.ToString("F2")+" | AGL "+(d.p.z-g).ToString("F2")+" | speed "+d.velocity.magnitude.ToString("F2")+" | "+d.mission.phase+" "+d.mission.height_mode+" "+d.mission.target_height.ToString("F1")+" | collisions "+d.collisionEvents);}
   GUI.Label(new Rect(25,40+drones.Count*25,860,25),"C follow/free | V overview | F focus | free RMB look, WASD/QE move | FPS "+viewer.Fps.ToString("F1"));
   GUI.DrawTexture(new Rect(Screen.width-330,10,320,180),drones[selected].rgbTarget,ScaleMode.ScaleToFit,false);
   }finally{SensorTiming("on_gui",guiStarted);}
  }
  void OnDestroy(){ReleaseYopoInputDepth();if(hudBackground!=null)Destroy(hudBackground);depthClosing=true;StopAllCoroutines();if(!pointcloudDepth)UnityEngine.Rendering.AsyncGPUReadback.WaitAllRequests();if(depthTransport!=null)depthTransport.Dispose();foreach(var d in drones){ReleaseSensors(d);ReleaseVertical(d);}}
 }
}




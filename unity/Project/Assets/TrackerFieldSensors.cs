using System;
using System.Collections;
using System.IO;
using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
namespace Yopo.TrackerTest {
 public partial class TrackerField {
  const int Width=480,Height=270;int frames;float sensorCycleHz=20,groundCycleHz=10;
  readonly Drone[] activeSensorDrones=new Drone[10];
  int activeSensorCount,nextSensorCycleStart;
  static string RawSensorTopic(Drone d,string suffix)=>"/sensors/raw/drone_"+d.id+"/"+suffix;
  bool cloudMission=Environment.GetEnvironmentVariable("YOPO_CLOUD_MISSION")=="1";
  string planningRun;int planningEpoch=-1;
  [Serializable] class PlanningMode {public string run_id,planner_mode;public int planner_epoch;}
  void OnPlanningMode(StringMsg msg){
   if(!cloudMission)return;
   try{
    var mode=JsonUtility.FromJson<PlanningMode>(msg.data);
    if(mode==null||string.IsNullOrEmpty(mode.run_id))return;
    if(planningRun==null){if(mode.planner_mode!="CLOUD"||mode.planner_epoch!=0)return;planningRun=mode.run_id;planningEpoch=0;}
    if(mode.run_id!=planningRun)return;
    if(mode.planner_mode=="YOPO_WARMUP"&&mode.planner_epoch==1&&planningEpoch==0){
     foreach(var d in drones)if(!d.haveOdom||Mathf.Abs(d.p.z-TerrainSampler.Height(layout,d.p.x,d.p.y)-5)>.3f||d.velocity.magnitude>.25f)return;
     planningEpoch=1;
    }
    if(mode.planner_mode=="YOPO"&&mode.planner_epoch==2&&planningEpoch==1)planningEpoch=2;
   }catch(Exception ex){Debug.LogWarning("Invalid planning mode: "+ex.Message);}
  }
  [Serializable] class DroneMetrics {public int id,collision_events,bounds_events;public float x,y,z,ground,agl,speed,body_contact_offset_m,odom_age_s;public bool collision,bounds_violation;public Mission mission;}
  [Serializable] class SceneMetrics {public string scene="TrackerField",map_sha256,depth_source,depth_hz_basis;public int trees,buildings,frames,scene_frame,collision_events,bounds_events,drone_count,active_sensor_count;public float length,width,max_altitude,drone_x,drone_y,drone_z,odom_age_s,ground,agl,body_contact_offset_m,depth_hz=20,ground_hz=10,depth_far_m=80;public bool collision,bounds_violation,person_present=false;public DroneMetrics[] drones;}
  void SetupSensors(Drone d){
   d.rgbCamera=new GameObject("RGB_"+d.id).AddComponent<Camera>();d.rgbCamera.transform.SetParent(d.body,false);d.rgbCamera.enabled=false;
   d.rgbCamera.fieldOfView=2*Mathf.Atan((float)Height/Width)*Mathf.Rad2Deg;d.rgbCamera.nearClipPlane=.05f;d.rgbCamera.farClipPlane=3500;d.rgbCamera.cullingMask=~((1<<10)|(1<<12));
   d.rgbCamera.clearFlags=CameraClearFlags.SolidColor;d.rgbCamera.backgroundColor=new Color(.57f,.70f,.84f);d.rgbCamera.allowMSAA=false;
   d.rgbTarget=new RenderTexture(Width,Height,24,RenderTextureFormat.ARGB32,RenderTextureReadWrite.Linear);d.rgbTarget.Create();d.rgbCamera.targetTexture=d.rgbTarget;d.rgbRead=new Texture2D(Width,Height,TextureFormat.RGB24,false,true);
   d.depthCamera=new GameObject("Depth_"+d.id).AddComponent<Camera>();d.depthCamera.CopyFrom(d.rgbCamera);d.depthCamera.transform.SetParent(d.body,false);d.depthCamera.transform.localPosition=Vector3.zero;d.depthCamera.transform.localRotation=Quaternion.identity;d.depthCamera.enabled=false;d.depthCamera.cullingMask=~(1<<10);d.depthCamera.farClipPlane=80;d.depthCamera.backgroundColor=new Color(80,0,0,1);
   if(d.selfDepthLayer>=0)d.depthCamera.cullingMask&=~(1<<d.selfDepthLayer);
   d.depthTarget=new RenderTexture(Width,Height,24,RenderTextureFormat.RFloat,RenderTextureReadWrite.Linear);d.depthTarget.Create();d.depthCamera.targetTexture=d.depthTarget;d.depthRead=new Texture2D(Width,Height,TextureFormat.RFloat,false,true);
  }
  void ValidateSensorExtrinsics(Drone d){
   foreach(var camera in new[]{d.rgbCamera,d.depthCamera}){
    if(Vector3.Distance(camera.transform.position,ToUnity(d.p))>.00001f || Vector3.Dot(camera.transform.forward,ToUnity(d.rosRotation*Vector3.right))<.999999f || Vector3.Dot(camera.transform.up,ToUnity(d.rosRotation*Vector3.forward))<.999999f)throw new Exception("Camera extrinsics mismatch: "+camera.name+" position="+camera.transform.position+" body="+ToUnity(d.p));
   }
  }
  void PublishImages(Drone d){
   // All ten vehicles stream previews continuously; the flight nodes own
   // image-planning activation at the settled L-point handoff.
   if(drones.Count==10&&SystemInfo.supportsAsyncGPUReadback){
    ValidateSensorExtrinsics(d);if(d.depthScratch==null)d.depthScratch=new float[Width*Height];
    PublishAsyncDepth(d,d.depthCamera,d.depthTarget,"front",d.depthScratch);return;
   }
   bool renderRgb=drones.Count!=10;ValidateSensorExtrinsics(d);var header=Header("drone_"+d.id+"/camera_frame");var old=RenderTexture.active;
   var enabled=new bool[d.renderers.Length];for(int i=0;i<enabled.Length;i++){enabled[i]=d.renderers[i].enabled;d.renderers[i].enabled=false;}
   // These textures are CPU readback buffers only; do not upload them back to the GPU.
   try{if(renderRgb){d.rgbCamera.Render();RenderTexture.active=d.rgbTarget;d.rgbRead.ReadPixels(new Rect(0,0,Width,Height),0,0,false);}d.depthCamera.RenderWithShader(depthShader,"");RenderTexture.active=d.depthTarget;d.depthRead.ReadPixels(new Rect(0,0,Width,Height),0,0,false);}
   finally{RenderTexture.active=old;for(int i=0;i<enabled.Length;i++)d.renderers[i].enabled=enabled[i];}
   var source=d.rgbRead.GetRawTextureData<byte>();var z=d.depthRead.GetRawTextureData<float>();var rgb=renderRgb?new byte[Width*Height*3]:null;if(d.depthScratch==null)d.depthScratch=new float[Width*Height];var depths=d.depthScratch;
   for(int row=0;row<Height;row++)for(int col=0;col<Width;col++){int dst=row*Width+col,src=(Height-1-row)*Width+col;if(renderRgb)for(int k=0;k<3;k++)rgb[dst*3+k]=source[src*3+k];float m=z[src];depths[dst]=float.IsNaN(m)||float.IsInfinity(m)||m<=0?80:Mathf.Min(80,m);}
   var depth=new byte[depths.Length*4];Buffer.BlockCopy(depths,0,depth,0,depth.Length);
   var capture=new PoseStampedMsg{header=new HeaderMsg{stamp=header.stamp,frame_id="world"}};var actual=WorldGeometry.UnityToRos(d.depthCamera.transform.position);var actualQ=Quaternion.LookRotation(WorldGeometry.UnityToRos(d.depthCamera.transform.up),-WorldGeometry.UnityToRos(d.depthCamera.transform.right));capture.pose.position=new PointMsg(actual.x,actual.y,actual.z);capture.pose.orientation=new QuaternionMsg(actualQ.x,actualQ.y,actualQ.z,actualQ.w);ros.Publish(d.Topic("depth_camera_pose"),capture);if(renderRgb)ros.Publish(d.Topic("rgb"),new ImageMsg(header,Height,Width,"rgb8",0,Width*3,rgb));ros.Publish(d.Topic("depth"),new ImageMsg(header,Height,Width,"32FC1",0,Width*4,depth));
   var info=new CameraInfoMsg{header=header,width=Width,height=Height,distortion_model="plumb_bob",k=new double[]{240,0,240,0,240,135,0,0,1},r=new double[]{1,0,0,0,1,0,0,0,1},p=new double[]{240,0,240,0,0,240,135,0,0,0,1,0}};ros.Publish(d.Topic("camera_info"),info);
  }
  void PublishGround(Drone d,string group=null){
   if(!d.haveOdom)return; // Never publish ground from the prefab spawn before real odometry.
   var header=Header("world");var groundMsg=new PointStampedMsg{header=header,point=new PointMsg(d.p.x,d.p.y,TerrainSampler.Height(layout,d.p.x,d.p.y))};ros.Publish(RawSensorTopic(d,"ground"),groundMsg);
   Vector2 lead=Vector2.ClampMagnitude(new Vector2(d.velocity.x,d.velocity.y)*2,10);const int side=121;float originX=Mathf.Floor((d.p.x+lead.x)/.5f)*.5f-30,originY=Mathf.Floor((d.p.y+lead.y)/.5f)*.5f-30;
   // Terrain is immutable. Reuse only overlapping samples of the same local
   // sensor grid; newly exposed cells still call the physical terrain sampler.
   if(d.groundData==null||originX!=d.groundOriginX||originY!=d.groundOriginY){
    var values=new float[side*side*3];int dx=Mathf.RoundToInt((originX-d.groundOriginX)*2),dy=Mathf.RoundToInt((originY-d.groundOriginY)*2);
    for(int r=0;r<side;r++)for(int c=0;c<side;c++){
     int k=(r*side+c)*3,oldR=r+dy,oldC=c+dx;float x=originX+c*.5f,y=originY+r*.5f;values[k]=x;values[k+1]=y;
     values[k+2]=d.groundValues!=null&&oldR>=0&&oldR<side&&oldC>=0&&oldC<side?d.groundValues[(oldR*side+oldC)*3+2]:x<layout.bounds.x_min||x>layout.bounds.x_max||y<layout.bounds.y_min||y>layout.bounds.y_max?float.NaN:TerrainSampler.Height(layout,x,y);
    }
    var bytes=new byte[values.Length*4];Buffer.BlockCopy(values,0,bytes,0,bytes.Length);d.groundValues=values;d.groundData=bytes;d.groundOriginX=originX;d.groundOriginY=originY;
   }
   var data=d.groundData;
   var cloud=new PointCloud2Msg{header=header,height=side,width=side,fields=new[]{new PointFieldMsg("x",0,7,1),new PointFieldMsg("y",4,7,1),new PointFieldMsg("z",8,7,1)},is_bigendian=false,point_step=12,row_step=side*12,data=data,is_dense=false};ros.Publish(RawSensorTopic(d,"local_ground"),cloud);if(group!=null){ros.Publish(group+"/ground",groundMsg);ros.Publish(group+"/local_ground",cloud);}
  }
  IEnumerator GroundLoop(){
   // The single-point height keeps AGL fresh at 20 Hz for every body. The
   // 175 KB terrain grid is produced once per group at 3.3 Hz and reused by
   // the five members instead of being copied to every member topic.
   float next=Time.realtimeSinceStartup;
   while(true){
    next+=1f/20f;
    PublishGroundPoints();
    if(0==(groundTick++%6))PublishGroundClouds();
    float wait=next-Time.realtimeSinceStartup;
    if(wait>0)yield return new WaitForSecondsRealtime(wait);
    else{next=Time.realtimeSinceStartup;yield return null;}
   }
  }
  int groundTick;
  // Opt-in leader-only depth for legacy non-ten scenes.
  bool leaderDepthOnly=Environment.GetEnvironmentVariable("YOPO_LEADER_DEPTH")=="1";
  void PublishGroundPoints(){
   foreach(var d in drones)PublishGroundPoint(d);
  }
  void PublishGroundClouds(){
   if(drones.Count!=10){
    foreach(var d in drones)if(d.haveOdom)PublishGround(d);
    return;
   }
   for(int g=0;g<2;g++){
    int leaderId=2+5*g;
    if(leaderId>=drones.Count)continue;
    var leader=drones[leaderId];
    if(!leader.haveOdom)continue;
    // PublishGround also emits the per-leader pair used by RViz and the
    // group-shared raw pair every member subscribes to.
    PublishGround(leader,"/sensors/raw/swarm10/group_"+g);
   }
  }

  void PublishGroundPoint(Drone d){
   if(!d.haveOdom)return;
   ros.Publish(RawSensorTopic(d,"ground"),new PointStampedMsg{header=Header("world"),point=new PointMsg(d.p.x,d.p.y,TerrainSampler.Height(layout,d.p.x,d.p.y))});
  }

  IEnumerator SensorLoop(){
   if(pointcloudDepth){
    // Only the independent CPU worker owns ground and depth in this mode.
    // Scene heartbeat/collisions remain evidence from the actual Unity process.
    sensorCycleHz=0;groundCycleHz=0;
    ros.RegisterPublisher<StringMsg>("/scene/sensor_timing",1);
    while(true){frames++;PublishSensorTimings();yield return new WaitForSecondsRealtime(.1f);}
   }
   if(drones.Count==10){
    ros.RegisterPublisher<StringMsg>("/scene/sensor_timing",1);
    // YOPO consumes only front/up/down. The cloud cruise renders the group
    // leaders (display + source reuse); from the YOPO warmup epoch on every
    // body renders its own three views. A per-stream due time replaces the
    // old "render all thirty every 100 ms" cycle.
    QualitySettings.shadows=ShadowQuality.Disable;QualitySettings.antiAliasing=0;
    float depthRate=5f;
    if(!float.TryParse(Environment.GetEnvironmentVariable("YOPO_DEPTH_HZ"),out depthRate))depthRate=5f;
    depthRate=Mathf.Clamp(depthRate,1f,15f);
    sensorCycleHz=depthRate;groundCycleHz=20f/6f;
    float[] due=new float[drones.Count*3];float started=Time.realtimeSinceStartup;
    for(int i=0;i<due.Length;i++)due[i]=started;
    while(true){
     float now=Time.realtimeSinceStartup;float period=1f/depthRate;
     activeSensorCount=0;
     for(int id=0;id<drones.Count;id++)if(DepthEnabled(drones[id]))activeSensorCount++;
     // At 60 FPS, 30 streams at 15 Hz need at least eight captures
     // per frame. Give slower frames limited capacity to catch up.
     int captureBudget=Mathf.Clamp(Mathf.CeilToInt(depthRate*due.Length*Mathf.Max(Time.unscaledDeltaTime,1f/60f)),3,12);
     for(int slot=0;slot<captureBudget;slot++){
      int best=-1;float bestDue=float.PositiveInfinity;
      for(int id=0;id<drones.Count;id++){
       if(!DepthEnabled(drones[id]))continue;
       for(int camera=0;camera<3;camera++){int key=id*3+camera;if(due[key]<=now&&due[key]<bestDue&&!asyncDepthBusy.Contains(ScheduledDepthCamera(drones[id],camera))){bestDue=due[key];best=key;}}
      }
      if(best<0)break;
      PublishCamera(drones[best/3],best%3);
      due[best]=Mathf.Max(now-period,due[best]+period);
     }
     frames++;
     if(frames%12==0){PublishSensorTimings();PublishDepthTransportStats();PublishFormationGround();}
     yield return null;
    }
   }
   float next=Time.realtimeSinceStartup;while(true){float delay=next-Time.realtimeSinceStartup;if(delay>0)yield return new WaitForSecondsRealtime(delay);else yield return null;
    next+=.05f;if(next<Time.realtimeSinceStartup-.05f)next=Time.realtimeSinceStartup+.05f;
    foreach(var d in drones){if(drones.Count!=10||!leaderDepthOnly||d.id==2||d.id==7)PublishImages(d);if(drones.Count==10||frames%2==0){PublishVertical(d);}}frames++;
   }
  }
  // One camera per scheduled stream. The ten-body loop owns cadence; the
  // synchronous single/dual-body path keeps PublishImages/PublishVertical.
  void PublishCamera(Drone d,int camera){
   if(camera==0){
    // The old ten-body PublishImages allocated this scratch buffer; the
    // scheduled path must do it too or PackInto rejects the null shape.
    if(d.depthScratch==null)d.depthScratch=new float[Width*Height];
    PublishAsyncDepth(d,d.depthCamera,d.depthTarget,"front",d.depthScratch);return;
   }
   var s=vertical[d.id][camera-1];
   s.camera.transform.position=d.body.position;
   bool up=s.name=="up";
   s.camera.transform.rotation=Quaternion.LookRotation(up?Vector3.up:Vector3.down,up?Vector3.back:Vector3.forward);
   if(s.scratch==null)s.scratch=new float[Width*Height];
   PublishAsyncDepth(d,s.camera,s.target,s.name,s.scratch);
  }
  Camera ScheduledDepthCamera(Drone d,int camera){
   return camera==0?d.depthCamera:vertical[d.id][camera-1].camera;
  }
  bool DepthEnabled(Drone d){
   if(drones.Count!=10)return true;
   if(!cloudMission)return true;
   if(planningEpoch>=1)return true;
   return d.id==2||d.id==7;
  }
  [Serializable] class DroneStateSample {public int id;public float x,y,z,vx,vy,vz,agl,odom_age;public bool collision,bounds;}
  [Serializable] class SwarmStateSample {public long stamp_ms;public int scene_frame;public DroneStateSample[] drones;}
  float nextStateWall;
  void PublishSwarmState(){
   if(ros==null||layout==null||drones.Count!=10)return;
   var sample=new SwarmStateSample{stamp_ms=DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),scene_frame=Time.frameCount,drones=new DroneStateSample[drones.Count]};
   for(int i=0;i<drones.Count;i++){var d=drones[i];float g=TerrainSampler.Height(layout,d.p.x,d.p.y);
    sample.drones[i]=new DroneStateSample{id=d.id,x=d.p.x,y=d.p.y,z=d.p.z,vx=d.velocity.x,vy=d.velocity.y,vz=d.velocity.z,agl=d.p.z-g,odom_age=d.haveOdom?Mathf.Max(0f,Time.realtimeSinceStartup-d.lastOdom):999f,collision=d.collision,bounds=d.outside};}
   ros.Publish("/swarm10/state",new StringMsg(JsonUtility.ToJson(sample)));
  }
  float nextMetricsFileWall;
  void PublishMetrics(){
   var metrics=new SceneMetrics{active_sensor_count=drones.Count==10?activeSensorCount:drones.Count,depth_hz_basis=pointcloudDepth?"external_cpu_worker":drones.Count==10?"split_profile_scheduler":"capture_cycle",depth_hz=drones.Count==10?sensorCycleHz:20,ground_hz=drones.Count==10?groundCycleHz:10,trees=layout.tree_count,buildings=layout.building_count,frames=frames,scene_frame=Time.frameCount,length=layout.bounds.x_max-layout.bounds.x_min,width=layout.bounds.y_max-layout.bounds.y_min,max_altitude=layout.bounds.z_max,drone_count=drones.Count,drones=new DroneMetrics[drones.Count]};
   metrics.map_sha256=mapSha;metrics.depth_source=pointcloudDepth?"pointcloud":"unity";
   foreach(var d in drones){float g=TerrainSampler.Height(layout,d.p.x,d.p.y);metrics.drones[d.id]=new DroneMetrics{id=d.id,x=d.p.x,y=d.p.y,z=d.p.z,ground=g,agl=d.p.z-g,speed=d.velocity.magnitude,body_contact_offset_m=d.bodyOffset,odom_age_s=d.haveOdom?Time.realtimeSinceStartup-d.lastOdom:-1,collision=d.collision,bounds_violation=d.outside,collision_events=d.collisionEvents,bounds_events=d.boundsEvents,mission=d.mission};metrics.collision|=d.collision;metrics.bounds_violation|=d.outside;metrics.collision_events+=d.collisionEvents;metrics.bounds_events+=d.boundsEvents;}
   var first=metrics.drones[0];metrics.drone_x=first.x;metrics.drone_y=first.y;metrics.drone_z=first.z;metrics.ground=first.ground;metrics.agl=first.agl;metrics.body_contact_offset_m=first.body_contact_offset_m;metrics.odom_age_s=first.odom_age_s;
   string json=JsonUtility.ToJson(metrics);ros.Publish("/scene/status",new StringMsg(json));ros.Publish("/scene/collision",new BoolMsg(metrics.collision));ros.Publish("/scene/bounds_violation",new BoolMsg(metrics.bounds_violation));if(Time.realtimeSinceStartup>=nextMetricsFileWall){nextMetricsFileWall=Time.realtimeSinceStartup+2f;double writeStarted=SensorClock();File.WriteAllText(Path.Combine(Application.persistentDataPath,"runtime-metrics.json"),json);SensorTiming("metrics_file_write",writeStarted);}
  }
  void ReleaseSensors(Drone d){if(d.rgbTarget!=null)d.rgbTarget.Release();if(d.depthTarget!=null)d.depthTarget.Release();if(d.rgbRead!=null)Destroy(d.rgbRead);if(d.depthRead!=null)Destroy(d.depthRead);}
 }
}





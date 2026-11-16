using System;
using System.Globalization;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
namespace Yopo.TrackerTest {
 public class GoalOverlay:MonoBehaviour {
  [Serializable] class TaskCommand {public string op,height_mode;public float height,x,y,speed;}
  [Serializable] class SwarmCommand {public string op;}
  [Serializable] class SwarmStatus {public string stage,reason;}
  string swarmStage="WAITING",swarmReason="Waiting for coordinator";
  ROSConnection ros;int drone,count;string x="60",y="-50",height="8",speed="2",notice="Legacy /drone_i/goal always uses ABS Z";bool agl=true;

  void Start(){ros=ROSConnection.GetOrCreateInstance();count=int.TryParse(Environment.GetEnvironmentVariable("YOPO_DRONE_COUNT"),out var n)?n:1;if(count!=1&&count!=2&&count!=5&&count!=10)count=1;if(count==10){ros.RegisterPublisher<StringMsg>("/swarm10/command",1);ros.Subscribe<StringMsg>("/swarm10/status",m=>{try{var st=JsonUtility.FromJson<SwarmStatus>(m.data);swarmStage=st.stage;swarmReason=st.reason;}catch(Exception){}});}
   if(count==10)notice="Two groups of five: use run_ten.sh. 1-9/0 selects drone.";else if(count==5)notice="Five-agent task: use run_five.sh. Individual hold/land interrupts the group.";for(int i=0;i<count;i++){ros.RegisterPublisher<StringMsg>("/drone_"+i+"/task",1);ros.RegisterPublisher<PoseStampedMsg>("/drone_"+i+"/goal",1);}}
  void OnGUI(){if(count==10)return;
   if(count==10){
   float w=Mathf.Min(370,Screen.width-20),tenTop=Screen.height-100;
   GUI.Box(new Rect(10,tenTop,w,90),"Mission: "+swarmStage);
   GUI.enabled=swarmStage=="IDLE";
   if(GUI.Button(new Rect(25,tenTop+25,100,27),"Start"))ros.Publish("/swarm10/command",new StringMsg("{\"op\":\"start\"}"));
   GUI.enabled=swarmStage!="WAITING"&&swarmStage!="IDLE";
   if(GUI.Button(new Rect(135,tenTop+25,120,27),"End / Hover"))ros.Publish("/swarm10/command",new StringMsg("{\"op\":\"hold\"}"));
   GUI.enabled=true;
   GUI.Label(new Rect(25,tenTop+59,w-30,22),new GUIContent(swarmReason,swarmReason));return;
  }float top=Screen.height-175;GUI.Box(new Rect(10,top,920,165),"Drone mission | metres | legacy goal Z = ABS");
   if(GUI.Button(new Rect(25,top+25,120,25),"Drone "+drone))drone=(drone+1)%count;
   x=Field("X",x,155,top+25);y=Field("Y",y,300,top+25);height=Field("Height",height,445,top+25);speed=Field("m/s",speed,610,top+25);
   if(GUI.Button(new Rect(775,top+25,130,25),agl?"AGL":"ABS"))agl=!agl;
   string[] ops={"takeoff","goto","terrain_follow","hold","land","cancel"};for(int i=0;i<ops.Length;i++){string op=ops[i];if(GUI.Button(new Rect(25+i*145,top+65,135,28),op))Send(op);}
   if(GUI.Button(new Rect(25,top+105,190,25),"Legacy ABS goal"))Legacy();GUI.Label(new Rect(230,top+105,675,45),notice);
  }
  string Field(string label,string value,float left,float top){GUI.Label(new Rect(left,top,45,25),label);return GUI.TextField(new Rect(left+45,top,90,25),value);}
  bool Values(out float a,out float b,out float h,out float s){a=b=h=s=0;return float.TryParse(x,NumberStyles.Float,CultureInfo.InvariantCulture,out a)&&float.TryParse(y,NumberStyles.Float,CultureInfo.InvariantCulture,out b)&&float.TryParse(height,NumberStyles.Float,CultureInfo.InvariantCulture,out h)&&float.TryParse(speed,NumberStyles.Float,CultureInfo.InvariantCulture,out s)&&Finite(a)&&Finite(b)&&Finite(h)&&Finite(s);}
  bool Finite(float n)=>!float.IsNaN(n)&&!float.IsInfinity(n);
  void Send(string op){float a,b,h,s;if(!Values(out a,out b,out h,out s)){notice="Enter finite numbers";return;}var command=new TaskCommand{op=op,height_mode=agl?"AGL":"ABS",height=h,x=a,y=b,speed=s};ros.Publish("/drone_"+drone+"/task",new StringMsg(JsonUtility.ToJson(command)));notice="Sent "+op+" to drone "+drone;}
  void Legacy(){float a,b,h,s;if(!Values(out a,out b,out h,out s)){notice="Enter finite numbers";return;}var msg=new PoseStampedMsg();msg.header.frame_id="world";msg.pose.position=new PointMsg(a,b,h);msg.pose.orientation.w=1;ros.Publish("/drone_"+drone+"/goal",msg);notice="Sent legacy ABS goal Z="+h;}
 }
}


using System;
using UnityEngine;
using RosMessageTypes.Sensor;
namespace Yopo.TrackerTest {
 // Display geometry only. Local ground measurements used by planners stay separate.
 public struct FormationGroundGrid {
  public float minX,maxX,minY,maxY,step;
  public int width,height;
  public static FormationGroundGrid Create(Vector3[] positions,FlightBounds bounds){
   if(positions==null||positions.Length==0)throw new ArgumentException("Empty formation");
   float x0=float.PositiveInfinity,y0=x0,x1=float.NegativeInfinity,y1=x1;
   foreach(var p in positions){x0=Mathf.Min(x0,p.x);x1=Mathf.Max(x1,p.x);y0=Mathf.Min(y0,p.y);y1=Mathf.Max(y1,p.y);}
   var g=new FormationGroundGrid{minX=Mathf.Clamp(Mathf.Floor((x0-50)*2)*.5f,bounds.x_min,bounds.x_max),maxX=Mathf.Clamp(Mathf.Ceil((x1+50)*2)*.5f,bounds.x_min,bounds.x_max),minY=Mathf.Clamp(Mathf.Floor((y0-50)*2)*.5f,bounds.y_min,bounds.y_max),maxY=Mathf.Clamp(Mathf.Ceil((y1+50)*2)*.5f,bounds.y_min,bounds.y_max),step=.5f};
   g.width=Mathf.CeilToInt((g.maxX-g.minX)/g.step)+1;g.height=Mathf.CeilToInt((g.maxY-g.minY)/g.step)+1;
   return g;
  }
  public float X(int c)=>Mathf.Min(maxX,minX+c*step);
  public float Y(int r)=>Mathf.Min(maxY,minY+r*step);
  public byte[] Sample(WorldLayout layout){
   // One row of scratch space; retain only the current frame, never a trail.
   var row=new float[width*3];var data=new byte[width*height*12];
   for(int r=0;r<height;r++){
    for(int c=0;c<width;c++){int k=c*3;float x=X(c),y=Y(r);row[k]=x;row[k+1]=y;row[k+2]=TerrainSampler.Height(layout,x,y);}
    Buffer.BlockCopy(row,0,data,r*width*12,width*12);
   }
   return data;
  }
 }
 public partial class TrackerField {
  FormationGroundGrid displayGroundGrid;byte[] displayGroundData;float nextDisplayGround;
  void PublishFormationGround(){
   // Point missions already publish the bounded near/far terrain display from
   // terrain_display_node. The legacy full 0.5m AABB grows with group distance
   // and can block depth readback for hundreds of milliseconds on the main thread.
   if(Environment.GetEnvironmentVariable("YOPO_POINT_MISSION")=="1")return;
   foreach(var d in drones)if(!d.haveOdom)return;
   if(Time.realtimeSinceStartup<nextDisplayGround)return;
   nextDisplayGround=Time.realtimeSinceStartup+.5f;
   var positions=new Vector3[drones.Count];for(int i=0;i<positions.Length;i++)positions[i]=drones[i].p;
   var grid=FormationGroundGrid.Create(positions,layout.bounds);
   if(displayGroundData==null||!grid.Equals(displayGroundGrid)){displayGroundData=grid.Sample(layout);displayGroundGrid=grid;}
   ros.Publish("/unity/formation_ground",new PointCloud2Msg{header=Header("world"),height=(uint)grid.height,width=(uint)grid.width,fields=new[]{new PointFieldMsg("x",0,7,1),new PointFieldMsg("y",4,7,1),new PointFieldMsg("z",8,7,1)},is_bigendian=false,point_step=12,row_step=(uint)grid.width*12,data=displayGroundData,is_dense=true});
  }
 }
}

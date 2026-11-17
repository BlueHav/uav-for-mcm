using System;using System.Collections.Generic;using System.Linq;using UnityEngine;
namespace Yopo.TrackerTest {
 [Serializable] public class DistrictData {public string name,theme;public float x_min,x_max,y_min,y_max,z;}
 [Serializable] public class RoadData {public float x0,y0,x1,y1,width,walk;public string group;}
 public static class CityRoads {
  public static float Height(WorldLayout layout,float x,float y){
   var h=layout.terrain;float u=Mathf.Clamp((x-layout.bounds.x_min)/(layout.bounds.x_max-layout.bounds.x_min)*(h.width-1),0,h.width-1),v=Mathf.Clamp((layout.bounds.y_max-y)/(layout.bounds.y_max-layout.bounds.y_min)*(h.height-1),0,h.height-1);
   int c=Mathf.Min((int)u,h.width-2),r=Mathf.Min((int)v,h.height-2);u-=c;v-=r;int i=r*h.width+c;float a=h.heights[i],b=h.heights[i+1],d=h.heights[i+h.width],e=h.heights[i+h.width+1];
   return u+v<=1?a+u*(b-a)+v*(d-a):e+(1-u)*(d-e)+(1-v)*(b-e);
  }
  static float Cross(Vector2 a,Vector2 b){return a.x*b.y-a.y*b.x;}
  static List<Vector2> Clip(List<Vector2> input,Vector2 a,Vector2 b){
   var result=new List<Vector2>();if(input.Count==0)return result;Vector2 prev=input[input.Count-1];float dp=Cross(b-a,prev-a);
   foreach(var curr in input){float dc=Cross(b-a,curr-a);bool pi=dp>=-0.00001f,ci=dc>=-0.00001f;
    if(ci!=pi){float t=dp/(dp-dc);result.Add(prev+(curr-prev)*t);}if(ci)result.Add(curr);prev=curr;dp=dc;
   }return result;
  }
  public static void Generate(WorldLayout layout,Transform parent){
   if(layout.roads==null)return;
   var verts=new List<Vector3>[] {new List<Vector3>(),new List<Vector3>(),new List<Vector3>(),new List<Vector3>()};var tris=new List<int>[] {new List<int>(),new List<int>(),new List<int>(),new List<int>()};
   // Clip every road face to the actual terrain triangles. The visual and depth road surface
   // then follows the collider plane exactly, including diagonal roads and sharp grid changes.
   Action<int,Vector2[],float> polygon=(kind,quad,lift)=>{
    float dx=(layout.bounds.x_max-layout.bounds.x_min)/(layout.terrain.width-1),dy=(layout.bounds.y_max-layout.bounds.y_min)/(layout.terrain.height-1);
    int c0=Mathf.Clamp(Mathf.FloorToInt((quad.Min(p=>p.x)-layout.bounds.x_min)/dx),0,layout.terrain.width-2),c1=Mathf.Clamp(Mathf.FloorToInt((quad.Max(p=>p.x)-layout.bounds.x_min)/dx),0,layout.terrain.width-2);
    int r0=Mathf.Clamp(Mathf.FloorToInt((layout.bounds.y_max-quad.Max(p=>p.y))/dy),0,layout.terrain.height-2),r1=Mathf.Clamp(Mathf.FloorToInt((layout.bounds.y_max-quad.Min(p=>p.y))/dy),0,layout.terrain.height-2);
    for(int r=r0;r<=r1;r++)for(int c=c0;c<=c1;c++){
     Vector2 a=new Vector2(layout.bounds.x_min+c*dx,layout.bounds.y_max-r*dy),b=a+new Vector2(dx,0),d=a-new Vector2(0,dy),e=d+new Vector2(dx,0);
     foreach(var triangle in new[]{new[]{a,d,b},new[]{b,d,e}}){
      var p=new List<Vector2>(quad);for(int edge=0;edge<3;edge++)p=Clip(p,triangle[edge],triangle[(edge+1)%3]);if(p.Count<3)continue;
      int offset=verts[kind].Count;foreach(var q in p)verts[kind].Add(WorldGeometry.RosToUnity(q.x,q.y,Height(layout,q.x,q.y)+lift));
      for(int j=1;j<p.Count-1;j++){
       // Determine winding in final Unity coordinates; clipping can leave tiny slivers.
       float up=Vector3.Cross(verts[kind][offset+j]-verts[kind][offset],verts[kind][offset+j+1]-verts[kind][offset]).y;
       if(Mathf.Abs(up)<.000001f)continue;
       tris[kind].Add(offset);tris[kind].Add(offset+(up>0?j:j+1));tris[kind].Add(offset+(up>0?j+1:j));
      }
     }
    }
   };
   for(int ri=0;ri<layout.roads.Length;ri++){
    var road=layout.roads[ri];var a=new Vector2(road.x0,road.y0);var b=new Vector2(road.x1,road.y1);Vector2 tangent=(b-a).normalized,normal=new Vector2(-tangent.y,tangent.x),na=normal,nb=normal;
    if(!string.IsNullOrEmpty(road.group)){
     if(ri>0&&layout.roads[ri-1].group==road.group){var prev=layout.roads[ri-1];Vector2 dir=(a-new Vector2(prev.x0,prev.y0)).normalized;Vector2 m=(normal+new Vector2(-dir.y,dir.x)).normalized;na=m/Mathf.Max(.5f,Vector2.Dot(m,normal));}
     if(ri+1<layout.roads.Length&&layout.roads[ri+1].group==road.group){var next=layout.roads[ri+1];Vector2 dir=(new Vector2(next.x1,next.y1)-b).normalized;Vector2 m=(normal+new Vector2(-dir.y,dir.x)).normalized;nb=m/Mathf.Max(.5f,Vector2.Dot(m,normal));}
    }
    float half=road.width/2;
    Action<int,float,float,float> strip=(kind,left,right,lift)=>polygon(kind,new[]{a+na*left,b+nb*left,b+nb*right,a+na*right},lift);
    strip(0,-half,half,.015f);strip(1,-half-road.walk,-half,.025f);strip(1,half,half+road.walk,.025f);
    if(!string.IsNullOrEmpty(road.group)){
     if(ri%5<2)strip(2,-.08f,.08f,.04f);
    }else{
     float length=Vector2.Distance(a,b);for(float s=4;s<length-4;s+=10){Vector2 p=a+tangent*s,q=a+tangent*Mathf.Min(s+4,length-4);polygon(2,new[]{p-normal*.08f,q-normal*.08f,q+normal*.08f,p+normal*.08f},.04f);}
    }
   }
   if(layout.plazas!=null)foreach(var p in layout.plazas){var ring=new Vector2[48];for(int i=0;i<48;i++){float angle=i*Mathf.PI*2/48;ring[i]=new Vector2(p.x+Mathf.Cos(angle)*p.radius,p.y+Mathf.Sin(angle)*p.radius);}polygon(3,ring,.06f);}
   var colors=new[]{new Color(.12f,.13f,.14f),new Color(.58f,.57f,.53f),new Color(.95f,.84f,.25f),new Color(.72f,.66f,.52f)};
   for(int k=0;k<4;k++){
    var g=new GameObject("CityRoadSurface_"+k);g.layer=8;g.transform.SetParent(parent,false);var mesh=new Mesh{name="CityRoadSurface_"+k,indexFormat=UnityEngine.Rendering.IndexFormat.UInt32};mesh.SetVertices(verts[k]);mesh.SetTriangles(tris[k],0);mesh.RecalculateNormals();mesh.RecalculateBounds();
    g.AddComponent<MeshFilter>().sharedMesh=mesh;var mat=WorldGeometry.Material("CityRoadSurface_"+k,colors[k]);mat.SetFloat("_Cull",0);g.AddComponent<MeshRenderer>().sharedMaterial=mat;
   }
  }
 }
}

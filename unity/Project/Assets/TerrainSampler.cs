using UnityEngine;
namespace Yopo.TrackerTest {
 // Interpolate the exact two triangles emitted by HeightFieldMesh, rather than
 // bilinear interpolation or scene raycasts (which can hit vegetation/roofs).
 public static class TerrainSampler {
  public static float Height(WorldLayout l,float x,float y){
   var h=l.terrain;if(h==null)return 0;
   float u=Mathf.Clamp((x-l.bounds.x_min)/(l.bounds.x_max-l.bounds.x_min)*(h.width-1),0,h.width-1);
   float v=Mathf.Clamp((l.bounds.y_max-y)/(l.bounds.y_max-l.bounds.y_min)*(h.height-1),0,h.height-1);
   int c=Mathf.Min(Mathf.FloorToInt(u),h.width-2),r=Mathf.Min(Mathf.FloorToInt(v),h.height-2);u-=c;v-=r;
   int i=r*h.width+c;float a=h.heights[i],b=h.heights[i+1],d=h.heights[i+h.width],e=h.heights[i+h.width+1];
   return u+v<=1 ? a+(b-a)*u+(d-a)*v : e+(d-e)*(1-u)+(b-e)*(1-v);
  }
 }
}

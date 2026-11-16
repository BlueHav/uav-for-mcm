using System;
using UnityEngine;
namespace Yopo.TrackerTest {
 [Serializable] public class SpawnPoint {public float x,y,z;}
 [Serializable] public class HeightField {public int width,height;public float[] heights;public float max_slope_deg,min_height_m,max_height_m;}
 public static class HeightFieldMesh {
  public static GameObject Create(HeightField h,FlightBounds bounds,Transform parent,Material material){
   if(h.width<2||h.height<2||h.heights.Length!=h.width*h.height)throw new Exception("Invalid heightfield");
   var v=new Vector3[h.heights.Length];var uv=new Vector2[v.Length];var t=new int[(h.width-1)*(h.height-1)*6];int k=0;
   for(int r=0;r<h.height;r++)for(int c=0;c<h.width;c++){
    int i=r*h.width+c;float u=(float)c/(h.width-1),w=(float)r/(h.height-1);
    v[i]=WorldGeometry.RosToUnity(Mathf.Lerp(bounds.x_min,bounds.x_max,u),Mathf.Lerp(bounds.y_max,bounds.y_min,w),h.heights[i]);uv[i]=new Vector2(u,w);
    if(r<h.height-1&&c<h.width-1){int b=i+1,d=i+h.width,e=d+1;t[k++]=i;t[k++]=b;t[k++]=d;t[k++]=b;t[k++]=e;t[k++]=d;}
   }
   var mesh=new Mesh{name="Heightfield_0_140m"};mesh.indexFormat=UnityEngine.Rendering.IndexFormat.UInt32;
   mesh.vertices=v;mesh.uv=uv;mesh.triangles=t;mesh.RecalculateNormals();mesh.RecalculateBounds();
   var g=new GameObject("Ground_3000x500");g.transform.SetParent(parent,false);g.layer=WorldGeometry.ObstacleLayer;
   g.AddComponent<MeshFilter>().sharedMesh=mesh;g.AddComponent<MeshRenderer>().sharedMaterial=material;g.AddComponent<MeshCollider>().sharedMesh=mesh;
   return g;
  }
 }
}

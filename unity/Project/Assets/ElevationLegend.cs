using UnityEngine;
namespace Yopo.TrackerTest {
 public class ElevationLegend:MonoBehaviour {
  static readonly float[] stops={0,10,25,45,70,100,140};
  static readonly Color[] colors={new Color32(24,92,76,255),new Color32(99,179,93,255),new Color32(217,207,102,255),new Color32(201,139,76,255),new Color32(147,96,71,255),new Color32(137,126,155,255),new Color32(243,240,235,255)};
  Texture2D bar;
  void Start(){bar=new Texture2D(1,281,TextureFormat.RGB24,false);bar.wrapMode=TextureWrapMode.Clamp;for(int j=0;j<=280;j++){float z=j*.5f;int i=0;while(i<stops.Length-2&&z>stops[i+1])i++;bar.SetPixel(0,j,Color.Lerp(colors[i],colors[i+1],Mathf.InverseLerp(stops[i],stops[i+1],z)));}bar.Apply();}
  void OnGUI(){if(bar==null)return;float x=Screen.width-145,y=Screen.height-325;GUI.Box(new Rect(x,y,138,317),"Terrain height (m)");GUI.DrawTexture(new Rect(x+12,y+27,24,280),bar);foreach(float z in stops)GUI.Label(new Rect(x+43,y+27+280*(1-z/140)-10,75,22),z.ToString("0")+" m");}
  void OnDestroy(){if(bar!=null)Destroy(bar);}
 }
}

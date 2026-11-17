using System.Collections.Generic;
using UnityEngine;
namespace Yopo.TrackerTest {
 public static class MountainPalette {
  static readonly Dictionary<string,Material> cache=new Dictionary<string,Material>();
  public static void Clear(){cache.Clear();}
  public static void Apply(GameObject building,string district){
   Color tint;float mix;
   switch(district){case "Courtyard Gardens":tint=new Color(1.85f,1.28f,.82f);mix=.65f;break;case "Market Commons":tint=new Color(1.8f,.95f,.68f);mix=.55f;break;case "Woodland Village":tint=new Color(.95f,1.55f,1.12f);mix=.65f;break;default:tint=new Color(.9f,1.25f,1.65f);mix=.25f;break;}
   foreach(var r in building.GetComponentsInChildren<Renderer>()){
    var mats=r.sharedMaterials;for(int i=0;i<mats.Length;i++){
     var source=mats[i];if(source.shader.name!="Yopo/MountainSurface")continue;
     string key=source.name+"_"+district.Replace(" ","");
     if(!cache.TryGetValue(key,out var mat)){mat=new Material(source){name=key,enableInstancing=true};mat.SetColor("_ThemeColor",tint);mat.SetFloat("_ThemeBlend",mix);cache[key]=mat;}mats[i]=mat;
    }r.sharedMaterials=mats;
   }
  }
 }
}


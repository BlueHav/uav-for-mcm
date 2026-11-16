using System;
using UnityEngine;
using Yopo.TrackerTest;
public static class FormationGroundChecks {
 static void Check(bool ok,string reason){if(!ok)throw new Exception("GROUND_CHECK_FAILED "+reason);}
 public static void Run(){
  var layout=WorldLayout.Load();var p=new Vector3[10];
  // Translate and widen the two groups; altitude must not change the footprint.
  for(int n=0;n<50;n++){
   for(int i=0;i<10;i++)p[i]=new Vector3(200+n*20+(i%5)*10,-100+(i/5)*(40+n),150);
   var g=FormationGroundGrid.Create(p,layout.bounds);
   Check(g.minX<=p[0].x-50&&g.maxX>=p[4].x+50,"X margin");
   Check(g.minY<=p[0].y-50&&g.maxY>=p[9].y+50,"Y margin");
   Check(g.step==.5f,"fixed half-meter sampling");
   Check(g.X(g.width-1)==g.maxX&&g.Y(g.height-1)==g.maxY,"edge coverage");
   for(int i=0;i<10;i++)p[i].z=2;
   Check(g.Equals(FormationGroundGrid.Create(p,layout.bounds)),"altitude independence");
   if(n==0||n==49){var data=g.Sample(layout);Check(data.Length==g.width*g.height*12,"complete grid");for(int r=0;r<g.height;r++)for(int c=0;c<g.width;c++){int k=(r*g.width+c)*12;Check(BitConverter.ToSingle(data,k)==g.X(c)&&BitConverter.ToSingle(data,k+4)==g.Y(r),"no holes");Check(Mathf.Abs(BitConverter.ToSingle(data,k+8)-TerrainSampler.Height(layout,g.X(c),g.Y(r)))<.0001f,"physical terrain height");}}
  }
  p[0]=new Vector3(layout.bounds.x_min,layout.bounds.y_min,0);p[1]=new Vector3(layout.bounds.x_max,layout.bounds.y_max,0);
  var wide=FormationGroundGrid.Create(p,layout.bounds);
  Check(wide.step==.5f,"wide formation retains half-meter sampling");
  Check(wide.minX==layout.bounds.x_min&&wide.maxX==layout.bounds.x_max&&wide.minY==layout.bounds.y_min&&wide.maxY==layout.bounds.y_max,"world clipping");
  Debug.Log("FORMATION_GROUND_CHECKS_OK moving_cases=50 margin_m=50 spacing_m=0.5");
  HudBuild.Build();
 }
}

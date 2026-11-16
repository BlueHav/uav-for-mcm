#pragma once
// Terrain-only adapter for the unmodified Swarm-Formation search/optimizer APIs.
// This does not treat an absent cloud point as unobserved free space.
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
// Upstream headers relied on GridMap's namespace declarations.
using namespace std;
class GridMap {
public:
 using Ptr=std::shared_ptr<GridMap>;
 int nx=0,ny=0;double resolution=.5,ox=0,oy=0,ceiling=300,cruise_z=-1;
 std::vector<double> heights;
 std::vector<double> tops;
 void rebuild(){
  tops.assign(nx*ny,ceiling);
  for(int x=0;x<nx-1;x++)for(int y=0;y<ny-1;y++){
   double h=-1e9;bool valid=true;
   for(int a=0;a<2;a++)for(int b=0;b<2;b++){double v=heights[(x+a)*ny+y+b];valid&=std::isfinite(v)&&std::abs(v)<1e8;h=std::max(h,v);}
   if(valid)tops[x*ny+y]=h;
  }
 }
 double getResolution() const{return std::max(1.,resolution);}
 bool height(const Eigen::Vector3d&p,double& h) const {
  if(!p.allFinite())return false;
  int x=int(std::floor((p.x()-ox)/resolution)),y=int(std::floor((p.y()-oy)/resolution));
  if(!p.allFinite()||x<0||y<0||x+1>=nx||y+1>=ny)return false;
  h=-1e9;
  for(int a=0;a<2;a++)for(int b=0;b<2;b++){
   double v=heights[(x+a)*ny+y+b];if(!std::isfinite(v)||std::abs(v)>1e8)return false;h=std::max(h,v);
  }
  return true;
 }
 double getDistance(const Eigen::Vector3d&p) const {
  double h;if(!height(p,h))return -1.;
  double distance=std::min({3.,ceiling-p.z(),p.x()-ox,p.y()-oy,ox+(nx-1)*resolution-p.x(),oy+(ny-1)*resolution-p.y()});
  int cx=int(std::floor((p.x()-ox)/resolution)),cy=int(std::floor((p.y()-oy)/resolution));
  int reach=int(std::ceil(3./resolution));
  for(int x=std::max(0,cx-reach);x<=std::min(nx-2,cx+reach);x++)for(int y=std::max(0,cy-reach);y<=std::min(ny-2,cy+reach);y++){
   double dz=std::max(0.,p.z()-tops[x*ny+y]);if(dz>=distance)continue;
   double dx=std::max({ox+x*resolution-p.x(),p.x()-(ox+(x+1)*resolution),0.});
   double dy=std::max({oy+y*resolution-p.y(),p.y()-(oy+(y+1)*resolution),0.});
   distance=std::min(distance,std::sqrt(dx*dx+dy*dy+dz*dz));
  }
  return distance-.4;
 }
 int getInflateOccupancy(const Eigen::Vector3d&p)const{
  return getDistance(p)<=0.||(cruise_z>=0&&std::abs(p.z()-cruise_z)>.25);
 }
 void evaluateEDT(const Eigen::Vector3d&p,double&d)const{d=getDistance(p);}
 void evaluateFirstGrad(const Eigen::Vector3d&p,Eigen::Vector3d&g)const{
  for(int k=0;k<3;k++){Eigen::Vector3d a=p,b=p;a[k]+=resolution;b[k]-=resolution;g[k]=(getDistance(a)-getDistance(b))/(2*resolution);}
 }
};

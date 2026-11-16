#include <optimizer/poly_traj_optimizer.h>
#include <chrono>
#include <iomanip>
#include <iostream>
using Eigen::Vector3d;
static Vector3d readVector(){Vector3d p;std::cin>>p.x()>>p.y()>>p.z();return p;}
int main(){
 ros::Time::init();auto grid=std::make_shared<GridMap>();
 ego_planner::PolyTrajOptimizer optimizer;optimizer.configureCloud(2);optimizer.setEnvironment(grid);
 AStar astar;astar.initGridMap(grid,Eigen::Vector3i(96,96,32));
 ego_planner::SwarmTrajData peers;optimizer.setSwarmTrajs(&peers);
 long token;int drone,neighbor_count;double speed_limit;
 while(std::cin>>token>>grid->nx>>grid->ny>>grid->resolution>>grid->ox>>grid->oy>>grid->ceiling>>grid->cruise_z>>drone>>neighbor_count>>speed_limit){
  if(grid->nx<2||grid->ny<2||grid->nx>512||grid->ny>512||grid->resolution<=0||neighbor_count<0||neighbor_count>10)return 2;
  grid->heights.resize(grid->nx*grid->ny);for(auto& h:grid->heights)std::cin>>h;
  grid->rebuild();
  Eigen::Matrix3d head,tail;head.col(0)=readVector();head.col(1)=readVector();head.col(2)=readVector();tail.col(0)=readVector();tail.col(1)=readVector();tail.col(2).setZero();
  // L-BFGS scales its gradient stop by the norm of the waypoint variables.
  // Large world XY values otherwise terminate acceleration optimization early.
  // Translate the entire planning scene together; retain absolute altitude for
  // the terrain/ceiling contract and restore world XY only at the output edge.
  const Vector3d world_offset(head(0,0),head(1,0),0.);
  grid->ox-=world_offset.x();grid->oy-=world_offset.y();
  head.col(0)-=world_offset;tail.col(0)-=world_offset;
  peers.clear();double now=ros::Time::now().toSec();
  for(int n=0;n<neighbor_count;n++){
   int id;std::cin>>id;Vector3d p=readVector()-world_offset,v=readVector();poly_traj::CoefficientMat coeff;coeff.setZero();coeff.col(5)=p;coeff.col(4)=v;
   ego_planner::LocalTrajData peer;peer.drone_id=id;peer.start_time=now;peer.duration=10.;peer.traj=poly_traj::Trajectory({10.},{coeff});peers.push_back(peer);
  }
  if(!std::cin)return 3;
  if(!std::isfinite(speed_limit)||speed_limit<=0||speed_limit>20.)return 4;
  optimizer.configureCloud(drone,speed_limit);
  auto started=std::chrono::steady_clock::now();bool ok=false;poly_traj::Trajectory trajectory;
  if(head.allFinite()&&tail.allFinite()&&(head.col(0)-tail.col(0)).norm()>=1.&&!grid->getInflateOccupancy(head.col(0))&&!grid->getInflateOccupancy(tail.col(0))&&astar.AstarSearch(grid->getResolution(),head.col(0),tail.col(0),false)){
   auto raw=astar.getPath();std::vector<Vector3d> path{head.col(0)};
   if(raw.size()<2)raw={head.col(0),tail.col(0)};
   // Short, checked shortcutting; upstream's convenience wrapper dereferences
   // an empty failed path. Never call that wrapper or assume search succeeded.
   raw.front()=head.col(0);raw.back()=tail.col(0);size_t current=0;
   while(current+1<raw.size()){
    size_t chosen=current+1;
    for(size_t j=current+1;j<raw.size();j++){
     Vector3d delta=raw[j]-raw[current];bool clear=true;int steps=std::max(1,int(std::ceil(delta.norm()/.2)));
     for(int k=0;k<=steps;k++)if(grid->getDistance(raw[current]+delta*(double(k)/steps))<1.){clear=false;break;}
     if(clear)chosen=j;
    }
    path.push_back(raw[chosen]);current=chosen;
   }
   if(path.size()==2)path.insert(path.begin()+1,(path.front()+path.back())*.5);
   int count=path.size()-1;Eigen::MatrixXd inner(3,count-1);Eigen::VectorXd durations(count);
   double seed_speed=std::max(3.,std::min(speed_limit,std::max(head.col(1).norm(),tail.col(1).norm())));
   for(int i=0;i<count;i++){durations[i]=std::max(.5,(path[i+1]-path[i]).norm()/seed_speed);if(i<count-1)inner.col(i)=path[i+1];}
   poly_traj::MinJerkOpt seed;seed.reset(head,tail,count);seed.generate(inner,durations);
   auto constraints=seed.getInitConstrainPoints(8);optimizer.setControlPoints(constraints);
   ok=optimizer.OptimizeTrajectory_lbfgs(head,tail,inner,durations,constraints,false);trajectory=optimizer.getMinJerkOptPtr()->getTraj();
   double duration=trajectory.getTotalDuration();
   std::cout<<"CERT optimizer="<<int(ok)<<" duration="<<duration<<" vmax="<<trajectory.getMaxVelRate()<<" amax="<<trajectory.getMaxAccRate()<<std::endl;
   ok=ok&&std::isfinite(duration)&&duration>0&&duration<120&&trajectory.getMaxVelRate()<=speed_limit+.15&&trajectory.getMaxAccRate()<=(speed_limit>6.?2.15:1.35);
   if(ok)for(double t=0;t<=duration;t+=.02)if(grid->getInflateOccupancy(trajectory.getPos(t))){ok=false;break;}
  }
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  std::cout<<"RESULT "<<token<<" "<<int(ok)<<" "<<(ok?trajectory.getPieceNum():0)<<" "<<std::setprecision(17)<<ms;
  if(ok)for(int i=0;i<trajectory.getPieceNum();i++){
   std::cout<<" "<<trajectory[i].getDuration();auto c=trajectory[i].getCoeffMat();c.col(5)+=world_offset;for(int r=0;r<3;r++)for(int k=0;k<6;k++)std::cout<<" "<<c(r,k);
  }
  std::cout<<std::endl;
 }
}

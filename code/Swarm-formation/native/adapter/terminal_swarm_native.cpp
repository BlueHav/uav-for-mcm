// Five-body terminal adapter using upstream Swarm-Formation MINCO, L-BFGS,
// and the actual normalized-Laplacian SwarmGraph cost. No ROS publisher.
#include <optimizer/poly_traj_utils.hpp>
#include <optimizer/lbfgs.hpp>
#include <swarm_graph/swarm_graph.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <array>
using Eigen::Vector3d;
struct JointProblem {
  double duration;
  std::array<Eigen::Matrix3d,5> head,tail;
  std::array<Vector3d,5> seed;
  std::array<poly_traj::MinJerkOpt,5> minco;
  std::array<poly_traj::Trajectory,5> trajectories;
  SwarmGraph graph;
  int graph_evaluations=0;
  void generate(const double *x) {
    for(int i=0;i<5;i++) {
      Eigen::MatrixXd inner(3,1);inner.col(0)=Eigen::Map<const Vector3d>(x+3*i);
      minco[i].reset(head[i],tail[i],2);
      minco[i].generate(inner,Eigen::Vector2d(duration/2,duration/2));
      trajectories[i]=minco[i].getTraj();
    }
  }
  double cost(const double *x) {
    generate(x);double value=0;
    for(int i=0;i<5;i++) value+=.0001*minco[i].getTrajJerkCost()+20*(Eigen::Map<const Vector3d>(x+3*i)-seed[i]).squaredNorm();
    for(int k=0;k<=20;k++) {
      double t=duration*k/20.;std::vector<Vector3d> p;
      for(int i=0;i<5;i++) {
        p.push_back(trajectories[i].getPos(t));
        double v=trajectories[i].getVel(t).norm(),a=trajectories[i].getAcc(t).norm();
        value+=100000*std::pow(std::max(0.,v-1.1),2)+100000*std::pow(std::max(0.,a-1.1),2);
      }
      graph.updateGraph(p);double form=0;graph.calcFNorm2(form);graph_evaluations++;
      value+=300*form;
      for(int i=0;i<5;i++)for(int j=0;j<i;j++)value+=100000*std::pow(std::max(0.,2.8-(p[i]-p[j]).norm()),2);
    }
    return value;
  }
  static double evaluate(void *ptr,const double *x,double *gradient,int n) {
    auto &p=*static_cast<JointProblem*>(ptr);std::vector<double> y(x,x+n);const double h=1e-5;
    for(int i=0;i<n;i++){y[i]+=h;double a=p.cost(y.data());y[i]-=2*h;double b=p.cost(y.data());y[i]+=h;gradient[i]=(a-b)/(2*h);}
    return p.cost(x);
  }
};
static Vector3d read3(){Vector3d v;std::cin>>v.x()>>v.y()>>v.z();return v;}
int main(){
  ros::Time::init();std::cout<<std::setprecision(17);
  long serial;double duration;
  while(std::cin>>serial>>duration){
    if(!std::isfinite(duration)||duration<.5||duration>15)return 2;
    JointProblem p;p.duration=duration;
    for(int i=0;i<5;i++){p.head[i].col(0)=read3();p.head[i].col(1)=read3();p.head[i].col(2)=read3();p.tail[i].setZero();p.tail[i].col(0)=read3();}
    if(!std::cin)return 3;
    std::vector<Vector3d> star{{0,-3.3,0},{-3.3,0,0},{0,0,0},{3.3,0,0},{0,3.3,0}};p.graph.setDesiredForm(star);
    std::vector<double> x(15);std::vector<Vector3d> initial;
    for(int i=0;i<5;i++){
      if(!p.head[i].allFinite()||!p.tail[i].allFinite()||(p.tail[i].col(0)-p.head[i].col(0)).norm()>3.)return 4;
      p.seed[i]=(p.head[i].col(0)+p.tail[i].col(0))*.5;Eigen::Map<Vector3d>(x.data()+3*i)=p.seed[i];initial.push_back(p.head[i].col(0));
    }
    p.graph.updateGraph(initial);double graph_initial=0;p.graph.calcFNorm2(graph_initial);
    auto begin=std::chrono::steady_clock::now();double initial_cost=p.cost(x.data()),cost=initial_cost;
    lbfgs::lbfgs_parameter_t options=lbfgs::_default_param;options.max_iterations=25;options.g_epsilon=1e-5;
    int status=lbfgs::lbfgs_optimize(15,x.data(),&cost,JointProblem::evaluate,nullptr,nullptr,&p,&options);
    cost=p.cost(x.data());bool ok=std::isfinite(cost)&&cost<=initial_cost+1e-6;
    double vmax=0,amax=0;
    for(auto &tr:p.trajectories){vmax=std::max(vmax,tr.getMaxVelRate());amax=std::max(amax,tr.getMaxAccRate());}
    ok=ok&&vmax<=1.2&&amax<=1.2;
    double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    std::cout<<"TERMINAL {\"serial\":"<<serial<<",\"accepted\":"<<(ok?"true":"false")<<",\"stats\":{\"formation_enabled\":true,\"graph_evaluations\":"<<p.graph_evaluations<<",\"graph_cost_initial\":"<<graph_initial<<",\"cost_initial\":"<<initial_cost<<",\"cost_final\":"<<cost<<",\"vmax\":"<<vmax<<",\"amax\":"<<amax<<",\"milliseconds\":"<<ms<<",\"optimizer_status\":"<<status<<"},\"plans\":[";
    if(ok)for(int i=0;i<5;i++){
      if(i)std::cout<<",";std::cout<<"{\"durations\":["<<duration/2<<","<<duration/2<<"],\"coefficients\":[";
      for(int j=0;j<2;j++){
        if(j)std::cout<<",";std::cout<<"[";auto c=p.trajectories[i][j].getCoeffMat();
        for(int r=0;r<3;r++){if(r)std::cout<<",";std::cout<<"[";for(int k=0;k<6;k++){if(k)std::cout<<",";std::cout<<c(r,k);}std::cout<<"]";}
        std::cout<<"]";
      }
      std::cout<<"]}";
    }
    std::cout<<"]}"<<std::endl;
  }
}

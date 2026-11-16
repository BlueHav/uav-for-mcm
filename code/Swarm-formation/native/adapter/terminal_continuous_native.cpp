// Shared horizontal progression, independent vertical terrain tracking.
// Upstream MINCO adjoint gradients; independent Python admission is mandatory.
#include <optimizer/poly_traj_utils.hpp>
#include <optimizer/lbfgs.hpp>
#include <swarm_graph/swarm_graph.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <array>
using Eigen::Vector3d;
struct Obstacle { Vector3d p; double radius; };
struct Terrain { double resolution; Eigen::Vector2d origin; Eigen::Matrix<double,31,31> z; };
struct JointProblem {
  double duration,environment_cost=0,peer_cost=0;
  std::array<Eigen::Matrix3d,5> head,tail;
  std::array<Vector3d,5> seed;
  std::array<poly_traj::MinJerkOpt,5> minco;
  std::array<poly_traj::Trajectory,5> trajectories;
  std::vector<Obstacle> obstacles;
  std::vector<Vector3d> reference,peers;
  std::vector<Terrain> terrain;
  unsigned reference_mask=0;
  int peer_count=0,graph_evaluations=0;
  bool flexible=false;
  int variables() const { return flexible ? 17 : 7; }
  SwarmGraph graph;
  Eigen::Matrix<double,5,5> desired;
  void generate(const double *x) {
    for(int i=0;i<5;i++) {
      Eigen::MatrixXd inner(3,1);
      inner.col(0)=seed[i]+Vector3d(x[0],x[1],x[2+i]);
      if(flexible)inner.col(0).head<2>()+=Eigen::Vector2d(x[7+2*i],x[8+2*i]);
      minco[i].reset(head[i],tail[i],2);
      minco[i].generate(inner,Eigen::Vector2d(duration/2,duration/2));
      trajectories[i]=minco[i].getTraj();
    }
  }
  static double repel(const Vector3d &delta,double radius,double weight,Vector3d &grad) {
    const double norm=delta.norm(),gap=radius-norm;
    if(gap<=0)return 0.;
    if(norm>1e-9)grad-=2*weight*gap/norm*delta;
    return weight*gap*gap;
  }
  double cost(const double *x,double *gradient=nullptr) {
    generate(x);double value=0;environment_cost=peer_cost=0;
    std::array<Eigen::Vector2d,5> gdT;
    if(gradient)std::fill(gradient,gradient+variables(),0.);
    for(int i=0;i<5;i++) {
      double jerk=0;minco[i].initGradCost(gdT[i],jerk);
      minco[i].get_gdC()*=.0001;gdT[i]*=.0001;
      value+=.0001*jerk+20*(x[0]*x[0]+x[1]*x[1]+x[2+i]*x[2+i]);
      if(flexible)value+=20*(x[7+2*i]*x[7+2*i]+x[8+2*i]*x[8+2*i]);
    }
    for(int k=0;k<=20;k++) {
      const double t=duration*k/20.,local=t>=duration/2?t-duration/2:t;
      const int piece=t>=duration/2?1:0;
      Eigen::Matrix<double,6,1> b0,b1,b2;
      b0<<1,local,pow(local,2),pow(local,3),pow(local,4),pow(local,5);
      b1<<0,1,2*local,3*pow(local,2),4*pow(local,3),5*pow(local,4);
      b2<<0,0,2,6*local,12*pow(local,2),20*pow(local,3);
      std::vector<Vector3d> p;
      std::array<Vector3d,5> gp,gv,ga;
      for(int i=0;i<5;i++) {
        p.push_back(trajectories[i].getPos(t));gp[i].setZero();gv[i].setZero();ga[i].setZero();
        const Vector3d v=trajectories[i].getVel(t),a=trajectories[i].getAcc(t);
        double vn=v.norm(),an=a.norm();
        if(vn>6.){value+=100000*pow(vn-6.,2);gv[i]+=200000*(vn-6.)/vn*v;}
        if(an>2.15){value+=100000*pow(an-2.15,2);ga[i]+=200000*(an-2.15)/an*a;}
        for(const auto &o:obstacles)environment_cost+=repel(p[i]-o.p,o.radius,4000.,gp[i]);
        if(terrain.size()==5) {
          const auto &g=terrain[i];Eigen::Vector2d xy=(p[i].head<2>()-g.origin)/g.resolution;
          int ix=std::floor(xy.x()),iy=std::floor(xy.y());
          if(ix>=0&&ix<30&&iy>=0&&iy<30) {
            const double a0=g.z(ix,iy),a1=g.z(ix+1,iy),a2=g.z(ix,iy+1),a3=g.z(ix+1,iy+1);
            if(std::max({a0,a1,a2,a3})<1e8) {
              double u=xy.x()-ix,w=xy.y()-iy;
              double h=(1-u)*(1-w)*a0+u*(1-w)*a1+(1-u)*w*a2+u*w*a3;
              double gap=h+.8-p[i].z();
              if(gap>0) {
                environment_cost+=4000*gap*gap;
                gp[i]+=8000*gap*Vector3d(((1-w)*(a1-a0)+w*(a3-a2))/g.resolution,
                                            ((1-u)*(a2-a0)+u*(a3-a1))/g.resolution,-1.);
              }
            }
          }
        }
        for(int j=0;j<peer_count;j++)peer_cost+=repel(p[i]-peers[k*peer_count+j],3.,10000.,gp[i]);
      }
      graph.updateGraph(p);double form=0;graph.calcFNorm2(form);graph_evaluations++;value+=300*form;
      // The upstream getGrad normalizes its vector. Derive the true gradient
      // of the same normalized-Laplacian objective for L-BFGS consistency.
      Eigen::Matrix<double,5,5> A,B,E;Eigen::Matrix<double,5,1> D;
      for(int i=0;i<5;i++)for(int j=0;j<5;j++)A(i,j)=(p[i]-p[j]).squaredNorm();
      D=A.rowwise().sum();
      for(int i=0;i<5;i++)for(int j=0;j<5;j++)B(i,j)=A(i,j)/sqrt(D(i)*D(j));
      E=B-desired;
      for(int i=0;i<5;i++)for(int j=0;j<i;j++) {
        double edge=4*E(i,j)/sqrt(D(i)*D(j))-2*(E.row(i).array()*B.row(i).array()).sum()/D(i)
                   -2*(E.row(j).array()*B.row(j).array()).sum()/D(j);
        Vector3d g=600*edge*(p[i]-p[j]);gp[i]+=g;gp[j]-=g;
        g.setZero();value+=repel(p[i]-p[j],2.8,100000.,g);gp[i]+=g;gp[j]-=g;
      }
      if(reference.size()==21) {
        Vector3d delta=p[2]-reference[k];delta.z()=0;
        value+=20*delta.squaredNorm();gp[2]+=40*delta;
      } else if(reference.size()==105) {
        for(int i=0;i<5;i++)if(reference_mask&(1u<<i)) {
          Vector3d delta=p[i]-reference[k*5+i];delta.z()=0;
          value+=20*delta.squaredNorm();gp[i]+=40*delta;
        }
      }
      for(int i=0;i<5;i++)minco[i].get_gdC().block<6,3>(piece*6,0)+=b0*gp[i].transpose()+b1*gv[i].transpose()+b2*ga[i].transpose();
    }
    value+=environment_cost+peer_cost;
    if(gradient)for(int i=0;i<5;i++) {
      Eigen::Matrix<double,3,1> gdP;minco[i].getGrad2TP(gdT[i],gdP);
      gradient[0]+=gdP(0)+40*x[0];gradient[1]+=gdP(1)+40*x[1];gradient[2+i]=gdP(2)+40*x[2+i];
      if(flexible) {
        gradient[7+2*i]=gdP(0)+40*x[7+2*i];
        gradient[8+2*i]=gdP(1)+40*x[8+2*i];
      }
    }
    return value;
  }
  static double evaluate(void *ptr,const double *x,double *gradient,int) {
    return static_cast<JointProblem*>(ptr)->cost(x,gradient);
  }
};
static Vector3d read3(){Vector3d v;std::cin>>v.x()>>v.y()>>v.z();return v;}
int main(){
  ros::Time::init();std::cout<<std::setprecision(17);
  long serial;double duration;int check,no,nr,np,nt;
  while(std::cin>>serial>>duration>>check>>no>>nr>>np>>nt){
    if(!std::isfinite(duration)||duration<.5||duration>15||no<0||no>2560||!(nr==0||nr==21||nr==105)||np<0||np>5||!(nt==0||nt==5))return 2;
    JointProblem p;p.duration=duration;p.peer_count=np;p.flexible=(check&2)!=0;p.reference_mask=(check>>2)&31;
    for(int i=0;i<5;i++){p.head[i].col(0)=read3();p.head[i].col(1)=read3();p.head[i].col(2)=read3();p.tail[i].setZero();p.tail[i].col(0)=read3();p.tail[i].col(1)=read3();}
    const Vector3d offset=p.head[2].col(0);
    for(int i=0;i<no;i++){Obstacle o;o.p=read3()-offset;std::cin>>o.radius;p.obstacles.push_back(o);}
    for(int i=0;i<nr;i++)p.reference.push_back(read3()-offset);
    for(int i=0;i<21*np;i++)p.peers.push_back(read3()-offset);
    for(int i=0;i<nt;i++) {
      Terrain g;std::cin>>g.resolution>>g.origin.x()>>g.origin.y();g.origin-=offset.head<2>();
      for(int x=0;x<31;x++)for(int y=0;y<31;y++){std::cin>>g.z(x,y);g.z(x,y)-=offset.z();}p.terrain.push_back(g);
    }
    if(!std::cin)return 3;
    for(int i=0;i<5;i++){p.head[i].col(0)-=offset;p.tail[i].col(0)-=offset;}
    std::vector<Vector3d> star{{0,-3.3,0},{-3.3,0,0},{0,0,0},{3.3,0,0},{0,3.3,0}};p.graph.setDesiredForm(star);
    Eigen::Matrix<double,5,5> A;Eigen::Matrix<double,5,1> D;
    for(int i=0;i<5;i++)for(int j=0;j<5;j++)A(i,j)=(star[i]-star[j]).squaredNorm();D=A.rowwise().sum();
    for(int i=0;i<5;i++)for(int j=0;j<5;j++)p.desired(i,j)=A(i,j)/sqrt(D(i)*D(j));
    std::vector<double> x(p.variables(),0.);std::vector<Vector3d> initial;
    for(int i=0;i<5;i++){
      if(!p.head[i].allFinite()||!p.tail[i].allFinite()||(p.tail[i].col(0)-p.head[i].col(0)).norm()>30.)return 4;
      p.seed[i]=(p.head[i].col(0)+p.tail[i].col(0))*.5
                +.15625*duration*(p.head[i].col(1)-p.tail[i].col(1))
                +.015625*duration*duration*p.head[i].col(2);
      initial.push_back(p.head[i].col(0));
      if(p.flexible) {
        x[7+2*i]=.08*star[i].x();
        x[8+2*i]=.08*star[i].y();
      }
    }
    p.graph.updateGraph(initial);double graph_initial=0;p.graph.calcFNorm2(graph_initial);
    double gradient_error=0;
    if(check&1) {
      std::vector<double> trial(p.variables(),.13),g(p.variables());p.cost(trial.data(),g.data());
      for(int i=0;i<7;i++) {
        const double h=1e-5;trial[i]+=h;double a=p.cost(trial.data());trial[i]-=2*h;double b=p.cost(trial.data());trial[i]+=h;
        double numeric=(a-b)/(2*h);gradient_error=std::max(gradient_error,abs(numeric-g[i])/std::max(1.,std::max(abs(numeric),abs(g[i]))));
      }
    }
    auto begin=std::chrono::steady_clock::now();double initial_cost=p.cost(x.data()),cost=initial_cost;
    double environment_initial=p.environment_cost,peer_initial=p.peer_cost;
    lbfgs::lbfgs_parameter_t options=lbfgs::_default_param;options.max_iterations=50;options.g_epsilon=1e-5;
    int status=lbfgs::lbfgs_optimize(p.variables(),x.data(),&cost,JointProblem::evaluate,nullptr,nullptr,&p,&options);
    cost=p.cost(x.data());bool ok=std::isfinite(cost)&&cost<=initial_cost+1e-6;
    double vmax=0,amax=0;
    for(auto &tr:p.trajectories){vmax=std::max(vmax,tr.getMaxVelRate());amax=std::max(amax,tr.getMaxAccRate());}
    ok=ok&&vmax<=6.001&&amax<=2.151;
    double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    std::cout<<"TERMINAL {\"serial\":"<<serial<<",\"accepted\":"<<(ok?"true":"false")<<",\"stats\":{\"formation_enabled\":true,\"gradient_mode\":\"analytic_minco\",\"gradient_relative_error\":"<<gradient_error<<",\"environment_cost_initial\":"<<environment_initial<<",\"environment_cost_final\":"<<p.environment_cost<<",\"peer_cost_initial\":"<<peer_initial<<",\"graph_evaluations\":"<<p.graph_evaluations<<",\"graph_cost_initial\":"<<graph_initial<<",\"cost_initial\":"<<initial_cost<<",\"cost_final\":"<<cost<<",\"vmax\":"<<vmax<<",\"amax\":"<<amax<<",\"milliseconds\":"<<ms<<",\"optimizer_status\":"<<status<<"},\"plans\":[";
    if(ok)for(int i=0;i<5;i++){
      if(i)std::cout<<",";std::cout<<"{\"durations\":["<<duration/2<<","<<duration/2<<"],\"coefficients\":[";
      for(int j=0;j<2;j++){
        if(j)std::cout<<",";std::cout<<"[";auto c=p.trajectories[i][j].getCoeffMat();c.col(5)+=offset;
        for(int r=0;r<3;r++){if(r)std::cout<<",";std::cout<<"[";for(int k=0;k<6;k++){if(k)std::cout<<",";std::cout<<c(r,k);}std::cout<<"]";}
        std::cout<<"]";
      }
      std::cout<<"]}";
    }
    std::cout<<"]}"<<std::endl;
  }
}

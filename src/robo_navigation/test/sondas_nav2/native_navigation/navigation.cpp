// In-process Nav2 experiment. No action client/server, no cmd_vel publisher.
// Integrates computed twists into an IDEAL synthetic plant; not a physical robot test.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <execinfo.h>
#include <csignal>
#include <unistd.h>
#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

constexpr double PI = 3.141592653589793;
struct Box {double x0,x1,y0,y1;};
class Fixture : public nav2_costmap_2d::Costmap2DROS {
public:
  Fixture(): Costmap2DROS("fixture", "/", "offline", false) {
    global_frame_="map"; robot_base_frame_="base_link"; transform_tolerance_=0.3;
    footprint_padding_=0.01; use_radius_=false;
    layered_costmap_=std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layered_costmap_->resizeMap(240,240,0.025,-3,-3);
    std::vector<geometry_msgs::msg::Point> fp(4);
    fp[0].x=.365;fp[0].y=.315;fp[1].x=.365;fp[1].y=-.315;
    fp[2].x=-.205;fp[2].y=-.315;fp[3].x=-.205;fp[3].y=.315;
    setRobotFootprint(fp);
    tf_buffer_=std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setUsingDedicatedThread(true); // fixture prepopulates all transforms synchronously
  }
  void fill(const std::vector<Box> &boxes) {
    auto map=getCostmap();
    for(unsigned int i=0;i<map->getSizeInCellsX();++i) for(unsigned int j=0;j<map->getSizeInCellsY();++j) {
      double x,y; map->mapToWorld(i,j,x,y); unsigned char cost=0;
      for(const auto &b:boxes) {
        const double dx=std::max({b.x0-x,0.,x-b.x1}),dy=std::max({b.y0-y,0.,y-b.y1});
        const double d=std::hypot(dx,dy);
        // Analytic 2-D inflation for this fixture, same radius/factor as YAML.
        unsigned char c=d<.00001?254:d<=.215?253:d<=.55?static_cast<unsigned char>(252*std::exp(-5*(d-.215))):0;
        cost=std::max(cost,c);
      }
      map->setCost(i,j,cost);
    }
  }
  void pose(double x,double y,double angle) {
    geometry_msgs::msg::TransformStamped tr;
    tr.header.frame_id="map";tr.child_frame_id="base_link";tr.header.stamp=now();
    tr.transform.translation.x=x;tr.transform.translation.y=y;
    tr.transform.rotation.z=std::sin(angle/2);tr.transform.rotation.w=std::cos(angle/2);
    if(!tf_buffer_->setTransform(tr,"offline_fixture",true)) throw std::runtime_error("TF rejected");
  }
};
bool collides(double x,double y,double a,const std::vector<Box> &boxes) {
  double c=std::cos(a),s=std::sin(a),cx=x+.08*c,cy=y+.08*s,hx=.295,hy=.325;
  for(const auto &b:boxes) {
    double bx=(b.x0+b.x1)/2,by=(b.y0+b.y1)/2,ex=(b.x1-b.x0)/2,ey=(b.y1-b.y0)/2;
    double dx=bx-cx,dy=by-cy;
    if(std::abs(dx)>ex+hx*std::abs(c)+hy*std::abs(s))continue;
    if(std::abs(dy)>ey+hx*std::abs(s)+hy*std::abs(c))continue;
    if(std::abs(dx*c+dy*s)>hx+ex*std::abs(c)+ey*std::abs(s))continue;
    if(std::abs(-dx*s+dy*c)>hy+ex*std::abs(s)+ey*std::abs(c))continue;
    return true;
  }
  return false;
}
geometry_msgs::msg::PoseStamped pose(double x,double y,double a) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";
  p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=std::sin(a/2);p.pose.orientation.w=std::cos(a/2);return p;
}
nav_msgs::msg::Path line(double x,double y,double gx,double gy,double ga) {
  nav_msgs::msg::Path p;p.header.frame_id="map";
  int n=std::max(2,int(std::hypot(gx-x,gy-y)/.025));
  for(int i=0;i<=n;++i)p.poses.push_back(pose(x+(gx-x)*i/n,y+(gy-y)*i/n,ga));
  return p;
}
int main(int argc,char **argv) {
  std::signal(SIGSEGV, [](int){void *frames[40];int count=backtrace(frames,40);backtrace_symbols_fd(frames,count,2);_Exit(139);});
  rclcpp::init(argc,argv);
  auto node=std::make_shared<nav2_util::LifecycleNode>("controller_server","");
  auto fixture=std::make_shared<Fixture>();
  fixture->pose(0,0,0);
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");
  pluginlib::ClassLoader<nav2_core::GoalChecker> goals("nav2_core","nav2_core::GoalChecker");
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> planners("nav2_core","nav2_core::GlobalPlanner");
  std::string type=node->declare_parameter<std::string>("FollowPath.plugin","dwb_core::DWBLocalPlanner");
  std::string scenario=node->declare_parameter<std::string>("experiment.scenario","open");
  std::string output=node->declare_parameter<std::string>("experiment.output","/tmp/nav_metrics.json");
  bool replanning=node->declare_parameter<bool>("experiment.replanning",false);
  // The Shim owns an INNER loader. Keep DWB's library alive until the lifecycle
  // node releases its managed publisher entities, even after Shim cleanup.
  auto dwb_library_pin=loader.createSharedInstance("dwb_core::DWBLocalPlanner");
  auto controller=loader.createSharedInstance(type);
  auto goal=goals.createSharedInstance("nav2_controller::SimpleGoalChecker");
  goal->initialize(node,"goal_checker",fixture);
  controller->configure(node,"FollowPath",fixture->getTfBuffer(),fixture);
  // Do not activate lifecycle publishers: compute methods return twists in memory.
  auto planner=planners.createSharedInstance("nav2_navfn_planner::NavfnPlanner");
  planner->configure(node,"GridBased",fixture->getTfBuffer(),fixture);
  double x=0,y=0,a=0,gx=1.2,gy=0,ga=0;
  std::vector<Box> boxes;
  if(scenario=="lateral") {gx=0;gy=.10;}
  if(scenario=="curve") {gx=1.;gy=.6;}
  if(scenario=="obstacle"||scenario=="dynamic") {gx=1.8;boxes={{.85,1.1,-.18,.18}};}
  if(scenario=="near") {gx=1.2;boxes={{.39,.6,-.2,.2}};}
  if(scenario=="corner") {gx=1.2;gy=.8;boxes={{.55,.8,.25,.55}};}
  if(scenario=="final_yaw") {gx=.6;ga=PI/2;}
  fixture->fill(scenario=="dynamic"?std::vector<Box>{}:boxes);
  geometry_msgs::msg::Twist velocity;
  auto path=line(0,0,gx,gy,ga);
  if(scenario=="curve") {
    path.poses.clear();
    for(int i=0;i<=60;++i){double t=i/60.;path.poses.push_back(pose(gx*t,gy*(3*t*t-2*t*t*t),ga));}
  }
  controller->setPlan(path);
  bool complete=false;int failures=0,steps=0;double first=-1,stopped=0,rotation=0,max_lateral=0,first_deviation=-1;
  std::string error="";std::vector<double> timings;
  double intermediate_rotation=0,plan_max_y=0;int collisions=0,plan_collision_samples=0;
  for(int k=0;k<300;++k) {
    steps=k+1;fixture->pose(x,y,a);
    if(scenario=="dynamic"&&k==10)fixture->fill(boxes);
    if(replanning&&k%10==0) {
      try {
        path=planner->createPlan(pose(x,y,a),pose(gx,gy,ga),[]{return false;});
        for(const auto &p:path.poses) {
          plan_max_y=std::max(plan_max_y,std::abs(p.pose.position.y));
          const auto &q=p.pose.orientation;
          double angle=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
          plan_collision_samples+=collides(p.pose.position.x,p.pose.position.y,angle,(scenario=="dynamic"&&k<10)?std::vector<Box>{}:boxes);
        }
        controller->setPlan(path);
      }
      catch(const std::exception &e){error=e.what();break;}
    }
    auto current=pose(x,y,a);current.header.stamp=node->now();
    if(goal->isGoalReached(current.pose,pose(gx,gy,ga).pose,velocity)){complete=true;break;}
    geometry_msgs::msg::Twist cmd;
    auto begin=std::chrono::steady_clock::now();
    try {cmd=controller->computeVelocityCommands(current,velocity,goal.get()).twist;}
    catch(const std::exception &e) {++failures;error=e.what();stopped+=.1;velocity=geometry_msgs::msg::Twist(); if(failures>=5)break; continue;}
    timings.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
    const double v=std::hypot(cmd.linear.x,cmd.linear.y);
    if(v>.005&&first<0)first=k*.1;
    if(v<.005)stopped+=.1;
    rotation+=std::abs(cmd.angular.z)*.1;
    if(std::hypot(gx-x,gy-y)>.06)intermediate_rotation+=std::abs(cmd.angular.z)*.1;
    max_lateral=std::max(max_lateral,std::abs(cmd.linear.y));
    if(std::abs(cmd.linear.y)>.005&&first_deviation<0)first_deviation=x;
    x+=.1*(std::cos(a)*cmd.linear.x-std::sin(a)*cmd.linear.y);
    y+=.1*(std::sin(a)*cmd.linear.x+std::cos(a)*cmd.linear.y);
    a+=.1*cmd.angular.z;velocity=cmd;
    if(collides(x,y,a,(scenario=="dynamic"&&k<10)?std::vector<Box>{}:boxes)) {
      ++collisions;error="synthetic footprint-box collision";break;
    }
  }
  std::sort(timings.begin(),timings.end());
  double p95=timings.empty()?0:timings[std::min(timings.size()-1,size_t(timings.size()*.95))];
  std::ofstream out(output);
  out<<"{\"scenario\":\""<<scenario<<"\",\"plugin\":\""<<type<<"\",\"replanning\":"<<(replanning?"true":"false")
     <<",\"complete\":"<<(complete?"true":"false")<<",\"first_translation_s\":"<<first
     <<",\"no_translation_s\":"<<stopped<<",\"rotation_rad\":"<<rotation<<",\"duration_s\":"<<steps*.1
     <<",\"failures\":"<<failures<<",\"max_vy\":"<<max_lateral<<",\"first_lateral_x\":"<<first_deviation
     <<",\"intermediate_rotation_rad\":"<<intermediate_rotation<<",\"collisions\":"<<collisions
     <<",\"plan_collision_samples\":"<<plan_collision_samples<<",\"plan_max_abs_y\":"<<plan_max_y
     <<",\"compute_p95_ms\":"<<p95<<",\"x\":"<<x<<",\"y\":"<<y<<",\"yaw\":"<<a
     <<",\"error\":"<<std::quoted(error)<<"}\n";
  out.flush();out.close();
  controller->cleanup();planner->cleanup();
  goal.reset();controller.reset();planner.reset();fixture.reset();node.reset();
  rclcpp::shutdown();
}

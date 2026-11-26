// Real scan callback -> observation buffer -> global ObstacleLayer + InflationLayer
// -> real Navfn createPlan. All data stays in memory, with an empty static background.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
class ScanLayer: public nav2_costmap_2d::ObstacleLayer {
 public: void feed(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if(observation_buffers_.size()!=1) throw std::runtime_error("scan source count");
  laserScanValidInfCallback(scan,observation_buffers_[0]);
 }
};
class GlobalFixture: public nav2_costmap_2d::Costmap2DROS {
 public: GlobalFixture():Costmap2DROS("global_fixture","/","offline",false) {
  global_frame_="map";robot_base_frame_="base_link";footprint_padding_=.01;use_radius_=false;
  layered_costmap_=std::make_unique<nav2_costmap_2d::LayeredCostmap>("map",false,false);
  layered_costmap_->resizeMap(200,200,.05,-5,-5);
  tf_buffer_=std::make_shared<tf2_ros::Buffer>(get_clock());tf_buffer_->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="map";tf.child_frame_id="laser";
  tf.header.stamp=now();tf.transform.rotation.w=1;tf.transform.translation.x=.33;tf.transform.translation.y=.005;
  tf_buffer_->setTransform(tf,"offline_scan",true);
 }
 nav2_costmap_2d::LayeredCostmap* layers(){return layered_costmap_.get();}
};
geometry_msgs::msg::PoseStamped pose(double x) {geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.orientation.w=1;return p;}
double deviation(const nav_msgs::msg::Path &path) {double y=0;for(auto &p:path.poses)y=std::max(y,std::abs(p.pose.position.y));return y;}
int main(int argc,char **argv) {
 rclcpp::init(argc,argv);
 {
 auto fixture=std::make_shared<GlobalFixture>();
 auto global=std::make_shared<nav2_util::LifecycleNode>("global_costmap","/global_costmap");
 auto names=global->declare_parameter<std::vector<std::string>>("plugins",{});
 if(std::find(names.begin(),names.end(),"obstacle_layer")==names.end())return 2;
 auto group=global->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
 auto obstacles=std::make_shared<ScanLayer>();auto inflation=std::make_shared<nav2_costmap_2d::InflationLayer>();
 fixture->layers()->addPlugin(obstacles);fixture->layers()->addPlugin(inflation);
 obstacles->initialize(fixture->layers(),"obstacle_layer",fixture->getTfBuffer().get(),global,group);
 inflation->initialize(fixture->layers(),"inflation_layer",fixture->getTfBuffer().get(),global,group);
 std::vector<geometry_msgs::msg::Point> fp(4);fp[0].x=.365;fp[0].y=.315;fp[1].x=.365;fp[1].y=-.315;fp[2].x=-.205;fp[2].y=-.315;fp[3].x=-.205;fp[3].y=.315;fixture->setRobotFootprint(fp);
 auto planner_node=std::make_shared<nav2_util::LifecycleNode>("planner_server","");
 pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
 auto planner=loader.createSharedInstance("nav2_navfn_planner::NavfnPlanner");
 planner->configure(planner_node,"GridBased",fixture->getTfBuffer(),fixture);
 fixture->layers()->updateMap(0,0,0);
 auto before=planner->createPlan(pose(0),pose(2.5),[]{return false;});
 auto scan=std::make_shared<sensor_msgs::msg::LaserScan>();scan->header.frame_id="laser";scan->header.stamp=global->now();scan->angle_min=-.2;scan->angle_max=.2;scan->angle_increment=.01;scan->range_min=.08;scan->range_max=5.;scan->ranges.assign(41,1.0);
 obstacles->feed(scan);fixture->layers()->updateMap(0,0,0);
 auto after=planner->createPlan(pose(0),pose(2.5),[]{return false;});
 int unsafe=0;for(auto &p:after.poses){unsigned int x,y;if(!fixture->getCostmap()->worldToMap(p.pose.position.x,p.pose.position.y,x,y)||fixture->getCostmap()->getCost(x,y)>=253)unsafe++;}
 std::cout<<"configured_source="<<global->get_parameter("obstacle_layer.scan.topic").as_string()<<" before_max_y="<<deviation(before)<<" after_max_y="<<deviation(after)<<" unsafe_plan_cells="<<unsafe<<std::endl;
 if(before.poses.empty()||after.poses.empty()||deviation(before)>.1||deviation(after)<.35||unsafe)return 3;
 planner->cleanup();planner.reset();
 }
 rclcpp::shutdown();
}

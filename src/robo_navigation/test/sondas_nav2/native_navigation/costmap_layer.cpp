// Exercise the real Jazzy LaserScan callback -> observation buffer -> ObstacleLayer
// entirely in memory. No sensor, velocity, goal, or costmap topic is published.
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
class TestLayer : public nav2_costmap_2d::ObstacleLayer {
public:
  void feed(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    if(observation_buffers_.size()!=1)throw std::runtime_error("expected one configured scan buffer");
    laserScanValidInfCallback(scan,observation_buffers_[0]);
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<nav2_util::LifecycleNode>("global_costmap","/global_costmap");
    auto plugins=node->declare_parameter<std::vector<std::string>>("plugins", {"static_layer","inflation_layer"});
    if(std::find(plugins.begin(),plugins.end(),"obstacle_layer")==plugins.end()) return 2;
    nav2_costmap_2d::LayeredCostmap layered("map",false,false);
    layered.resizeMap(200,200,.05,-5,-5);
    auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped tr;
    tr.header.frame_id="map";tr.child_frame_id="laser";tr.header.stamp=node->now();
    tr.transform.translation.x=.33;tr.transform.translation.y=.005;tr.transform.rotation.w=1;
    tf->setTransform(tr,"offline_fixture",true);
    auto layer=std::make_shared<TestLayer>();
    layered.addPlugin(layer);
    auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer->initialize(&layered,"obstacle_layer",tf.get(),node,group);
    std::cout<<"configured_topic="<<node->get_parameter("obstacle_layer.scan.topic").as_string()
             <<" obstacle_max_range="<<node->get_parameter("obstacle_layer.scan.obstacle_max_range").as_double()
             <<" raytrace_max_range="<<node->get_parameter("obstacle_layer.scan.raytrace_max_range").as_double()<<std::endl;
    layered.updateMap(0,0,0);
    unsigned int mx,my;auto map=layered.getCostmap();map->worldToMap(1.33,.005,mx,my);
    int before=map->getCost(mx,my);
    auto scan=std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.frame_id="laser";scan->header.stamp=node->now();
    scan->angle_min=-.02;scan->angle_max=.02;scan->angle_increment=.01;
    scan->range_min=.08;scan->range_max=5.0;scan->ranges=std::vector<float>(5,1.0);
    layer->feed(scan);layered.updateMap(0,0,0);int marked=map->getCost(mx,my);
    scan->header.stamp=node->now();scan->ranges=std::vector<float>(5,3.0);
    layer->feed(scan);layered.updateMap(0,0,0);int cleared=map->getCost(mx,my);
    std::cout<<"before="<<before<<" marked="<<marked<<" cleared="<<cleared<<std::endl;
    if(before!=0||marked!=254||cleared!=0)return 3;
    layer.reset();tf.reset();node.reset();
  }
  rclcpp::shutdown();
}

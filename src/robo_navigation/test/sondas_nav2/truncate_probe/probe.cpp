#include <iostream>
#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
int main() {
  BT::BehaviorTreeFactory f;
  f.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_truncate_path_local_action_bt_node.so");
  auto bb=BT::Blackboard::create();
  auto clock=std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  auto tf=std::make_shared<tf2_ros::Buffer>(clock); bb->set("tf_buffer",tf);
  nav_msgs::msg::Path path; path.header.frame_id="map";
  for(int i=0;i<=20;i++){geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=i*.1;p.pose.orientation.w=1;path.poses.push_back(p);}
  auto robot=path.poses[10]; bb->set("p",path);bb->set("robot",robot);
  auto tree=f.createTreeFromText(R"(<root BTCPP_format="4" main_tree_to_execute="T"><BehaviorTree ID="T"><TruncatePathLocal input_path="{p}" output_path="{remaining}" pose="{robot}" distance_forward="1000000.0" distance_backward="0.0"/></BehaviorTree></root>)",bb);
  if(tree.tickOnce()!=BT::NodeStatus::SUCCESS)return 2;
  auto out=bb->get<nav_msgs::msg::Path>("remaining");
  if(out.poses.empty()||out.poses.front().pose.position.x<.9||out.poses.back().pose.position.x!=2.)return 3;
  std::cout<<"remaining_points="<<out.poses.size()<<" first_x="<<out.poses.front().pose.position.x<<" last_x="<<out.poses.back().pose.position.x<<std::endl;
}

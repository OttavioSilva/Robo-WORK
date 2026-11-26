// Offline plugin and sampling probe: no cmd_vel publisher, action, or robot connection.
#include <cmath>
#include <iostream>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_core/controller.hpp>
#include <dwb_core/trajectory_generator.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<nav2_util::LifecycleNode>("controller_server", "");
    pluginlib::ClassLoader<nav2_core::Controller> controllers("nav2_core", "nav2_core::Controller");
    auto type = node->declare_parameter<std::string>("FollowPath.plugin", "dwb_core::DWBLocalPlanner");
    auto controller = controllers.createSharedInstance(type);
    std::cout << "configured_plugin=" << type << std::endl;
    pluginlib::ClassLoader<dwb_core::TrajectoryGenerator> loader("dwb_core", "dwb_core::TrajectoryGenerator");
    auto gen = loader.createSharedInstance("dwb_plugins::StandardTrajectoryGenerator");
    gen->initialize(node, "FollowPath");
    std::cout << "vx_samples=" << node->get_parameter("FollowPath.vx_samples").as_int()
              << " vy_samples=" << node->get_parameter("FollowPath.vy_samples").as_int()
              << " theta_samples=" << node->get_parameter("FollowPath.vtheta_samples").as_int() << std::endl;
    if (node->get_parameter("FollowPath.vx_samples").as_int() != 7) return 3;
    nav_2d_msgs::msg::Twist2D stopped;
    auto twists = gen->getTwists(stopped);
    int left=0, right=0, front=0, rear=0, diagonal=0;
    double maxx=0, maxy=0, maxw=0;
    for (const auto &v : twists) {
      maxx=std::max(maxx,std::abs(v.x)); maxy=std::max(maxy,std::abs(v.y)); maxw=std::max(maxw,std::abs(v.theta));
      if (std::abs(v.theta) < 1e-8 && std::abs(v.x) < 1e-8) {
        left += v.y > 0;
        right += v.y < 0;
      }
      front += v.x > 0 && std::abs(v.y) < 1e-8 && std::abs(v.theta) < 1e-8;
      rear += v.x < 0;
      diagonal += v.x > 0 && std::abs(v.y) > 1e-8 && std::abs(v.theta) < 1e-8;
    }
    std::cout << "samples=" << twists.size() << " pure_left=" << left << " pure_right=" << right
              << " pure_front=" << front << " diagonal=" << diagonal << " rear=" << rear
              << " max_x=" << maxx << " max_y=" << maxy << " max_theta=" << maxw << std::endl;
    const bool expect_reverse = node->declare_parameter<bool>("probe.expect_reverse", false);
    if (!left || !right || !front || !diagonal || (expect_reverse ? rear == 0 : rear != 0)) return 2;
  }
  rclcpp::shutdown();
  return 0;
}

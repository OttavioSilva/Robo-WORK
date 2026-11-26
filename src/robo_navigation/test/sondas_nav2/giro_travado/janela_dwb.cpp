// GIRO_TRAVADO (11/11): amostras REAIS do dwb_plugins::StandardTrajectoryGenerator (parametros FollowPath do
// nav2_params.yaml) para uma velocidade atual dada. Lista as velocidades de giro parado (vx=vy=0) que passam no
// isValidSpeed: e o que sobra para o RotateToGoal, que proibe translacao perto do goal.
// Uso: janela_dwb <wz_atual> [--ros-args --params-file nav2_params.yaml -p FollowPath.decel_lim_theta:=X]
#include <cstdio>
#include <memory>
#include <string>
#include "dwb_core/trajectory_generator.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  double wz = argc > 1 ? std::stod(argv[1]) : 0.16;
  auto node = std::make_shared<nav2_util::LifecycleNode>("controller_server");
  pluginlib::ClassLoader<dwb_core::TrajectoryGenerator> ld("dwb_core", "dwb_core::TrajectoryGenerator");
  auto g = ld.createSharedInstance("dwb_plugins::StandardTrajectoryGenerator");
  g->initialize(node, "FollowPath");
  nav_2d_msgs::msg::Twist2D v; v.theta = wz;
  g->startNewIteration(v);
  int total = 0; std::string parado;
  while (g->hasMoreTwists()) {
    auto t = g->nextTwist(); ++total;
    if (t.x == 0.0 && t.y == 0.0) {char b[32]; std::snprintf(b, sizeof b, "%+.3f ", t.theta); parado += b;}
  }
  double dec = node->get_parameter("FollowPath.decel_lim_theta").as_double();
  std::printf("wz_atual=%+.3f decel_lim_theta=%.2f amostras_validas=%d giro_parado=[ %s]\n", wz, dec, total, parado.c_str());
  rclcpp::shutdown();
}

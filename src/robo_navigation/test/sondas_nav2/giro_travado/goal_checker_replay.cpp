// GIRO_TRAVADO (11/11): replay do goal checker REAL (nav2_controller::SimpleGoalChecker via pluginlib, parametros
// do nav2_params.yaml) com a conversao REAL do controller_server (nav_2d_utils::transformPose) e o map->odom do bag.
// Entrada: fixture_map_odom.csv (stamp x y yaw) e fixture_ciclos.csv (t rx ry ryaw stamp_fim fx fy fyaw).
// Modo "atual": fim do caminho com o carimbo da pose (como controller_server::setPlannerPath guarda end_pose_).
// Modo "carimbo0": fim do caminho com carimbo zero (correcao proposta: TF mais recente, igual ao DWB).
// Modo "arvore" (GOAL_TF_ATUAL, 11/11): o caminho passa pelo no CaminhoSemCarimbo REAL (biblioteca em
// $ROBO_SEM_CARIMBO_LIB) e o fim do caminho que sai dele vai ao goal checker, como no FollowPath da arvore aplicada.
// O map->odom entra no buffer so quando "chega" (carimbo - 0,30 s <= t do ciclo), como no robo.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include <cstdlib>
struct MO {double s, x, y, w;};
struct Ci {double t, rx, ry, rw, sf, fx, fy, fw;};
static geometry_msgs::msg::Quaternion q(double w) {tf2::Quaternion a; a.setRPY(0, 0, w); return tf2::toMsg(a);}
static builtin_interfaces::msg::Time stamp(double s) {
  builtin_interfaces::msg::Time t; t.sec = static_cast<int32_t>(std::floor(s));
  t.nanosec = static_cast<uint32_t>((s - std::floor(s)) * 1e9); return t;}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string dir = argc > 1 ? argv[1] : ".";
  for (int i = 1; i < argc; ++i) {std::string a = argv[i]; if (a == "--ros-args") break; dir = a; break;}
  std::vector<MO> mo; std::vector<Ci> ci;
  {std::ifstream f(dir + "/fixture_map_odom.csv"); MO m; while (f >> m.s >> m.x >> m.y >> m.w) mo.push_back(m);}
  {std::ifstream f(dir + "/fixture_ciclos.csv"); Ci c; while (f >> c.t >> c.rx >> c.ry >> c.rw >> c.sf >> c.fx >> c.fy >> c.fw) ci.push_back(c);}
  if (mo.empty() || ci.empty()) {std::cerr << "fixture vazia em " << dir << "\n"; return 2;}
  auto node = std::make_shared<nav2_util::LifecycleNode>("controller_server");
  pluginlib::ClassLoader<nav2_core::GoalChecker> ld("nav2_core", "nav2_core::GoalChecker");
  std::string plugin = node->declare_parameter<std::string>("goal_checker.plugin", "nav2_controller::SimpleGoalChecker");
  int ret = 0;
  std::vector<std::string> modos = {"atual", "carimbo0"};
  BT::BehaviorTreeFactory fab; BT::Tree arv; BT::Blackboard::Ptr bb;
  if (const char * lib = std::getenv("ROBO_SEM_CARIMBO_LIB")) {
    fab.registerFromPlugin(lib);
    bb = BT::Blackboard::create();
    arv = fab.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="T">
      <CaminhoSemCarimbo input_path="{smoothed_path}" output_path="{caminho_controlador}"/></BehaviorTree></root>)", bb);
    modos.push_back("arvore");
  }
  for (std::string modo : modos) {
    auto gc = ld.createSharedInstance(plugin);
    gc->initialize(node, "goal_checker", nullptr);
    // cache de 10 s = padrao do tf2 e do buffer do Costmap2DROS que o controller_server usa (getTfBuffer)
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock(), tf2::durationFromSec(10.0));
    tf->setUsingDedicatedThread(true);
    size_t k = 0; double ult = -1; double chegou = -1; int ciclos_xy = 0; double chegou_c = -1;
    rclcpp::Duration tol = rclcpp::Duration::from_seconds(0.3);
    for (const auto & c : ci) {
      while (k < mo.size() && mo[k].s - 0.30 <= c.t) {
        geometry_msgs::msg::TransformStamped tr; tr.header.frame_id = "map"; tr.child_frame_id = "odom";
        tr.header.stamp = stamp(mo[k].s); tr.transform.translation.x = mo[k].x; tr.transform.translation.y = mo[k].y;
        tr.transform.rotation = q(mo[k].w); tf->setTransform(tr, "bag", false); ++k;
      }
      if (c.sf != ult) {   // controller_server::setPlannerPath -> reset()
        if (ult > 0 && std::getenv("POR_CAMINHO")) {std::printf("caminho modo=%s sf=%.3f chegou_t=%.3f\n", modo.c_str(), ult, chegou_c);}
        gc->reset(); ult = c.sf; chegou_c = -1;
      }
      geometry_msgs::msg::PoseStamped fim, fim_odom;
      fim.header.frame_id = "map"; fim.header.stamp = modo == "atual" ? stamp(c.sf) : builtin_interfaces::msg::Time();
      fim.pose.position.x = c.fx; fim.pose.position.y = c.fy; fim.pose.orientation = q(c.fw);
      if (modo == "arvore") {   // caminho com o carimbo do planner -> no real -> ultima pose (end_pose_)
        nav_msgs::msg::Path p; p.header.frame_id = "map"; p.header.stamp = stamp(c.sf);
        auto f2 = fim; f2.header.stamp = stamp(c.sf); p.poses.push_back(f2);
        bb->set<nav_msgs::msg::Path>("smoothed_path", p);
        if (arv.tickOnce() != BT::NodeStatus::SUCCESS) {std::cerr << "no CaminhoSemCarimbo falhou\n"; return 3;}
        auto out = bb->get<nav_msgs::msg::Path>("caminho_controlador");
        fim = out.poses.back(); fim.header.frame_id = out.header.frame_id;   // como setPlannerPath
      }
      nav_2d_utils::transformPose(tf, "odom", fim, fim_odom, tol);   // retorno ignorado, como no controller_server
      geometry_msgs::msg::Pose robo; robo.position.x = c.rx; robo.position.y = c.ry; robo.orientation = q(c.rw);
      const double d = std::hypot(c.rx - fim_odom.pose.position.x, c.ry - fim_odom.pose.position.y);
      ciclos_xy += d <= 0.06;
      const bool ok_ciclo = gc->isGoalReached(robo, fim_odom.pose, geometry_msgs::msg::Twist());
      if (std::getenv("POR_CICLO")) {std::printf("ciclo modo=%s t=%.3f chegou=%d\n", modo.c_str(), c.t, ok_ciclo ? 1 : 0);}
      if (ok_ciclo) {
        if (chegou < 0) {chegou = c.t;}
        if (chegou_c < 0) {chegou_c = c.t;}
      }
    }
    if (ult > 0 && std::getenv("POR_CAMINHO")) {std::printf("caminho modo=%s sf=%.3f chegou_t=%.3f\n", modo.c_str(), ult, chegou_c);}
    std::printf("modo=%s chegou=%s t=%.2f ciclos_dentro_xy=%d\n", modo.c_str(), chegou > 0 ? "SIM" : "NAO",
      chegou > 0 ? chegou - ci.front().t : -1.0, ciclos_xy);
  }
  rclcpp::shutdown();
  return ret;
}

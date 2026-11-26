// CAMINHO_ORIGINAL (11/11/2026) - ver include/robo_bt_plugins/caminho_original_seguro_condition.hpp
#include "robo_bt_plugins/caminho_original_seguro_condition.hpp"
#include <cmath>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/json_export.h"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"

namespace robo_bt_plugins
{

CaminhoOriginalSeguro::CaminhoOriginalSeguro(const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  try {
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  } catch (...) {
    tf_.reset();
  }
  std::string topico = "global_costmap/costmap_raw", fp = "", gf = "map", rf = "base_link";
  double padding = 0.01;
  getInput("costmap_topic", topico);
  getInput("footprint", fp);
  getInput("footprint_padding", padding);
  getInput("global_frame", gf);
  getInput("robot_base_frame", rf);
  transform_tolerance_ = 0.3;
  getInput("transform_tolerance", transform_tolerance_);
  global_frame_ = gf;
  robot_frame_ = rf;
  fp_ = fp.empty() ? footprintPadrao() : footprintDeString(fp, padding);
  if (fp_.size() < 3) {
    RCLCPP_ERROR(node_->get_logger(), "CAMINHO_ORIGINAL: footprint '%s' invalido; usando o padrao", fp.c_str());
    fp_ = footprintPadrao();
  }
  std::string topico_modo = "/caminho_original/modo";
  getInput("modo_topic", topico_modo);
  const bool usar_costmap = !topico.empty() && topico != "nenhum";
  const bool usar_modo = !topico_modo.empty() && topico_modo != "nenhum";
  if (usar_costmap || usar_modo) {
    grupo_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(grupo_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions op;
    op.callback_group = grupo_;
    if (usar_costmap) {
      sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
        topico, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&CaminhoOriginalSeguro::costmapCb, this, std::placeholders::_1), op);
    }
    if (usar_modo) {
      sub_modo_ = node_->create_subscription<std_msgs::msg::String>(
        topico_modo, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        [this](const std_msgs::msg::String::SharedPtr m) {modo_topico_ = m->data;}, op);
    }
  }
}

BT::PortsList CaminhoOriginalSeguro::providedPorts()
{
  BT::RegisterJsonDefinition<nav_msgs::msg::Path>();
  BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Caminho original do planner"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Goal (yaw da ultima pose)"),
    BT::InputPort<std::string>("modo", "registrar", "aplicar | registrar"),
    BT::InputPort<double>("folga_inicio", 0.30, "m iniciais nao conferidos (onde o robo ja esta)"),
    BT::InputPort<double>("folga_fim", 0.60, "m finais nao conferidos (zona do goal)"),
    BT::InputPort<double>("tolerancia_lateral", 0.10, "deslocamento lateral que ainda conta como rasante"),
    BT::InputPort<double>("idade_max_costmap", 3.0, "s; costmap mais velho: nao confere"),
    BT::InputPort<std::string>("costmap_topic", "global_costmap/costmap_raw", "nav2_msgs/Costmap"),
    BT::InputPort<std::string>("modo_topic", "/caminho_original/modo", "std_msgs/String; nenhum = sem topico"),
    BT::InputPort<std::string>("footprint", "", "vazio = footprint do robo com padding"),
    BT::InputPort<double>("footprint_padding", 0.01, "padding somado ao footprint da porta"),
    BT::InputPort<std::string>("global_frame", "map", ""),
    BT::InputPort<std::string>("robot_base_frame", "base_link", ""),
    BT::InputPort<double>("transform_tolerance", 0.3, ""),
  };
}

void CaminhoOriginalSeguro::costmapCb(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  const auto & m = msg->metadata;
  auto cm = std::make_shared<nav2_costmap_2d::Costmap2D>(m.size_x, m.size_y, m.resolution,
      m.origin.position.x, m.origin.position.y);
  unsigned char * dst = cm->getCharMap();
  const size_t n = static_cast<size_t>(m.size_x) * m.size_y;
  if (msg->data.size() < n) {return;}
  std::copy(msg->data.begin(), msg->data.begin() + n, dst);
  std::lock_guard<std::mutex> lk(mtx_);
  costmap_ = cm;
  t_costmap_ = node_->now();
}

void CaminhoOriginalSeguro::injetarCostmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> cm)
{
  std::lock_guard<std::mutex> lk(mtx_);
  costmap_ = cm;
  t_costmap_ = node_->now();
}

std::string CaminhoOriginalSeguro::modoAtual()
{
  std::string modo = "registrar";
  getInput("modo", modo);
  if (modo_topico_ == "aplicar" || modo_topico_ == "registrar") {modo = modo_topico_;}
  return modo;
}

BT::NodeStatus CaminhoOriginalSeguro::tick()
{
  if (grupo_) {executor_.spin_some();}
  const std::string modo = modoAtual();
  const bool aplicar = (modo != "registrar");
  auto log = node_->get_logger();

  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {
    RCLCPP_WARN(log, "CAMINHO_ORIGINAL: caminho vazio%s", aplicar ? ": reprovado" : " (registrar)");
    return aplicar ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
  rclcpp::Time t_cm;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cm = costmap_;
    t_cm = t_costmap_;
  }
  double idade_max = 3.0;
  getInput("idade_max_costmap", idade_max);
  if (!cm) {
    RCLCPP_WARN(log, "CAMINHO_ORIGINAL: sem costmap ainda: sigo com o original sem conferir (como antes)");
    return BT::NodeStatus::SUCCESS;
  }
  const double idade = (node_->now() - t_cm).seconds();
  if (idade_max > 0.0 && idade > idade_max) {
    RCLCPP_WARN(log, "CAMINHO_ORIGINAL: costmap com %.1f s: sigo com o original sem conferir (como antes)", idade);
    return BT::NodeStatus::SUCCESS;
  }
  geometry_msgs::msg::PoseStamped robo;
  if (!tf_ || !nav2_util::getCurrentPose(robo, *tf_, global_frame_, robot_frame_, transform_tolerance_)) {
    RCLCPP_WARN(log, "CAMINHO_ORIGINAL: sem pose do robo: sigo com o original sem conferir (como antes)");
    return BT::NodeStatus::SUCCESS;
  }
  std::vector<Pose2> pts;
  pts.reserve(path.poses.size());
  for (const auto & p : path.poses) {
    pts.push_back({p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)});
  }
  double yaw_goal = pts.back().yaw;
  geometry_msgs::msg::PoseStamped goal;
  if (getInput("goal", goal)) {yaw_goal = tf2::getYaw(goal.pose.orientation);}
  ParamsCaminho prm;
  getInput("folga_inicio", prm.folga_inicio);
  getInput("folga_fim", prm.folga_fim);
  getInput("tolerancia_lateral", prm.tolerancia_lateral);
  const auto r = avaliarCaminhoOriginal(*cm, pts,
    Pose2{robo.pose.position.x, robo.pose.position.y, tf2::getYaw(robo.pose.orientation)}, yaw_goal, fp_, prm);
  if (r.valido) {
    RCLCPP_INFO_THROTTLE(log, *node_->get_clock(), 2000,
      "CAMINHO_ORIGINAL: suavizado reprovado, original conferido e liberado (%d poses, %d rasantes)",
      r.poses_conferidas, r.poses_rasantes);
    return BT::NodeStatus::SUCCESS;
  }
  if (!aplicar) {
    RCLCPP_WARN(log, "CAMINHO_ORIGINAL (registrar): REPROVARIA: %s", r.motivo.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(log, "CAMINHO_ORIGINAL: REPROVADO (vai para a recuperacao): %s", r.motivo.c_str());
  return BT::NodeStatus::FAILURE;
}
}  // namespace robo_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robo_bt_plugins::CaminhoOriginalSeguro>("CaminhoOriginalSeguro");
}

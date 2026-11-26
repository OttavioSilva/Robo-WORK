// CAMINHO_ORIGINAL (11/11/2026) - Patch 01: condicao de BT CaminhoOriginalSeguro.
// SUCCESS: o caminho original pode ir ao FollowPath. FAILURE: bloqueado (so no modo "aplicar").
// Sem costmap ou sem pose do robo devolve SUCCESS (igual a arvore anterior) e avisa no log.
// Modo: porta `modo` ("registrar" | "aplicar"; padrao registrar). Em tempo de execucao, a ultima mensagem
// do topico /caminho_original/modo (std_msgs/String, transient_local) tem prioridade se nao vazia:
//   ros2 topic pub --once --qos-durability transient_local /caminho_original/modo std_msgs/msg/String "{data: aplicar}"
// (O no cliente do BT nao e atendido por executor: `ros2 param set` nele trava. Por isso topico.)
// "registrar" so escreve no log o que faria e sempre devolve SUCCESS.
#pragma once
#include <memory>
#include <mutex>
#include <string>
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robo_bt_plugins/caminho_original.hpp"
#include "tf2_ros/buffer.h"

namespace robo_bt_plugins
{
class CaminhoOriginalSeguro : public BT::ConditionNode
{
public:
  CaminhoOriginalSeguro(const std::string & name, const BT::NodeConfiguration & conf);
  CaminhoOriginalSeguro() = delete;
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

  // para o teste offline: injeta o costmap sem topico
  void injetarCostmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> cm);
  void injetarModo(const std::string & m) {modo_topico_ = m;}

private:
  void costmapCb(const nav2_msgs::msg::Costmap::SharedPtr msg);
  std::string modoAtual();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr grupo_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_modo_;
  std::string modo_topico_;
  std::mutex mtx_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  rclcpp::Time t_costmap_;
  Footprint fp_;
  std::string global_frame_, robot_frame_;
  double transform_tolerance_;
};
}  // namespace robo_bt_plugins

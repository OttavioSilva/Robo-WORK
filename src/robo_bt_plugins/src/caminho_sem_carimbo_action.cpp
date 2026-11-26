// GOAL_TF_ATUAL (11/11/2026): copia o caminho com o carimbo de tempo ZERADO (header e poses), mantendo frame e poses.
// Motivo (bag 2nav 12:37:22-41, fonte do Nav2 1.3.12): o controller_server guarda end_pose_ = path.poses.back() com o
// carimbo da pose e o converte para odom em cada ciclo com o map->odom DAQUELE instante (nav_2d_utils::transformPose
// -> tf->transform), enquanto o DWB (prepareGlobalPlan) usa carimbo zero (map->odom mais recente). Com o SLAM
// corrigindo o map->odom, o goal checker via o alvo a 7-55 cm do que o DWB via e nunca encerrava; o DWB girava no
// goal. Com carimbo zero os dois usam o mesmo map->odom.
#include <string>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/json_export.h"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robo_bt_plugins
{
class CaminhoSemCarimbo : public BT::SyncActionNode
{
public:
  CaminhoSemCarimbo(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}
  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "caminho (do planner/smoother)"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "mesmo caminho com carimbo zero"),
    };
  }
  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path p;
    if (!getInput("input_path", p) || p.poses.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    p.header.stamp = builtin_interfaces::msg::Time();
    for (auto & ps : p.poses) {
      ps.header.stamp = builtin_interfaces::msg::Time();   // frame_id e pose ficam como vieram
    }
    setOutput("output_path", p);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace robo_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robo_bt_plugins::CaminhoSemCarimbo>("CaminhoSemCarimbo");
}

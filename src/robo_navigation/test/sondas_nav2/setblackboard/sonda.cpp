// NAV_INTEGRADA: prova que SetBlackboard value="{path}" copia um nav_msgs::Path tipado (BT.CPP instalado).
#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/msg/path.hpp>
#include <iostream>
using Path = nav_msgs::msg::Path;
static bool planner_ok = true;
static int seguiu = 0;
class Produz : public BT::SyncActionNode {
 public:
  Produz(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {BT::OutputPort<Path>("path")}; }
  BT::NodeStatus tick() override {
    if (!planner_ok) return BT::NodeStatus::FAILURE;
    Path p; p.header.frame_id = "map"; p.poses.resize(7); setOutput("path", p); return BT::NodeStatus::SUCCESS; }
};
class Falha : public BT::SyncActionNode {  // como o SmoothPath que falhou: nao escreve a saida
 public:
  Falha(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<Path>("unsmoothed_path"), BT::OutputPort<Path>("smoothed_path")}; }
  BT::NodeStatus tick() override { return BT::NodeStatus::FAILURE; }
};
class Consome : public BT::SyncActionNode {
 public:
  Consome(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<Path>("path")}; }
  BT::NodeStatus tick() override {
    auto p = getInput<Path>("path");
    if (!p) { std::cout << "consumo_falhou=" << p.error() << std::endl; return BT::NodeStatus::FAILURE; }
    std::cout << "consumido_poses=" << p->poses.size() << " frame=" << p->header.frame_id << std::endl;
    seguiu++;
    return p->poses.size() == 7 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
int main() {
  BT::BehaviorTreeFactory f;
  f.registerNodeType<Produz>("ComputePathToPose");
  f.registerNodeType<Falha>("SmoothPath");
  f.registerNodeType<Consome>("FollowPath");
  const char* xml = R"(
<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence>
  <ComputePathToPose path="{path}"/>
  <Fallback>
    <SmoothPath unsmoothed_path="{path}" smoothed_path="{smoothed_path}"/>
    <SetBlackboard output_key="smoothed_path" value="{path}"/>
  </Fallback>
  <FollowPath path="{smoothed_path}"/>
</Sequence></BehaviorTree></root>)";
  auto tree = f.createTreeFromText(xml);
  auto st = tree.tickWhileRunning();
  std::cout << "resultado=" << BT::toStr(st) << std::endl;
  if (st != BT::NodeStatus::SUCCESS) return 1;

  // CAMINHO_DO_GOAL: mesma arvore (mesmo blackboard) para dois goals, como o bt_navigator faz.
  const char* xml2 = R"(
<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence>
  <Script code="caminho_do_goal := false"/>
  <Fallback>
    <Sequence>
      <ScriptCondition code="caminho_do_goal"/>
      <FollowPath path="{smoothed_path}"/>
    </Sequence>
    <Sequence>
      <Script code="caminho_do_goal := false"/>
      <ComputePathToPose path="{path}"/>
      <Fallback>
        <SmoothPath unsmoothed_path="{path}" smoothed_path="{smoothed_path}"/>
        <SetBlackboard output_key="smoothed_path" value="{path}"/>
      </Fallback>
      <Script code="caminho_do_goal := true"/>
      <FollowPath path="{smoothed_path}"/>
    </Sequence>
  </Fallback>
</Sequence></BehaviorTree></root>)";
  auto t2 = f.createTreeFromText(xml2);
  seguiu = 0; planner_ok = true;
  auto g1 = t2.tickWhileRunning(); t2.haltTree();
  int seguiu_g1 = seguiu;
  seguiu = 0; planner_ok = false;
  auto g2 = t2.tickWhileRunning(); t2.haltTree();
  std::cout << "goal1=" << BT::toStr(g1) << " seguiu=" << seguiu_g1
            << " goal2_sem_planner=" << BT::toStr(g2) << " seguiu=" << seguiu << std::endl;
  bool ok = g1 == BT::NodeStatus::SUCCESS && seguiu_g1 == 1 && g2 == BT::NodeStatus::FAILURE && seguiu == 0;
  std::cout << (ok ? "caminho_do_goal=OK" : "caminho_do_goal=FALHOU") << std::endl;
  return ok ? 0 : 1;
}

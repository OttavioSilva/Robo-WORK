// GOAL_TF_ATUAL (11/11): o no zera os carimbos, preserva frame e poses e e estavel (mesma entrada -> mesma saida,
// para o FollowPath do BT nao ver "caminho novo" e resetar o goal checker a toa).
#include <gtest/gtest.h>
#include "behaviortree_cpp/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
static nav_msgs::msg::Path caminho(int sec)
{
  nav_msgs::msg::Path p; p.header.frame_id = "map"; p.header.stamp.sec = sec;
  for (int i = 0; i < 5; ++i) {
    geometry_msgs::msg::PoseStamped ps; ps.header.frame_id = "map"; ps.header.stamp.sec = sec;
    ps.pose.position.x = 0.1 * i; ps.pose.orientation.w = 1.0; p.poses.push_back(ps);
  }
  return p;
}
TEST(CaminhoSemCarimbo, ZeraCarimboEPreservaOResto)
{
  BT::BehaviorTreeFactory f;
  f.registerFromPlugin(LIB_SEM_CARIMBO);
  auto bb = BT::Blackboard::create();
  bb->set<nav_msgs::msg::Path>("smoothed_path", caminho(1794325046));
  auto t = f.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="T">
    <CaminhoSemCarimbo input_path="{smoothed_path}" output_path="{caminho_controlador}"/></BehaviorTree></root>)", bb);
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  auto out = bb->get<nav_msgs::msg::Path>("caminho_controlador");
  auto in = caminho(1794325046);
  ASSERT_EQ(out.poses.size(), in.poses.size());
  EXPECT_EQ(out.header.stamp.sec, 0); EXPECT_EQ(out.header.frame_id, "map");
  for (size_t i = 0; i < out.poses.size(); ++i) {
    EXPECT_EQ(out.poses[i].header.stamp.sec, 0); EXPECT_EQ(out.poses[i].header.stamp.nanosec, 0u);
    EXPECT_EQ(out.poses[i].header.frame_id, "map");
    EXPECT_EQ(out.poses[i].pose, in.poses[i].pose);
  }
  // estavel: o mesmo caminho com carimbo diferente vira a MESMA saida
  bb->set<nav_msgs::msg::Path>("smoothed_path", caminho(1794325999));
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("caminho_controlador"), out);
}
TEST(CaminhoSemCarimbo, NaoMexeNaEntradaNemNoFrame)
{
  // a entrada (smoothed_path, usada pelo IsPathValid/TruncatePathLocal) fica intacta; frame vazio continua vazio
  BT::BehaviorTreeFactory f;
  f.registerFromPlugin(LIB_SEM_CARIMBO);
  auto bb = BT::Blackboard::create();
  auto in = caminho(1794325046);
  in.poses[2].header.frame_id = "";
  bb->set<nav_msgs::msg::Path>("smoothed_path", in);
  auto t = f.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="T">
    <CaminhoSemCarimbo input_path="{smoothed_path}" output_path="{caminho_controlador}"/></BehaviorTree></root>)", bb);
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("smoothed_path"), in);
  auto out = bb->get<nav_msgs::msg::Path>("caminho_controlador");
  EXPECT_EQ(out.poses[2].header.frame_id, "");
  EXPECT_EQ(out.poses[1].header.frame_id, "map");
}

TEST(CaminhoSemCarimbo, CaminhoVazioFalha)
{
  BT::BehaviorTreeFactory f;
  f.registerFromPlugin(LIB_SEM_CARIMBO);
  auto bb = BT::Blackboard::create();
  bb->set<nav_msgs::msg::Path>("smoothed_path", nav_msgs::msg::Path());
  auto t = f.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="T">
    <CaminhoSemCarimbo input_path="{smoothed_path}" output_path="{caminho_controlador}"/></BehaviorTree></root>)", bb);
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
}

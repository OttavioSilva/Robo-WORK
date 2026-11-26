// CAMINHO_ORIGINAL (11/11/2026): o no de BT dentro de uma arvore BT.CPP real (sem Nav2 rodando,
// sem topico: costmap injetado, TF estatico em memoria). Nao publica nada.
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robo_bt_plugins/caminho_original_seguro_condition.hpp"
#include "tf2_ros/buffer.h"
using robo_bt_plugins::CaminhoOriginalSeguro;

class Fixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("teste_caminho_original_bt");
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map"; t.child_frame_id = "base_link"; t.header.stamp = node->now();
    t.transform.rotation.w = 1.0;
    tf->setTransform(t, "teste", true);
    factory.registerNodeType<CaminhoOriginalSeguro>("CaminhoOriginalSeguro");
    cm = std::make_shared<nav2_costmap_2d::Costmap2D>(120, 120, 0.05, -1.0, -3.0, 0);
  }
  void caixa(double x0, double x1, double y0, double y1)
  {
    for (unsigned int i = 0; i < cm->getSizeInCellsX(); ++i) {
      for (unsigned int j = 0; j < cm->getSizeInCellsY(); ++j) {
        double x, y; cm->mapToWorld(i, j, x, y);
        if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {cm->setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);}
      }
    }
  }
  nav_msgs::msg::Path reta(double x1)
  {
    nav_msgs::msg::Path p; p.header.frame_id = "map";
    for (int i = 0; i <= static_cast<int>(x1 / 0.05); ++i) {
      geometry_msgs::msg::PoseStamped ps; ps.header.frame_id = "map";
      ps.pose.position.x = i * 0.05; ps.pose.orientation.w = 1.0; p.poses.push_back(ps);
    }
    return p;
  }
  // arvore com o MESMO formato do fallback instalado na navigate_to_pose_smooth.xml
  BT::NodeStatus rodar(const nav_msgs::msg::Path & path, bool injetar = true, const std::string & modo = "aplicar")
  {
    const std::string xml = R"(
<root BTCPP_format="4"><BehaviorTree ID="T">
  <Sequence>
    <Script code="caminho_bloqueado := false" />
    <Fallback name="SuavizadoOuOriginal">
      <AlwaysFailure name="SmoothPathReprovou" />
      <Sequence name="OriginalConferido">
        <CaminhoOriginalSeguro path="{path}" goal="{goal}" modo=")" + modo + R"(" costmap_topic="nenhum" modo_topic="nenhum" />
        <SetBlackboard output_key="smoothed_path" value="{path}" />
      </Sequence>
      <Sequence name="OriginalBloqueado">
        <Script code="caminho_bloqueado := true" />
        <AlwaysFailure />
      </Sequence>
    </Fallback>
  </Sequence>
</BehaviorTree></root>)";
    bb = BT::Blackboard::create();
    bb->set<rclcpp::Node::SharedPtr>("node", node);
    bb->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf);
    bb->set<nav_msgs::msg::Path>("path", path);
    geometry_msgs::msg::PoseStamped g; g.pose.orientation.w = 1.0;
    bb->set<geometry_msgs::msg::PoseStamped>("goal", g);
    tree = factory.createTreeFromText(xml, bb);
    if (injetar) {
      tree.applyVisitor([this](BT::TreeNode * n) {
          if (auto c = dynamic_cast<CaminhoOriginalSeguro *>(n)) {c->injetarCostmap(cm);}
        });
    }
    return tree.tickOnce();
  }
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> cm;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr bb;
  BT::Tree tree;
};

TEST_F(Fixture, OriginalLivreVaiParaOFollowPath)
{
  auto p = reta(3.0);
  EXPECT_EQ(rodar(p), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("smoothed_path").poses.size(), p.poses.size());
  EXPECT_FALSE(bb->get<bool>("caminho_bloqueado"));
}

TEST_F(Fixture, OriginalBloqueadoMarcaEFalha)
{
  caixa(1.5, 1.6, -0.05, 0.05);
  EXPECT_EQ(rodar(reta(3.0)), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(bb->get<bool>("caminho_bloqueado"));
  EXPECT_THROW(bb->get<nav_msgs::msg::Path>("smoothed_path"), std::exception);  // nada foi ao FollowPath
}

TEST_F(Fixture, ModoRegistrarNaoMudaComportamento)
{
  caixa(1.5, 1.6, -0.05, 0.05);
  EXPECT_EQ(rodar(reta(3.0), true, "registrar"), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(bb->get<bool>("caminho_bloqueado"));
}

TEST_F(Fixture, ModoDoTopicoTemPrioridade)
{
  caixa(1.5, 1.6, -0.05, 0.05);
  auto com_modo = [this](const std::string & m) {
      tree.applyVisitor([&](BT::TreeNode * n) {
          if (auto c = dynamic_cast<CaminhoOriginalSeguro *>(n)) {c->injetarModo(m);}
        });
      bb->set<bool>("caminho_bloqueado", false);
      return tree.tickExactlyOnce();
    };
  rodar(reta(3.0));
  tree.haltTree();
  EXPECT_EQ(com_modo("registrar"), BT::NodeStatus::SUCCESS);
  tree.haltTree();
  EXPECT_EQ(com_modo("aplicar"), BT::NodeStatus::FAILURE);
  tree.haltTree();
  EXPECT_EQ(com_modo("lixo"), BT::NodeStatus::FAILURE);  // valor invalido: fica a porta (aplicar)
}

TEST_F(Fixture, TopicoDeModoRealTrocaParaRegistrar)
{
  // publica no topico de verdade (dominio isolado do teste) e confere que o no le no tick
  caixa(1.5, 1.6, -0.05, 0.05);
  const std::string xml = R"(<root BTCPP_format="4"><BehaviorTree ID="T">
    <CaminhoOriginalSeguro path="{path}" modo="aplicar" costmap_topic="nenhum" modo_topic="/teste_caminho_original/modo" />
  </BehaviorTree></root>)";
  bb = BT::Blackboard::create();
  bb->set<rclcpp::Node::SharedPtr>("node", node);
  bb->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf);
  bb->set<nav_msgs::msg::Path>("path", reta(3.0));
  tree = factory.createTreeFromText(xml, bb);
  tree.applyVisitor([this](BT::TreeNode * n) {
      if (auto c = dynamic_cast<CaminhoOriginalSeguro *>(n)) {c->injetarCostmap(cm);}
    });
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);
  auto pub = node->create_publisher<std_msgs::msg::String>("/teste_caminho_original/modo",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  std_msgs::msg::String m; m.data = "registrar"; pub->publish(m);
  BT::NodeStatus st = BT::NodeStatus::FAILURE;
  for (int i = 0; i < 50 && st != BT::NodeStatus::SUCCESS; ++i) {
    rclcpp::sleep_for(std::chrono::milliseconds(20));
    tree.haltTree();
    st = tree.tickExactlyOnce();
  }
  EXPECT_EQ(st, BT::NodeStatus::SUCCESS);
}

TEST_F(Fixture, SemCostmapSegueComoAntes)
{
  caixa(1.5, 1.6, -0.05, 0.05);
  EXPECT_EQ(rodar(reta(3.0), false), BT::NodeStatus::SUCCESS);
}

TEST_F(Fixture, CaminhoVazioReprova)
{
  EXPECT_EQ(rodar(nav_msgs::msg::Path()), BT::NodeStatus::FAILURE);
}

// CAMINHO_ORIGINAL (11/11/2026): testes do nucleo, costmap sintetico (sem ROS em execucao).
#include <gtest/gtest.h>
#include <cmath>
#include "nav2_costmap_2d/cost_values.hpp"
#include "robo_bt_plugins/caminho_original.hpp"
using namespace robo_bt_plugins;
using nav2_costmap_2d::LETHAL_OBSTACLE;

// mapa 6x6 m, 5 cm, origem (-1,-3)
static nav2_costmap_2d::Costmap2D mapa() {return nav2_costmap_2d::Costmap2D(120, 120, 0.05, -1.0, -3.0, 0);}
static void caixa(nav2_costmap_2d::Costmap2D & cm, double x0, double x1, double y0, double y1, unsigned char v = LETHAL_OBSTACLE)
{
  for (unsigned int i = 0; i < cm.getSizeInCellsX(); ++i) {
    for (unsigned int j = 0; j < cm.getSizeInCellsY(); ++j) {
      double x, y; cm.mapToWorld(i, j, x, y);
      if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {cm.setCost(i, j, v);}
    }
  }
}
static std::vector<Pose2> reta(double x0, double y0, double x1, double y1, double passo = 0.05)
{
  std::vector<Pose2> p;
  const int n = static_cast<int>(std::hypot(x1 - x0, y1 - y0) / passo);
  for (int i = 0; i <= n; ++i) {p.push_back({x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, 0.0});}  // yaw 0 como o NavFn
  return p;
}
static ParamsCaminho prm() {return ParamsCaminho();}

TEST(CaminhoOriginal, AreaLivreValida)
{
  auto cm = mapa();
  auto r = avaliarCaminhoOriginal(cm, reta(0, 0, 3, 0), Pose2{0, 0, 0.0}, 0, footprintPadrao(), prm());
  EXPECT_TRUE(r.valido); EXPECT_EQ(r.poses_rasantes, 0); EXPECT_GT(r.poses_conferidas, 40);
}

TEST(CaminhoOriginal, CaixaNoMeioReprova)
{
  auto cm = mapa();
  caixa(cm, 1.5, 1.6, -0.05, 0.05);  // obstaculo 10x10 cm no eixo do caminho
  auto r = avaliarCaminhoOriginal(cm, reta(0, 0, 3, 0), Pose2{0, 0, 0.0}, 0, footprintPadrao(), prm());
  EXPECT_FALSE(r.valido);
  EXPECT_GT(r.dist_robo, 0.8);
}

TEST(CaminhoOriginal, CorredorDe80cmComRasanteNaoReprova)
{
  // corredor de 0,78 m (paredes em y=+-0.39): robo de 0,65 m; caminho 3 cm fora do centro
  auto cm = mapa();
  caixa(cm, -1, 5, 0.39, 0.6); caixa(cm, -1, 5, -0.6, -0.39);
  auto pts = reta(0, 0.03, 3, 0.03);
  auto r = avaliarCaminhoOriginal(cm, pts, Pose2{0, 0.03, 0.0}, 0, footprintPadrao(), prm());
  EXPECT_TRUE(r.valido) << r.motivo;
}

TEST(CaminhoOriginal, CorredorEstreitoDemaisReprova)
{
  // 0,60 m de vao: o robo de 0,65 m nao passa com deslocamento nenhum
  auto cm = mapa();
  caixa(cm, 1.0, 2.0, 0.30, 0.6); caixa(cm, 1.0, 2.0, -0.6, -0.30);
  auto r = avaliarCaminhoOriginal(cm, reta(0, 0, 3, 0), Pose2{0, 0, 0.0}, 0, footprintPadrao(), prm());
  EXPECT_FALSE(r.valido);
}

TEST(CaminhoOriginal, ObstaculoOndeORoboJaEstaNaoReprova)
{
  // ruido/inflacao encostando no footprint atual: zona de folga_inicio
  auto cm = mapa();
  caixa(cm, 0.0, 0.1, 0.30, 0.35);
  caixa(cm, 0.0, 0.1, -0.35, -0.30);
  auto r = avaliarCaminhoOriginal(cm, reta(0, 0, 3, 0), Pose2{0, 0, 0.0}, 0, footprintPadrao(), prm());
  EXPECT_TRUE(r.valido) << r.motivo;
}

TEST(CaminhoOriginal, TrechoJaPercorridoNaoConta)
{
  // caixa atras do robo (robo em x=2): so o trecho restante e conferido
  auto cm = mapa();
  caixa(cm, 0.9, 1.0, -0.05, 0.05);
  auto r = avaliarCaminhoOriginal(cm, reta(0, 0, 3.5, 0), Pose2{2.0, 0, 0.0}, 0, footprintPadrao(), prm());
  EXPECT_TRUE(r.valido) << r.motivo;
}

TEST(CaminhoOriginal, ZonaDoGoalNaoConta)
{
  // goal com a frente a 3 cm de uma parede: a zona final e do goal checker/CM/alinhador
  auto cm = mapa();
  caixa(cm, 3.35, 3.6, -1, 1);  // frente do robo no goal em 3,375
  auto r = avaliarCaminhoOriginal(cm, reta(0, 0, 3.0, 0), Pose2{0, 0, 0.0}, 0, footprintPadrao(), prm());
  EXPECT_TRUE(r.valido) << r.motivo;
  ParamsCaminho p = prm(); p.folga_fim = 0.0;
  EXPECT_FALSE(avaliarCaminhoOriginal(cm, reta(0, 0, 3.0, 0), Pose2{0, 0, 0.0}, 0, footprintPadrao(), p).valido);
}

TEST(CaminhoOriginal, OrientacaoPelaTangente)
{
  // vao vertical de 0,62 m (x de -0,31 a 0,31) entre y=-0,5 e 0,5, caminho subindo em x=0.
  // Com o yaw 0 gravado pelo NavFn o robo (0,59 em x) caberia; andando na tangente (90 graus) a largura
  // de 0,65 m fica em x e nao cabe. A checagem tem de usar a tangente -> reprova.
  auto cm = mapa();
  caixa(cm, 0.31, 1.0, -0.5, 0.5); caixa(cm, -1.0, -0.31, -0.5, 0.5);
  std::vector<Pose2> pts;
  for (int i = 0; i <= 80; ++i) {pts.push_back({0.0, -2.0 + 0.05 * i, 0.0});}
  auto y = yawTangente(pts, M_PI / 2);
  EXPECT_NEAR(y[10], M_PI / 2, 1e-6);
  EXPECT_FALSE(avaliarCaminhoOriginal(cm, pts, Pose2{0, -2.0, 0.0}, M_PI / 2, footprintPadrao(), prm()).valido);
  bool ras = false;
  // com yaw 0 e o centro em x=-0,08 o footprint (x de -0,295 a 0,295) cabe: so a tangente reprova
  EXPECT_FALSE(poseBloqueada(cm, {-0.08, 0.0, 0.0}, footprintPadrao(), prm(), ras));
}

TEST(CaminhoOriginal, DesconhecidoNaoBloqueia)
{
  auto cm = mapa();
  caixa(cm, 1.0, 2.0, -1, 1, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_TRUE(avaliarCaminhoOriginal(cm, reta(0, 0, 3, 0), Pose2{0, 0, 0.0}, 0, footprintPadrao(), prm()).valido);
}

TEST(CaminhoOriginal, ForaDoMapaBloqueia)
{
  auto cm = mapa();
  EXPECT_FALSE(avaliarCaminhoOriginal(cm, reta(0, 0, 5.5, 0), Pose2{0, 0, 0.0}, 0, footprintPadrao(), prm()).valido);
}

TEST(CaminhoOriginal, CaminhoVazio)
{
  auto cm = mapa();
  EXPECT_FALSE(avaliarCaminhoOriginal(cm, {}, Pose2{0, 0, 0.0}, 0, footprintPadrao(), prm()).valido);
}

TEST(CaminhoOriginal, FootprintDaStringIgualAoPadrao)
{
  auto fp = footprintDeString("[[0.365, 0.315], [0.365, -0.315], [-0.205, -0.315], [-0.205, 0.315]]", 0.01);
  auto pd = footprintPadrao();
  ASSERT_EQ(fp.size(), pd.size());
  for (size_t i = 0; i < fp.size(); ++i) {
    EXPECT_NEAR(fp[i].first, pd[i].first, 1e-9); EXPECT_NEAR(fp[i].second, pd[i].second, 1e-9);
  }
}

TEST(CaminhoOriginal, ContatoAtualNaoContaMasObstaculoNovoSim)
{
  // robo espremido: celulas letais sob o proprio footprint (ruido/zona cega). Sozinhas nao reprovam;
  // uma caixa nova a 1,5 m reprova.
  auto cm = mapa();
  caixa(cm, -0.1, 0.1, 0.28, 0.36); caixa(cm, -0.1, 0.1, -0.36, -0.28);
  EXPECT_TRUE(avaliarCaminhoOriginal(cm, reta(0, 0, 3, 0), Pose2{0, 0, 0}, 0, footprintPadrao(), prm()).valido);
  caixa(cm, 1.5, 1.6, -0.05, 0.05);
  EXPECT_FALSE(avaliarCaminhoOriginal(cm, reta(0, 0, 3, 0), Pose2{0, 0, 0}, 0, footprintPadrao(), prm()).valido);
}

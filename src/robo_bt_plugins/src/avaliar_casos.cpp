// CAMINHO_ORIGINAL (11/11/2026): avalia casos exportados do bag pelo nucleo C++ (sem ROS).
// Uso: avaliar_casos_caminho <pasta_casos>   (cada caso: NNN.txt + NNN.bin)
// Saida por linha: nome valido indice rasantes conferidas
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <vector>
#include "robo_bt_plugins/caminho_original.hpp"
using namespace robo_bt_plugins;
int main(int argc, char ** argv)
{
  if (argc < 2) {std::cerr << "uso: avaliar_casos_caminho <pasta>\n"; return 2;}
  std::set<std::string> nomes;
  for (auto & e : std::filesystem::directory_iterator(argv[1])) {
    if (e.path().extension() == ".txt") {nomes.insert(e.path().stem().string());}
  }
  for (const auto & n : nomes) {
    std::ifstream f(std::string(argv[1]) + "/" + n + ".txt");
    unsigned int sx, sy; double res, ox, oy, rx, ry, ryaw, yg; size_t np;
    f >> sx >> sy >> res >> ox >> oy >> rx >> ry >> ryaw >> yg >> np;
    std::vector<Pose2> pts(np);
    for (auto & p : pts) {f >> p.x >> p.y >> p.yaw;}
    ParamsCaminho prm;
    f >> prm.folga_inicio >> prm.folga_fim >> prm.tolerancia_lateral;
    nav2_costmap_2d::Costmap2D cm(sx, sy, res, ox, oy);
    std::ifstream b(std::string(argv[1]) + "/" + n + ".bin", std::ios::binary);
    b.read(reinterpret_cast<char *>(cm.getCharMap()), static_cast<std::streamsize>(sx) * sy);
    auto r = avaliarCaminhoOriginal(cm, pts, Pose2{rx, ry, ryaw}, yg, footprintPadrao(), prm);
    std::cout << n << " " << r.valido << " " << r.indice << " " << r.poses_rasantes << " " << r.poses_conferidas << "\n";
  }
  return 0;
}

// CAMINHO_ORIGINAL (11/11/2026) - Patch 01.
// Quando o SmoothPath reprova o caminho suavizado por colisao, a arvore entregava o caminho ORIGINAL do
// planner ao FollowPath sem conferir nada (ORIGINAL_SEM_GATE). Este nucleo confere o trecho restante do
// original e so o reprova quando ha BLOQUEIO de fato:
//   - footprint (com a orientacao da tangente; o NavFn grava yaw 0) sobre celula LETHAL; e
//   - nenhum deslocamento lateral de ate `tolerancia_lateral` livra o footprint nessa pose.
// Rasante de 2-8 cm (grade de 5 cm, parede do corredor) NAO reprova: no bag 2nav 23 de 30 caminhos
// "em colisao" pelo criterio do Nav2 se livravam com 2-8 cm. Ignora o trecho onde o robo ja esta
// (`folga_inicio`) e o fim do caminho (`folga_fim`), que e a zona do goal pedido pelo operador.
#pragma once
#include <string>
#include <utility>
#include <vector>
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace robo_bt_plugins
{
struct Pose2 { double x, y, yaw; };
using Footprint = std::vector<std::pair<double, double>>;

struct ParamsCaminho
{
  double folga_inicio = 0.30;        // m de caminho a partir do robo que nao sao conferidos
  double folga_fim = 0.60;           // m finais (zona do goal: o robo gira para o yaw do goal; ~1 comprimento)
  double tolerancia_lateral = 0.10;  // m de deslocamento lateral procurado para livrar o footprint
  double passo_lateral = 0.02;       // m
  bool desconhecido_livre = true;    // allow_unknown: true no planner
  double margem_footprint_atual = 0.05;  // m; celulas sob o footprint atual (+margem) nao contam
};

struct ResultadoCaminho
{
  bool valido = true;
  int indice = -1;               // pose bloqueada
  double dist_robo = 0.0;        // m de caminho do robo ate a pose bloqueada
  int poses_rasantes = 0;        // poses com footprint em LETHAL que um deslocamento lateral livra
  int poses_conferidas = 0;
  std::string motivo = "ok";
};

// Footprint padrao do robo: nav2_params.yaml footprint + footprint_padding 0.01
Footprint footprintPadrao();
Footprint footprintDeString(const std::string & s, double padding);
std::vector<double> yawTangente(const std::vector<Pose2> & pts, double yaw_final);
size_t indiceMaisProximo(const std::vector<Pose2> & pts, double rx, double ry);
// true se alguma celula das arestas do footprint e LETHAL ou se um vertice cai fora do mapa
// Poligono (mundo) do footprint numa pose, expandido de `margem` m.
std::vector<std::pair<double, double>> poligonoFootprint(const Footprint & fp, const Pose2 & p, double margem);
bool dentroPoligono(const std::vector<std::pair<double, double>> & poli, double x, double y);
// `ignorar`: celulas cujo centro cai neste poligono nao contam (o footprint ATUAL do robo: ruido, zona cega
// do LiDAR ou contato que o Collision Monitor/destravar ja tratam). Vazio = nada ignorado.
bool footprintLetal(const nav2_costmap_2d::Costmap2D & cm, double x, double y, double th,
  const Footprint & fp, bool desconhecido_livre,
  const std::vector<std::pair<double, double>> & ignorar = {});
bool poseBloqueada(const nav2_costmap_2d::Costmap2D & cm, const Pose2 & p, const Footprint & fp,
  const ParamsCaminho & prm, bool & rasante,
  const std::vector<std::pair<double, double>> & ignorar = {});
ResultadoCaminho avaliarCaminhoOriginal(const nav2_costmap_2d::Costmap2D & cm,
  const std::vector<Pose2> & pts, const Pose2 & robo, double yaw_goal, const Footprint & fp,
  const ParamsCaminho & prm);
}  // namespace robo_bt_plugins

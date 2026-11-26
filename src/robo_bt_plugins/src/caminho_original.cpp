// CAMINHO_ORIGINAL (11/11/2026) - ver include/robo_bt_plugins/caminho_original.hpp
#include "robo_bt_plugins/caminho_original.hpp"
#include <cmath>
#include <limits>
#include <sstream>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/line_iterator.hpp"

namespace robo_bt_plugins
{
Footprint footprintPadrao()
{
  return {{0.375, 0.325}, {0.375, -0.325}, {-0.215, -0.325}, {-0.215, 0.325}};
}

Footprint footprintDeString(const std::string & s, double padding)
{
  // formato do nav2: '[[x, y], [x, y], ...]'
  std::vector<double> v;
  std::string num;
  for (char c : s) {
    if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == 'e' || c == 'E' || c == '+') {
      num += c;
    } else if (!num.empty()) {
      v.push_back(std::stod(num));
      num.clear();
    }
  }
  if (!num.empty()) {v.push_back(std::stod(num));}
  Footprint fp;
  if (v.size() < 6 || v.size() % 2) {return fp;}
  for (size_t i = 0; i < v.size(); i += 2) {
    // padding como no nav2_costmap_2d::padFootprint: soma o sinal de cada coordenada
    double x = v[i], y = v[i + 1];
    x += (x > 0 ? padding : (x < 0 ? -padding : 0.0));
    y += (y > 0 ? padding : (y < 0 ? -padding : 0.0));
    fp.emplace_back(x, y);
  }
  return fp;
}

std::vector<double> yawTangente(const std::vector<Pose2> & pts, double yaw_final)
{
  const size_t n = pts.size();
  std::vector<double> out(n, 0.0);
  if (n == 0) {return out;}
  if (n == 1) {out[0] = yaw_final; return out;}
  for (size_t i = 0; i < n; ++i) {
    const size_t j0 = (i < n - 1) ? i : i - 1, j1 = (i < n - 1) ? i + 1 : i;
    out[i] = std::atan2(pts[j1].y - pts[j0].y, pts[j1].x - pts[j0].x);
  }
  out[n - 1] = yaw_final;
  return out;
}

size_t indiceMaisProximo(const std::vector<Pose2> & pts, double rx, double ry)
{
  size_t bi = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < pts.size(); ++i) {
    const double d = std::hypot(pts[i].x - rx, pts[i].y - ry);
    if (d < best) {best = d; bi = i;}
  }
  return bi;
}

std::vector<std::pair<double, double>> poligonoFootprint(const Footprint & fp, const Pose2 & p, double margem)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  std::vector<std::pair<double, double>> out;
  for (const auto & v : fp) {
    const double a = v.first + (v.first > 0 ? margem : (v.first < 0 ? -margem : 0.0));
    const double b = v.second + (v.second > 0 ? margem : (v.second < 0 ? -margem : 0.0));
    out.emplace_back(p.x + c * a - s * b, p.y + s * a + c * b);
  }
  return out;
}

bool dentroPoligono(const std::vector<std::pair<double, double>> & poli, double x, double y)
{
  bool dentro = false;
  for (size_t i = 0, j = poli.size() - 1; i < poli.size(); j = i++) {
    const auto & a = poli[i];
    const auto & b = poli[j];
    if (((a.second > y) != (b.second > y)) &&
      (x < (b.first - a.first) * (y - a.second) / (b.second - a.second) + a.first))
    {
      dentro = !dentro;
    }
  }
  return dentro;
}

bool footprintLetal(const nav2_costmap_2d::Costmap2D & cm, double x, double y, double th,
  const Footprint & fp, bool desconhecido_livre, const std::vector<std::pair<double, double>> & ignorar)
{
  (void)desconhecido_livre;  // so LETHAL conta; 255 nunca e LETHAL (sem o mascaramento do footprintCost)
  const double c = std::cos(th), s = std::sin(th);
  std::vector<std::pair<unsigned int, unsigned int>> vs;
  for (const auto & v : fp) {
    unsigned int mx, my;
    if (!cm.worldToMap(x + c * v.first - s * v.second, y + s * v.first + c * v.second, mx, my)) {
      return true;  // como o Nav2: vertice fora do mapa = LETHAL
    }
    vs.emplace_back(mx, my);
  }
  for (size_t i = 0; i < vs.size(); ++i) {
    const auto & a = vs[i];
    const auto & b = vs[(i + 1) % vs.size()];
    for (nav2_util::LineIterator it(a.first, a.second, b.first, b.second); it.isValid(); it.advance()) {
      if (cm.getCost(it.getX(), it.getY()) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        if (!ignorar.empty()) {
          double wx, wy;
          cm.mapToWorld(it.getX(), it.getY(), wx, wy);
          if (dentroPoligono(ignorar, wx, wy)) {continue;}
        }
        return true;
      }
    }
  }
  return false;
}

bool poseBloqueada(const nav2_costmap_2d::Costmap2D & cm, const Pose2 & p, const Footprint & fp,
  const ParamsCaminho & prm, bool & rasante, const std::vector<std::pair<double, double>> & ignorar)
{
  rasante = false;
  if (!footprintLetal(cm, p.x, p.y, p.yaw, fp, prm.desconhecido_livre, ignorar)) {return false;}
  const double nx = -std::sin(p.yaw), ny = std::cos(p.yaw);
  const int n = static_cast<int>(std::floor(prm.tolerancia_lateral / prm.passo_lateral + 1e-9));
  for (int k = 1; k <= n; ++k) {
    for (int sg : {1, -1}) {
      const double d = sg * k * prm.passo_lateral;
      if (!footprintLetal(cm, p.x + d * nx, p.y + d * ny, p.yaw, fp, prm.desconhecido_livre, ignorar)) {
        rasante = true;
        return false;
      }
    }
  }
  return true;
}

ResultadoCaminho avaliarCaminhoOriginal(const nav2_costmap_2d::Costmap2D & cm,
  const std::vector<Pose2> & pts, const Pose2 & robo, double yaw_goal, const Footprint & fp,
  const ParamsCaminho & prm)
{
  const double rx = robo.x, ry = robo.y;
  const auto atual = poligonoFootprint(fp, robo, prm.margem_footprint_atual);
  ResultadoCaminho r;
  if (pts.empty()) {r.valido = false; r.motivo = "caminho vazio"; return r;}
  if (fp.size() < 3) {r.valido = false; r.motivo = "footprint invalido"; return r;}
  const size_t i0 = indiceMaisProximo(pts, rx, ry);
  const auto yaws = yawTangente(pts, yaw_goal);
  // comprimento ate o fim, para a folga do goal
  std::vector<double> ate_fim(pts.size(), 0.0);
  for (size_t i = pts.size() - 1; i-- > 0; ) {
    ate_fim[i] = ate_fim[i + 1] + std::hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
  }
  double s = 0.0;
  for (size_t i = i0; i < pts.size(); ++i) {
    if (i > i0) {s += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);}
    if (s < prm.folga_inicio || ate_fim[i] < prm.folga_fim) {continue;}
    ++r.poses_conferidas;
    bool rasante = false;
    const Pose2 p{pts[i].x, pts[i].y, yaws[i]};
    if (poseBloqueada(cm, p, fp, prm, rasante, atual)) {
      r.valido = false;
      r.indice = static_cast<int>(i);
      r.dist_robo = s;
      std::ostringstream o;
      o.precision(2);
      o << std::fixed << "footprint bloqueado em (" << p.x << ", " << p.y << ") a " << s
        << " m do robo; nenhum deslocamento lateral ate " << prm.tolerancia_lateral << " m livra";
      r.motivo = o.str();
      return r;
    }
    if (rasante) {++r.poses_rasantes;}
  }
  return r;
}
}  // namespace robo_bt_plugins

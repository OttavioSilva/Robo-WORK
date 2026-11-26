#include "m2m2_sdk.hpp"

#include <rpos/features/location_provider/map.h>
#include <rpos/robot_platforms/objects/composite_map_reader.h>
#include <rpos/robot_platforms/objects/composite_map_writer.h>
#include <rpos/robot_platforms/slamware_core_platform.h>

#include <stdexcept>

using rpos::robot_platforms::SlamwareCorePlatform;
namespace lp = rpos::features::location_provider;
namespace obj = rpos::robot_platforms::objects;

namespace m2m2 {

struct Dispositivo::Impl { std::unique_ptr<SlamwareCorePlatform> p; };

Dispositivo::Dispositivo() : impl_(new Impl) {}
Dispositivo::~Dispositivo() = default;

#define SDK_TRY try {
#define SDK_CATCH } catch (const std::exception& e) { throw std::runtime_error(e.what()); }

static SlamwareCorePlatform& plat(const std::unique_ptr<SlamwareCorePlatform>& p) {
  if (!p) throw std::runtime_error("desconectado");
  return *p;
}

std::string Dispositivo::conectar(const std::string& host, int porta) {
  SDK_TRY
    impl_->p.reset(new SlamwareCorePlatform(SlamwareCorePlatform::connect(host, porta)));
    return impl_->p->getDeviceInfo().softwareVersion();
  SDK_CATCH
}
bool Dispositivo::conectado() const { return bool(impl_->p); }
void Dispositivo::desconectar() { impl_->p.reset(); }

Pose Dispositivo::pose() {
  SDK_TRY auto q = plat(impl_->p).getPose(); return Pose{q.x(), q.y(), q.yaw()}; SDK_CATCH
}

std::vector<Ponto> Dispositivo::scan() {
  SDK_TRY
    auto s = plat(impl_->p).getLaserScan();
    std::vector<Ponto> out;
    out.reserve(s.getLaserPoints().size());
    for (const auto& lp : s.getLaserPoints()) out.push_back(Ponto{lp.distance(), lp.angle(), lp.valid()});
    return out;
  SDK_CATCH
}

bool Dispositivo::mapa(Mapa& out) {
  SDK_TRY
    auto& p = plat(impl_->p);
    auto area = p.getKnownArea(lp::MapTypeBitmap8Bit, lp::EXPLORERMAP);
    if (area.width() <= 0 || area.height() <= 0) return false;
    auto m = p.getMap(lp::MapTypeBitmap8Bit, area, lp::EXPLORERMAP);
    out.origem_x = m.getMapArea().x();
    out.origem_y = m.getMapArea().y();
    out.resolucao = m.getMapResolution().x();
    out.largura = m.getMapDimension().x();
    out.altura = m.getMapDimension().y();
    out.dados = m.getMapData();
    return true;
  SDK_CATCH
}

bool Dispositivo::limpar_mapa() { SDK_TRY return plat(impl_->p).clearMap(); SDK_CATCH }
bool Dispositivo::mapeando(bool ligar) { SDK_TRY return plat(impl_->p).setMapUpdate(ligar); SDK_CATCH }
int Dispositivo::qualidade_localizacao() { SDK_TRY return plat(impl_->p).getLocalizationQuality(); SDK_CATCH }

bool Dispositivo::definir_pose(const Pose& q) {
  SDK_TRY return plat(impl_->p).setPose(rpos::core::Pose(rpos::core::Location(q.x, q.y, 0), rpos::core::Rotation(q.yaw, 0, 0))); SDK_CATCH
}

bool Dispositivo::salvar(const std::string& arquivo, std::string& erro) {
  SDK_TRY
    auto cm = plat(impl_->p).getCompositeMap();
    obj::CompositeMapWriter w;
    return w.saveFile(erro, arquivo, cm);
  SDK_CATCH
}

bool Dispositivo::carregar(const std::string& arquivo, std::string& erro) {
  SDK_TRY
    auto& p = plat(impl_->p);
    obj::CompositeMapReader r;
    auto cm = r.loadFile(erro, arquivo);
    if (!cm) return false;
    // mantem a pose atual; depois reposicionar com ~/definir_pose se preciso
    return p.setCompositeMap(*cm, p.getPose());
  SDK_CATCH
}

}  // namespace m2m2

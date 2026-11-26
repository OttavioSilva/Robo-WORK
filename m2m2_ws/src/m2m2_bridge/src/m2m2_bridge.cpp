// Ponte ROS 2 <-> Slamtec Mapper M2M2 (Slamware SDK 5.1.1, via m2m2_sdk.cpp).
// Publica: scan_raw (frame laser_raw), map (OccupancyGrid), m2m2/pose,
//          m2m2/qualidade, TF map->odom.
// Servicos: ~/limpar_mapa, ~/salvar_mapa, ~/carregar_mapa (Trigger),
//           ~/mapeando (SetBool: false = so localiza).
// Topico initialpose (PoseWithCovarianceStamped, pose do base_link no map)
// reposiciona o M2M2, como no AMCL/RViz "2D Pose Estimate".
// A pose do M2M2 e a do PROPRIO sensor; map->odom usa base_link->laser (TF
// estatico) e odom->base_link (odometria do robo).
#include "m2m2_sdk.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;

class M2M2Bridge : public rclcpp::Node {
public:
  M2M2Bridge() : Node("m2m2_bridge") {
    host_ = declare_parameter("host", std::string("192.168.11.1"));
    port_ = declare_parameter("port", 1445);
    laser_frame_ = declare_parameter("laser_frame", std::string("laser_raw"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    scan_topic_ = declare_parameter("scan_topic", std::string("scan_raw"));
    publish_tf_ = declare_parameter("publish_tf", true);
    publish_map_ = declare_parameter("publish_map", true);
    scan_hz_ = declare_parameter("scan_hz", 10.0);
    pose_hz_ = declare_parameter("pose_hz", 20.0);
    map_period_ = declare_parameter("map_period_s", 3.0);
    range_min_ = declare_parameter("range_min", 0.10);
    range_max_ = declare_parameter("range_max", 40.0);
    angle_sign_ = declare_parameter("angle_sign", 1.0);   // -1 se o scan sair espelhado
    angle_offset_ = declare_parameter("angle_offset", 0.0);  // rad, se a frente do scan girar
    range_offset_ = declare_parameter("range_offset", 0.0);  // RANGE_OFFSET (25/11): m descontados de cada leitura (M2M2 media +0,31..0,41 m)
    range_scale_ = declare_parameter("range_scale", 1.0);
    feixes_ = declare_parameter("feixes", 920);   // FEIXES_FIXOS (26/11): grade fixa; o slam_toolbox descarta scan com tamanho diferente do 1o (0 = como antes)    // RANGE_OFFSET: dist = (bruta - offset) * escala
    clear_on_start_ = declare_parameter("limpar_ao_iniciar", false);
    mapping_on_start_ = declare_parameter("mapeando", true);
    map_file_ = declare_parameter("arquivo_mapa", std::string(""));
    load_on_start_ = declare_parameter("carregar_ao_iniciar", false);
    tf_future_ = declare_parameter("tf_future_s", 0.1);
    limiar_ocupado_ = declare_parameter("limiar_ocupado", 128);

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS().keep_last(5));
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local().reliable());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("m2m2/pose", 10);
    qual_pub_ = create_publisher<std_msgs::msg::Int32>("m2m2/qualidade", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    srv_clear_ = create_service<Trigger>("~/limpar_mapa",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          r->success = sdk([&] { return dev_.limpar_mapa(); }, r->message); });
    srv_map_ = create_service<SetBool>("~/mapeando",
        [this](SetBool::Request::SharedPtr q, SetBool::Response::SharedPtr r) {
          r->success = sdk([&] { return dev_.mapeando(q->data); }, r->message);
          if (r->success) r->message = q->data ? "mapeando" : "so localizando"; });
    srv_save_ = create_service<Trigger>("~/salvar_mapa",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          map_file_ = get_parameter("arquivo_mapa").as_string();
          r->success = sdk([&] { return dev_.salvar(map_file_, r->message); }, r->message);
          if (r->success) r->message = "salvo em " + map_file_; });
    srv_load_ = create_service<Trigger>("~/carregar_mapa",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
          map_file_ = get_parameter("arquivo_mapa").as_string();
          r->success = sdk([&] { return dev_.carregar(map_file_, r->message); }, r->message);
          if (r->success) r->message = "carregado " + map_file_; });
    init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) { definir_pose(*m); });

    worker_ = std::thread([this] { loop(); });
    // TRAVA_MAPA (24/11): publicar o /map (centenas de KB) pode bloquear no DDS;
    // fica numa thread separada para scan/pose/TF nunca pararem.
    if (publish_map_) map_worker_ = std::thread([this] { loop_mapa(); });
  }

  ~M2M2Bridge() override {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    if (map_worker_.joinable()) map_worker_.detach();  // pode estar presa no DDS
  }

private:
  template <typename F> bool sdk(F f, std::string& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!dev_.conectado()) { msg = "M2M2 desconectado"; return false; }
    try { bool ok = f(); if (!ok && msg.empty()) msg = "falhou"; return ok; }
    catch (const std::exception& e) { msg = e.what(); RCLCPP_ERROR(get_logger(), "SDK: %s", e.what()); return false; }
  }

  // initialpose = base_link no map -> pose do sensor = base * (base->laser)
  void definir_pose(const geometry_msgs::msg::PoseWithCovarianceStamped& m) {
    tf2::Transform map_base, base_laser;
    tf2::fromMsg(m.pose.pose, map_base);
    try {
      tf2::fromMsg(tf_buffer_->lookupTransform(base_frame_, laser_frame_, tf2::TimePointZero).transform, base_laser);
    } catch (const tf2::TransformException& e) {
      RCLCPP_ERROR(get_logger(), "initialpose sem TF base->laser: %s", e.what()); return;
    }
    tf2::Transform ml = map_base * base_laser;
    m2m2::Pose p{ml.getOrigin().x(), ml.getOrigin().y(), tf2::getYaw(ml.getRotation())};
    std::string msg;
    bool ok = sdk([&] { return dev_.definir_pose(p); }, msg);
    RCLCPP_INFO(get_logger(), "initialpose sensor x=%.3f y=%.3f yaw=%.1f graus: %s", p.x, p.y, p.yaw * 180 / M_PI, ok ? "ok" : msg.c_str());
  }

  void inicializar() {
    std::string msg;
    if (clear_on_start_) RCLCPP_INFO(get_logger(), "limpar mapa: %s", sdk([&] { return dev_.limpar_mapa(); }, msg) ? "ok" : msg.c_str());
    if (load_on_start_) { msg.clear(); RCLCPP_INFO(get_logger(), "carregar %s: %s", map_file_.c_str(), sdk([&] { return dev_.carregar(map_file_, msg); }, msg) ? "ok" : msg.c_str()); }
    msg.clear();
    sdk([&] { return dev_.mapeando(mapping_on_start_); }, msg);
    RCLCPP_INFO(get_logger(), "mapeamento %s", mapping_on_start_ ? "LIGADO" : "DESLIGADO (so localiza)");
  }

  void loop() {
    auto t0 = std::chrono::steady_clock::now();
    auto next_scan = t0, next_pose = t0, next_q = t0;
    while (running_ && rclcpp::ok()) {
      if (!dev_.conectado()) {
        try {
          std::string fw;
          { std::lock_guard<std::mutex> lk(mtx_); fw = dev_.conectar(host_, port_); }
          RCLCPP_INFO(get_logger(), "M2M2 conectado %s:%d fw=%s", host_.c_str(), port_, fw.c_str());
          inicializar();
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Sem conexao com M2M2 %s: %s", host_.c_str(), e.what());
          std::this_thread::sleep_for(1s);
          continue;
        }
      }
      auto now = std::chrono::steady_clock::now();
      try {
        if (now >= next_pose) { publicar_pose(); next_pose = now + periodo(pose_hz_); }
        if (now >= next_scan) { publicar_scan(); next_scan = now + periodo(scan_hz_); }
        if (now >= next_q) { publicar_qualidade(); next_q = now + 1s; }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Falha de comunicacao, reconectando: %s", e.what());
        std::lock_guard<std::mutex> lk(mtx_);
        dev_.desconectar();
      }
      std::this_thread::sleep_for(2ms);
    }
  }

  void loop_mapa() {
    while (running_ && rclcpp::ok()) {
      auto t = std::chrono::steady_clock::now() + periodo(1.0 / map_period_);
      if (dev_.conectado()) {
        try { publicar_mapa(); }
        catch (const std::exception& e) { RCLCPP_WARN(get_logger(), "mapa: %s", e.what()); }
      }
      std::this_thread::sleep_until(t);
    }
  }

  static std::chrono::steady_clock::duration periodo(double hz) {
    return std::chrono::microseconds(int64_t(1e6 / std::max(hz, 0.01)));
  }

  void publicar_scan() {
    std::vector<m2m2::Ponto> pts;
    { std::lock_guard<std::mutex> lk(mtx_); pts = dev_.scan(); }
    if (pts.size() < 10) return;
    const long fx = get_parameter("feixes").as_int();   // FEIXES_FIXOS (26/11)
    const size_t n = fx > 0 ? size_t(fx) : pts.size();
    const double inc = 2.0 * M_PI / double(n);
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = laser_frame_;
    msg.angle_min = -M_PI;
    msg.angle_max = -M_PI + inc * double(n - 1);
    msg.angle_increment = inc;
    msg.range_min = range_min_;
    msg.range_max = range_max_;
    msg.scan_time = 1.0 / scan_hz_;
    msg.ranges.assign(n, std::numeric_limits<float>::infinity());
    const double r_off = get_parameter("range_offset").as_double();   // RANGE_OFFSET: ros2 param set muda na hora
    const double r_esc = get_parameter("range_scale").as_double();
    const double a_sig = get_parameter("angle_sign").as_double();   // ESPELHO_M2M2 (25/11): ao vivo
    for (const auto& p : pts) {  // re-amostra em grade uniforme [-pi, pi)
      const double dist = (p.dist - r_off) * r_esc;   // RANGE_OFFSET (25/11)
      if (!p.valido || dist < range_min_ || dist > range_max_) continue;
      double a = std::remainder(a_sig * p.ang + angle_offset_, 2.0 * M_PI);   // ESPELHO_M2M2
      long i = std::lround((a + M_PI) / inc);
      if (i >= long(n)) i -= long(n);
      if (i < 0) i = 0;
      if (dist < msg.ranges[i]) msg.ranges[i] = dist;   // RANGE_OFFSET
    }
    scan_pub_->publish(msg);
  }

  void publicar_qualidade() {
    int q;
    { std::lock_guard<std::mutex> lk(mtx_); q = dev_.qualidade_localizacao(); }
    std_msgs::msg::Int32 m; m.data = q; qual_pub_->publish(m);
  }

  void publicar_pose() {
    m2m2::Pose p;
    { std::lock_guard<std::mutex> lk(mtx_); p = dev_.pose(); }
    const auto stamp = now();
    tf2::Quaternion q; q.setRPY(0, 0, p.yaw);
    tf2::Transform map_laser(q, tf2::Vector3(p.x, p.y, 0));

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = map_frame_;
    tf2::toMsg(map_laser, ps.pose);
    pose_pub_->publish(ps);
    if (!publish_tf_) return;

    tf2::Transform t_bl, t_ob;
    try {
      tf2::fromMsg(tf_buffer_->lookupTransform(base_frame_, laser_frame_, tf2::TimePointZero).transform, t_bl);
      tf2::fromMsg(tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero).transform, t_ob);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Sem TF para map->odom: %s", e.what());
      return;
    }
    // map->odom = map->laser * laser->base * base->odom
    tf2::Transform map_odom = map_laser * t_bl.inverse() * t_ob.inverse();
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = stamp + rclcpp::Duration::from_seconds(tf_future_);
    out.header.frame_id = map_frame_;
    out.child_frame_id = odom_frame_;
    out.transform = tf2::toMsg(map_odom);
    tf_pub_->sendTransform(out);
  }

  void publicar_mapa() {
    m2m2::Mapa m;
    { std::lock_guard<std::mutex> lk(mtx_); if (!dev_.mapa(m)) return; }
    nav_msgs::msg::OccupancyGrid g;
    g.header.stamp = now();
    g.header.frame_id = map_frame_;
    g.info.resolution = m.resolucao;
    g.info.width = m.largura;
    g.info.height = m.altura;
    g.info.origin.position.x = m.origem_x;
    g.info.origin.position.y = m.origem_y;
    g.info.origin.orientation.w = 1.0;
    g.data.assign(size_t(m.largura) * m.altura, -1);
    // MAPA_CODIFICACAO (24/11, medido): 0 = desconhecido; 127 = livre (chao); >=128 = ocupado.
    // Medido: o M2M2 deixa as paredes da arena em 127 (livre) -- so a sala distante vira >=128.
    for (size_t i = 0; i < g.data.size() && i < m.dados.size(); ++i) {
      const uint8_t v = m.dados[i];
      g.data[i] = v == 0 ? -1 : (v >= limiar_ocupado_ ? 100 : 0);
    }
    map_pub_->publish(g);
  }

  std::string host_, laser_frame_, base_frame_, odom_frame_, map_frame_, scan_topic_, map_file_;
  int port_, limiar_ocupado_;
  bool publish_tf_, publish_map_, clear_on_start_, mapping_on_start_, load_on_start_;
  double scan_hz_, pose_hz_, map_period_, range_min_, range_max_, angle_sign_, angle_offset_, tf_future_, range_offset_, range_scale_;   // RANGE_OFFSET
  long feixes_ = 920;   // FEIXES_FIXOS

  std::mutex mtx_;
  m2m2::Dispositivo dev_;
  std::atomic<bool> running_{true};
  std::thread worker_, map_worker_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr qual_pub_;
  rclcpp::Service<Trigger>::SharedPtr srv_clear_, srv_save_, srv_load_;
  rclcpp::Service<SetBool>::SharedPtr srv_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<M2M2Bridge>());
  rclcpp::shutdown();
}

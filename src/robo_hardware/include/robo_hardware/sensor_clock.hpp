#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace robo_hardware {
inline bool valid_command_stamp(int64_t source_ns, int64_t now_ns, double limit_s) {
  return source_ns > 0 && now_ns >= source_ns && std::isfinite(limit_s) &&
    limit_s > 0 && (now_ns-source_ns)*1e-9 <= limit_s;
}
// Intervalo causal: host_send <= instante da leitura do micros no probe <= host_recv.
// Sem simetria presumida da serial. ppm e lag precisam de limites comprovados.
struct SensorClock {
  double sent = 0, received = 0;
  uint32_t device = 0;
  bool anchored = false;
  void reset() { anchored = false; }
  bool anchor(double tx, double rx, uint32_t us) {
    anchored = std::isfinite(tx) && std::isfinite(rx) && rx >= tx && rx-tx <= 0.3;
    sent = tx; received = rx; device = us;
    return anchored;
  }
  bool oldest(double host_now, uint32_t begin, uint32_t end, uint32_t queued,
    double drift_ppm, double sensor_lag, double &age_upper, double &uncertainty) const
  {
    if (!anchored || drift_ppm < 0 || drift_ppm > 100000 || sensor_lag < 0 ||
      !std::isfinite(drift_ppm) || !std::isfinite(sensor_lag) ||
      host_now < received || host_now-received > 1.0) return false;
    const int32_t span = static_cast<int32_t>(end-begin);
    const int32_t queue = static_cast<int32_t>(queued-end);
    const double delta = static_cast<int32_t>(begin-device)*1e-6;
    if (span < 0 || span > 1000000 || queue < 0 || queue > 1000000 ||
      std::abs(delta) > 2.0) return false;
    // Para duracao d do dispositivo, duracao host esta em [d/(1+p),d/(1-p)].
    const double p = drift_ppm*1e-6;
    const double d1 = delta/(1+p), d2 = delta/(1-p);
    const double lower = sent+std::min(d1,d2)-sensor_lag;
    const double upper = received+std::max(d1,d2);
    const double queued_delta = static_cast<int32_t>(queued-device)*1e-6;
    const double earliest_queue = sent+std::min(queued_delta/(1+p),queued_delta/(1-p));
    if (earliest_queue > host_now || lower > host_now || upper < lower) return false;
    age_upper = host_now-lower;
    uncertainty = upper-lower;
    return std::isfinite(age_upper) && age_upper >= 0 && age_upper <= 0.3;
  }
};
}

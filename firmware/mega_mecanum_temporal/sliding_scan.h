#pragma once
#include <stdint.h>
// Janela deslizante de RECEPCAO no MCU. Nao cria timestamp optico.
struct SlidingScan180 {
  uint16_t ranges[180] = {};
  uint32_t stamps[180] = {};
  void sample(uint16_t bin, uint16_t mm, uint32_t packet_us) {
    if (bin >= 180) return;
    if (stamps[bin] != packet_us) { ranges[bin] = 0; stamps[bin] = packet_us; }
    // Invalido nao transforma falta de eco em livre e substitui leitura antiga.
    if (mm && (!ranges[bin] || mm < ranges[bin])) ranges[bin] = mm;
  }
  void expire(uint32_t now_us) {
    for (uint16_t i=0; i<180; ++i) {
      if (stamps[i] && uint32_t(now_us-stamps[i]) > 250000UL) {
        stamps[i]=0; ranges[i]=0;
      }
    }
  }
  uint16_t value(uint16_t bin, uint32_t now_us) const {
    return stamps[bin] && uint32_t(now_us-stamps[bin]) <= 250000UL ? ranges[bin] : 0;
  }
  uint32_t oldest(uint32_t now_us) const {
    uint32_t oldest_age = 0;
    for (uint16_t i=0; i<180; ++i) {
      if (value(i,now_us) && uint32_t(now_us-stamps[i]) > oldest_age) oldest_age=now_us-stamps[i];
    }
    return now_us-oldest_age;
  }
};

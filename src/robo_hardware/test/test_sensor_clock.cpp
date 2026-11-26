#include <cstdlib>
#include <iostream>
#include <limits>
#include "robo_hardware/sensor_clock.hpp"
#include "../../../firmware/mega_mecanum_temporal/sliding_scan.h"
#define CHECK(x) do { ++checks; if (!(x)) {std::cerr << __LINE__ << ": " #x "\n"; return 1;} } while (0)
int main() {
  int checks = 0;
  CHECK(robo_hardware::valid_command_stamp(1000000000,1100000000,.25));
  CHECK(!robo_hardware::valid_command_stamp(1000000000,1300000000,.25));
  CHECK(!robo_hardware::valid_command_stamp(1200000000,1100000000,.25));
  CHECK(!robo_hardware::valid_command_stamp(0,1100000000,.25));
  CHECK(!robo_hardware::valid_command_stamp(-1,1100000000,.25));
  CHECK(!robo_hardware::valid_command_stamp(1000000000,1100000000,-1));
  SlidingScan180 scan;
  CHECK(scan.value(0,1000)==0);
  scan.sample(0,1000,10000); scan.sample(0,900,10000);
  CHECK(scan.value(0,11000)==900);
  scan.sample(0,0,12000);
  CHECK(scan.value(0,13000)==0);
  scan.sample(0,800,14000); scan.sample(1,700,15000);
  CHECK(scan.oldest(20000)==14000);
  CHECK(scan.value(0,300000)==0);
  scan.expire(300000);
  CHECK(scan.value(0,14001)==0);  // depois de uma volta inteira do relogio nao ressuscita
  scan.sample(0,700,0xffffff00U);
  CHECK(scan.value(0,0x10U)==700);
  CHECK(scan.oldest(0x10U)==0xffffff00U);
  robo_hardware::SensorClock c;
  double age = 0, uncertainty = 0;
  CHECK(!c.oldest(10, 0, 0, 0, 100, 0, age, uncertainty));
  CHECK(c.anchor(10, 10.02, 1000000));
  CHECK(c.oldest(10.1, 950000, 980000, 990000, 1000, 0.01, age, uncertainty));
  CHECK(age >= 0.16 && age < 0.161);
  CHECK(uncertainty >= 0.03);
  CHECK(!c.oldest(10.1, 950000, 980000, 990000, -1, 0.01, age, uncertainty));
  CHECK(!c.oldest(10.1, 950000, 980000, 990000, 1000, -1, age, uncertainty));
  CHECK(!c.oldest(10.7, 950000, 980000, 990000, 1000, 0.01, age, uncertainty));
  CHECK(!c.oldest(11.2, 1100000, 1150000, 1190000, 1000, 0.01, age, uncertainty));
  CHECK(!c.oldest(10.1, 980000, 950000, 990000, 1000, 0.01, age, uncertainty));
  CHECK(!c.oldest(10.1, 950000, 990000, 980000, 1000, 0.01, age, uncertainty));
  CHECK(!c.oldest(10.1, 950000, 980000, 990000, std::numeric_limits<double>::quiet_NaN(), 0, age, uncertainty));
  CHECK(!c.anchor(10, 9, 0));
  CHECK(!c.anchor(10, 10.31, 0));
  CHECK(c.anchor(20, 20.01, 0xfffffff0U));
  CHECK(c.oldest(20.1, 0xffffff00U, 0x10U, 0x20U, 1000, 0, age, uncertainty));
  CHECK(age > 0.1);
  c.reset();
  CHECK(!c.oldest(20.1, 0xffffff00U, 0x10U, 0x20U, 1000, 0, age, uncertainty));
  std::cout << "sensor_clock: " << checks << " checks passed\n";
}

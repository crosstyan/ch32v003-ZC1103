//
// Created by Kurosu Chan on 2023/6/30.
//

#include "spot.h"
#include "map"

int main() {
  auto t0 = Track();
  auto m0 = std::map<int, float>{
      {0, 0},
      {50, 1.1},
      {100, 1.5},
      {200, 1.3},
      {300, 1.1},
      {400, 1.0},
      {500, 0.9},
      {600, 0.8},
      {700, 0.7},
      {800, 0.6},
      {900, 0.5},
      {1000, 0.4},
  };
  t0.color = 0b00000001;
  for (auto &[k, v] : m0) {
    t0.addSpeed(k, v);
  }
  uint8_t buffer0[128];
  auto sz = serd::toBytes(t0, buffer0);
  auto sz_calc = t0.sizeNeeded();
  printf("t0 size: %d, calculated: %d\n", sz, sz_calc);
  assert(sz == sz_calc);
  auto t0_ = Track::fromBytes(buffer0);
  if (t0_.has_value()){
    auto val = t0_.value();
    assert(val.color == t0.color);
    assert(val.getSpeeds().size() == t0.getSpeeds().size());
    for (auto const &[k, v] : val.getSpeeds()) {
      assert(t0.getSpeeds().at(k) == v);
    }
  } else {
    printf("t0_ is empty\n");
  }
  printf("Track serialized/deserialized successfully\n");

  auto m1 = std::map<int, float>{
      {0, 0.0},
      {50, 1.1},
      {100, 1.5},
      {200, 1.3},
      {300, 1.1},
      {400, 1.0}
  };
  auto m2 = std::map<int, float>{
      {0, 0.0},
      {50, 0.9},
      {100, 1.0},
      {200, 2.0},
      {300, 2.5},
      {400, 4.0}
  };
  auto t1  = Track(0);
  t1.color = 0b00000011;
  for (auto const &[k, v] : m1) {
    t1.addSpeed(k, v);
  }
  auto t2  = Track(1);
  t2.color = 0b00000010;
  for (auto const &[k, v] : m2) {
    t2.addSpeed(k, v);
  }
  auto scfg = SpotConfig{
      .circleLength   = 400,
      .lineLength     = 18,
      .total          = 400,
      .current        = -1,
      .updateInterval = 100,
  };
  uint8_t buffer_cfg[128];
  auto sz_cfg = serd::toBytes(scfg, buffer_cfg);
  auto sz_cfg_calc = scfg.sizeNeeded();
  printf("scfg size: %d, calculated: %d\n", sz_cfg, sz_cfg_calc);
  assert(sz_cfg == sz_cfg_calc);
  auto scfg_ = SpotConfig::fromBytes(buffer_cfg);
  if (scfg_.has_value()){
    auto val = scfg_.value();
    assert(val.circleLength == scfg.circleLength);
    assert(val.lineLength == scfg.lineLength);
    assert(val.total == scfg.total);
    assert(val.current == scfg.current);
    assert(val.updateInterval == scfg.updateInterval);
  } else {
    printf("scfg_ is empty\n");
  }
  printf("SpotConfig serialized/deserialized successfully\n");

  auto s = Spot(scfg);
  s.addTrack(std::move(t1));
  s.addTrack(std::move(t2));
  uint8_t buffer_spot[256];
  auto sz_spot = s.toBytes(buffer_spot);
  auto sz_spot_calc = s.sizeNeeded();
  printf("s size: %d, calculated: %d\n", sz_spot, sz_spot_calc);
  assert(sz_spot == sz_spot_calc);
  auto s_ = Spot(scfg);
  s_.fromBytes(buffer_spot);
  assert(s_.getTracks().size() == s.getTracks().size());
  auto t1_ = s_.getTracks().at(0).first;
  assert(t1_.color == t1.color);
  assert(t1_.getSpeeds().size() == t1.getSpeeds().size());
  for(auto const &[k, v] : t1_.getSpeeds()){
    assert(t1.getSpeeds().at(k) == v);
  }
  auto t2_ = s_.getTracks().at(1).first;
  assert(t2_.color == t2.color);
  assert(t2_.getSpeeds().size() == t2.getSpeeds().size());
  for(auto const &[k, v] : t2_.getSpeeds()){
    assert(t2.getSpeeds().at(k) == v);
  }
  printf("Spot serialized/deserialized successfully\n");

  return 0;
}

//
// Created by Kurosu Chan on 2023/6/28.
//

#ifndef SIMPLE_SPOT_H
#define SIMPLE_SPOT_H
#include "cnl_def.h"
#include <etl/unordered_map.h>
#include <etl/vector.h>
#include <etl/delegate.h>
#include "system_tick.h"

const auto MAX_SPEED_MAP_SIZE = 16;
const auto MAX_ENABLED_ID_SIZE = 16;
const auto MAX_TRACK_SIZE     = 3;

/**
 * @brief a function retrieve value by the number nearing the key. always move a unit up.
 *
 * @tparam T the type of value of map
 * @param keys a std::vector of int which should be sorted
 * @param m  a map whose key is int
 * @param val a value you want
 * @return T
 */
template <typename U, typename T>
T retrieveByVal(etl::ivector<U> const &keys, etl::iunordered_map<U, T> const &m, int val) {
  size_t idx = 0;
  // since keys is sorted we can get result easily
  for (auto key : keys) {
    if (val < key) {
      break;
    }
    idx += 1;
  }
  if (idx >= keys.size()) {
    idx = keys.size() - 1;
  }
  return m.at(keys[idx]);
}

enum class SpotState {
  STOP,
  START,
};

struct SpotConfig {
  fixed_16_16 circleLength;
  fixed_16_16 lineLength;
  uint16_t total;
  uint16_t current;
  /// in ms
  uint16_t updateInterval;
};

class Track {
public:
  /// only need 3 bits
  uint8_t color;

private:
  etl::vector<uint16_t, MAX_SPEED_MAP_SIZE> keys;
  etl::unordered_map<uint16_t, fixed_16_16, MAX_SPEED_MAP_SIZE> speeds;
  uint16_t maxKey;

public:
  Track() {
    color  = 0;
    maxKey = 0;
  }

  void addSpeed(uint16_t key, fixed_16_16 speed) {
    keys.push_back(key);
    etl::sort(keys.begin(), keys.end());
    speeds.insert(etl::make_pair(key, speed));
    if (key > maxKey) {
      maxKey = key;
    }
  }

  uint16_t getMaxKey() const {
    return maxKey;
  }

  fixed_16_16 getSpeed(uint16_t key) {
    return retrieveByVal<uint16_t, fixed_16_16>(keys, speeds, key);
  }
};

struct CalcState {
  uint64_t startTime;
  uint64_t lastIntegralTime;
  uint32_t lastIntegralRelativeTime;
  fixed_16_16 lastIntegralDistance;
  uint16_t maxDistance;
};

/**
 * @brief calculate the next state
 * @param lastState last state
 * @param now current timestamp in millis
 * @param track Track
 * @param spot SpotConfig
 * @return optional<CalcState> if the next state is valid, return the next state, otherwise return nullopt
 */
etl::optional<CalcState> nextState(const CalcState &lastState, uint64_t now, Track &track, const SpotConfig &spot) {
  /// deltaT in ms (millis)
  auto deltaT = static_cast<fixed_16_16>(static_cast<uint32_t>(now - lastState.lastIntegralTime));
  if (deltaT < 0) {
    return etl::nullopt;
  }

  //  int speed = getNearestSpeed(speeds, lastState.lastIntegralDistance);
  auto speed  = track.getSpeed(static_cast<uint16_t>(lastState.lastIntegralDistance));
  auto deltaL = speed * deltaT;
  auto d      = lastState.lastIntegralDistance + deltaL;

  if (d > (lastState.maxDistance + spot.lineLength)) {
    return etl::nullopt;
  }

  CalcState newState{
      .startTime                = lastState.startTime,
      .lastIntegralTime         = now,
      .lastIntegralRelativeTime = lastState.lastIntegralRelativeTime + static_cast<uint32_t>(deltaT),
      .lastIntegralDistance     = d,
      .maxDistance              = lastState.maxDistance};

  return newState;
}


etl::vector<uint16_t, MAX_ENABLED_ID_SIZE> calcEnabledId(const CalcState &state, const SpotConfig &spot) {
  auto headDist                       = state.lastIntegralDistance;
  auto head                           = headDist % spot.circleLength;
  etl::optional<decltype(head)> extra = etl::nullopt;
  decltype(spot.circleLength) tail;

  if (head < spot.lineLength) {
    auto extraVal = spot.lineLength - head;
    extra         = extraVal;
    tail          = spot.circleLength - extraVal;
  } else {
    tail = head - spot.lineLength;
  }

  auto spotDistance = spot.circleLength / spot.total;
  auto headSpotId   = static_cast<uint16_t>(cnl::floor(head / spotDistance));
  auto tailSpotId   = static_cast<uint16_t>(cnl::floor(tail / spotDistance));

  etl::vector<uint16_t, MAX_ENABLED_ID_SIZE> enabledIds;

  if (headDist > state.maxDistance) {
    // range(tailSpotId, spot.total)
    for (auto i = tailSpotId; i < spot.total; ++i) {
      enabledIds.push_back(i);
    }
  } else if (extra.has_value()) {
    // range(0, headSpotId)
    for (auto i = 0; i < headSpotId; ++i) {
      enabledIds.push_back(i);
    }
    if (headDist >= spot.circleLength) {
      // range(tailSpotId, spot.total)
      for (auto i = tailSpotId; i < spot.total; ++i) {
        enabledIds.push_back(i);
      }
    }
  } else {
    // range(tailSpotId, headSpotId)
    for (auto i = tailSpotId; i < headSpotId; ++i) {
      enabledIds.push_back(i);
    }
  }

  return enabledIds;
}

class Spot {
public:
  SpotState state;
  etl::delegate<void(uint8_t)> setColorCallback = [](uint8_t) {};

private:
  etl::vector<etl::pair<Track, CalcState>, MAX_TRACK_SIZE> tracks;
  SpotConfig config;

public:
  Spot(SpotConfig config) : config(config) {
    state     = SpotState::STOP;
  };

  /// use std::move to avoid copy
  void setSpotConfig(SpotConfig cfg) {
    this->config = cfg;
  }

  /// use std::move to avoid copy
  void addTrack(Track track) {
    auto calcState = CalcState{
        .startTime                = 0,
        .lastIntegralTime         = 0,
        .lastIntegralRelativeTime = 0,
        .lastIntegralDistance     = 0,
        .maxDistance              = track.getMaxKey(),
    };
    tracks.push_back(etl::make_pair(track, calcState));
  }

  void start(){
    state = SpotState::START;
    for (auto &track : tracks) {
      auto &[t, calc] = track;
      calc.startTime  = millis();
      calc.lastIntegralTime = calc.startTime;
      calc.lastIntegralRelativeTime = 0;
      calc.lastIntegralDistance = 0;
      calc.maxDistance = t.getMaxKey();
    }
  }

  decltype(tracks) &getTracks() {
    return tracks;
  }

  void clearTracks() {
    tracks.clear();
  }

  void update() {
    if (state != SpotState::START) {
      return;
    }
    bool isChanged = false;
    etl::vector<bool, MAX_TRACK_SIZE> isAllStop;

    for (auto &track : tracks) {
      auto &[t, calc] = track;
      auto now        = millis();
      auto newState   = nextState(calc, now, t, config);

      if (newState.has_value()) {
        auto newCalc    = newState.value();
        auto enabledIds = calcEnabledId(newCalc, config);

        if (etl::find(enabledIds.begin(), enabledIds.end(), config.current) != enabledIds.end()) {
          isChanged = true;
          setColorCallback(t.color);
        }

        calc = newCalc;
        isAllStop.push_back(false);
      } else {
        isAllStop.push_back(true);
      }
      // identity
      if (etl::all_of(isAllStop.begin(), isAllStop.end(), [](bool v) { return v; })) {
        state = SpotState::STOP;
      }
    }
  }
};

#endif // SIMPLE_SPOT_H

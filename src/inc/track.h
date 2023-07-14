//
// Created by Kurosu Chan on 2023/7/13.
//

#ifndef SIMPLE_TRACK_H
#define SIMPLE_TRACK_H

#include "cnl_def.h"
#include "error.h"
#include <etl/flat_map.h>
#include <etl/unordered_map.h>
#include <etl/map.h>
#include <map>
#include <etl/vector.h>
#include <etl/delegate.h>
#include <etl/expected.h>
#include "system_tick.h"
#include "def.h"
#include "current.h"

namespace RfMessage {

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
static T retrieveByVal(etl::ivector<U> const &keys, etl::ivector<etl::pair<U, T>> const &m, int val) {
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
  auto k  = keys[idx];
  auto pv = etl::find_if(m.begin(), m.end(), [k](etl::pair<U, T> const &p) { return p.first == k; });
  // should not enter this branch
  if (pv == m.end()) [[unlikely]] {
    return T();
  } else {
    return pv->second;
  }
}

const auto MAX_SPEED_MAP_SIZE  = 16;
const auto MAX_ENABLED_ID_SIZE = 16;
const auto MAX_TRACK_SIZE      = 2;

// could choose fixed_8_8 or fixed_16_16
using speed_type = fixed_8_8;

struct Track {
public:
  /// only need 3 bits
  uint8_t color;
  uint8_t id;

private:
  etl::vector<uint16_t, MAX_SPEED_MAP_SIZE> keys;
  etl::vector<etl::pair<uint16_t, speed_type>, MAX_SPEED_MAP_SIZE> speeds;
  uint16_t maxKey;

public:
  explicit Track() {
    id     = 0;
    color  = 0;
    maxKey = 0;
  };
  explicit Track(uint8_t identifier) : id(identifier) {
    color  = 0;
    maxKey = 0;
  }

  /**
   * @brief add a speed to the track
   * @param key the distance
   * @param speed the speed at the distance
   */
  void addSpeed(uint16_t key, speed_type speed) {
    keys.push_back(key);
    etl::sort(keys.begin(), keys.end());
    speeds.emplace_back(etl::make_pair(key, speed));
    if (key > maxKey) {
      maxKey = key;
    }
  }

  [[nodiscard]] const auto &getSpeeds() const {
    return speeds;
  }

  [[nodiscard]] const auto &getKeys() const {
    return keys;
  }

  uint16_t getMaxKey() const {
    return maxKey;
  }

  speed_type getSpeed(uint16_t key) {
    return retrieveByVal<uint16_t, speed_type>(keys, speeds, key);
  }

  void clear(){
    color = 0;
    id = 0;
    maxKey = 0;
    keys.clear();
    speeds.clear();
  }

  /**
   * @brief get the size needed to serialize the track
   * @return the size needed
   */
  [[nodiscard]] size_t sizeNeeded() const {
    // id, color, speed count, keys, speeds
    return 1 + 1 + 1 + keys.size() * 2 + keys.size() * sizeof(speed_type);
  }
  ParseResult fromBytes(uint8_t *bytes) {
    size_t offset = 0;
    auto id       = bytes[offset];
    this->id      = id;
    offset += sizeof id;
    auto color  = bytes[offset];
    this->color = color;
    offset += sizeof color;
    auto speed_count = bytes[offset];
    if (speed_count > MAX_SPEED_MAP_SIZE) {
      return ParseResult::VALUE_ERROR;
    }
    offset += sizeof speed_count;
    for (auto j = 0; j < speed_count; ++j) {
      auto distance = __ntohs(*reinterpret_cast<uint16_t *>(bytes + offset));
      if (distance > 6000) {
        return ParseResult::VALUE_ERROR;
      }
      offset += sizeof distance;
      if constexpr (sizeof(speed_type) == 2) {
        auto speed       = __ntohs(*reinterpret_cast<uint16_t *>(bytes + offset));
        auto fixed_speed = cnl::wrap<speed_type>(speed);
        if (fixed_speed > 10) {
          return ParseResult::VALUE_ERROR;
        }
        this->addSpeed(distance, fixed_speed);
      } else if constexpr (sizeof(speed_type) == 4) {
        auto speed       = __ntohl(*reinterpret_cast<uint32_t *>(bytes + offset));
        auto fixed_speed = cnl::wrap<speed_type>(speed);
        if (fixed_speed > 10) {
          return ParseResult::VALUE_ERROR;
        }
        this->addSpeed(distance, fixed_speed);
      } else {
        static_assert(sizeof(speed_type) == 2 || sizeof(speed_type) == 4);
      }
      offset += sizeof(speed_type);
    }
    return ParseResult::OK;
  }

  static etl::expected<Track, ParseResult> newFromBytes(uint8_t *bytes) {
    auto track = Track();
    auto res   = track.fromBytes(bytes);
    if (res == ParseResult::OK) {
      return etl::expected<Track, ParseResult>(track);
    } else {
      auto ue = etl::unexpected<ParseResult>(res);
      return etl::expected<Track, ParseResult>(ue);
    }
  }
};

using Tracks = etl::vector<Track, MAX_TRACK_SIZE>;
}

#endif // SIMPLE_TRACK_H

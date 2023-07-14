//
// Created by Kurosu Chan on 2023/5/29.
//

#ifndef SIMPLE_MESSAGE_WRAPPER_H
#define SIMPLE_MESSAGE_WRAPPER_H

#include <etl/optional.h>
#include <etl/variant.h>
#include <etl/vector.h>
#include <etl/span.h>
#include <cstring>

inline void static push_back_many(etl::ivector<uint8_t> &vec, const char *data, size_t size) {
  for (size_t i = 0; i < size; i++) {
    vec.push_back(data[i]);
  }
}

inline void static push_back_many(etl::ivector<uint8_t> &vec, const uint8_t *data, size_t size) {
  auto d = reinterpret_cast<const uint8_t *>(data);
  for (size_t i = 0; i < size; i++) {
    vec.push_back(d[i]);
  }
}

inline void static add_padding(etl::ivector<uint8_t> &vec, size_t size) {
  for (size_t i = 0; i < size; i++) {
    vec.push_back(0);
  }
}

/// TODO: unit test
namespace MessageWrapper {
// total size = 3 + 3 + 1 + 1 + 2 + 1 = 11
// use __attribute__((packed)) to avoid padding
struct WrapperHeader {
  uint8_t src[3];
  uint8_t dst[3];
  uint8_t pkt_id;
  /// inference by encoder
  uint8_t pkt_cur_count;
  /// inference by encoder
  uint16_t total_payload_size;
  /// inference by encoder
  uint8_t cur_payload_size;
} __attribute__((packed));
// pad is needed since the transmission would corrupt the last few bytes (why????)
constexpr size_t ENDING_PAD_SIZE         = 3;
constexpr size_t HEADER_SIZE             = sizeof(WrapperHeader);
constexpr size_t MAX_ENCODER_OUTPUT_SIZE = 254;
constexpr size_t MAX_PAYLOAD_SIZE        = MAX_ENCODER_OUTPUT_SIZE - HEADER_SIZE - ENDING_PAD_SIZE;
constexpr size_t MAX_DECODER_OUTPUT_SIZE = 512;
enum class WrapperDecodeResult {
  Finished,
  Unfinished,
  BadHeader,
  BadSpan,
  UnexpectedSrc,
  UnexpectedPktId,
  UnexpectedPktCount,
  UnexpectedTotalPayloadSize,
  TotalPayloadSizeTooLarge,
};

const char *decodeResultToString(WrapperDecodeResult result);

void printHeader(const WrapperHeader &header);

/// the unique packet id is the packet id + packet current count
uint16_t getUniquePktId(const WrapperHeader &header);

template <size_t N = MAX_ENCODER_OUTPUT_SIZE>
class Encoder {
  etl::vector<uint8_t, N> output;
  WrapperHeader header{};
  uint8_t *message          = nullptr;
  size_t total_message_size = 0;
  size_t cur_left           = 0;

public:
  bool setPayload(const char *payload, size_t size) {
    return setPayload(reinterpret_cast<const uint8_t *>(payload), size);
  };
  bool setPayload(const uint8_t *payload, size_t size) {
    if (size > MAX_PAYLOAD_SIZE) {
      return false;
    }
    this->message            = const_cast<uint8_t *>(payload);
    this->total_message_size = size;
    this->cur_left           = size;
    return true;
  };

  /*
   * @brief iterator like. return `etl::nullopt` when finished, otherwise return the next encoded packet.
   */
  [[nodiscard]] etl::optional<etl::vector<uint8_t, N>> next() {
    if (this->message == nullptr) {
      return etl::nullopt;
    }
    output.clear();
    push_back_many(output, header.src, 3);
    push_back_many(output, header.dst, 3);
    output.push_back(header.pkt_id);
    output.push_back(header.pkt_cur_count);
    header.total_payload_size       = total_message_size;
    auto network_total_payload_size = __htons(header.total_payload_size);
    for (auto i = 0; i < sizeof network_total_payload_size; i++) {
      output.push_back(*(reinterpret_cast<uint8_t *>(&network_total_payload_size) + i));
    }
    auto current_payload_size = std::min(cur_left, MAX_PAYLOAD_SIZE);
    header.cur_payload_size   = current_payload_size;
    output.push_back(header.cur_payload_size);
    push_back_many(output, message, header.cur_payload_size);
    add_padding(output, ENDING_PAD_SIZE);
    cur_left -= current_payload_size;
    if (cur_left < 0) [[unlikely]] { // should not happen
      reset(header.pkt_id);
      return etl::nullopt;
    } else if (cur_left == 0) { // last packet
      this->message            = nullptr;
      this->total_message_size = 0;
      this->cur_left           = 0;
      return etl::make_optional(output);
    } else { // continue
      message += current_payload_size;
      header.pkt_cur_count++;
      return etl::make_optional(output);
    }
  };

  /*
   * @param src 3 bytes
   * @param dst 3 bytes
   * @param pkt_id 1 byte
   */
  Encoder(const uint8_t *src, const uint8_t *dst, uint8_t pkt_id) {
    static_assert(N <= MAX_ENCODER_OUTPUT_SIZE, "N is too large");
    reset(src, dst, pkt_id);
  };

  void reset() {
    reset(header.pkt_id + 1);
  }
  void reset(uint8_t pkt_id) {
    output.clear();
    this->message             = nullptr;
    header.pkt_cur_count      = 0;
    header.total_payload_size = 0;
    header.cur_payload_size   = 0;
    header.pkt_id             = pkt_id;
  }

  void reset(const uint8_t *src, const uint8_t *dst, uint8_t pkt_id) {
    output.clear();
    this->message = nullptr;
    header        = {
               .src                = {0},
               .dst                = {0},
               .pkt_id             = 0,
               .pkt_cur_count      = 0,
               .total_payload_size = 0,
               .cur_payload_size   = 0,
    };
    memcpy(header.src, src, 3);
    memcpy(header.dst, dst, 3);
    header.pkt_id = pkt_id;
  };
};

template <size_t N = MAX_DECODER_OUTPUT_SIZE>
class Decoder {
  /// if decoding then header from first packet is determined
  bool _decoding = false;
  etl::array<uint8_t, N> output;
  WrapperHeader header{};
  // nullable
  // the end of previous output and the start of next output
  // https://stackoverflow.com/questions/15252002/what-is-the-past-the-end-iterator-in-stl-c
  uint8_t *separator = nullptr;

public:
  /*
   * @brief when `WrapperDecodeResult::Finished` is returned then use `getOutput()` to retrieve the output
   */
  etl::pair<MessageWrapper::WrapperDecodeResult, MessageWrapper::WrapperHeader> decode(const uint8_t *message, size_t size, bool is_simple = false) {
    auto h = decodeHeader(message, size, is_simple);
    if (!h.has_value()) {
      return etl::pair(WrapperDecodeResult::BadHeader, WrapperHeader{});
    }
    // first packet
    if (!_decoding) {
      this->header = h.value();
      _decoding    = true;
      if (header.total_payload_size > MAX_DECODER_OUTPUT_SIZE) {
        return etl::pair(WrapperDecodeResult::TotalPayloadSizeTooLarge, header);
      }
      separator = nullptr;
      std::fill(output.begin(), output.end(), 0);
      std::memcpy(output.begin(), message + HEADER_SIZE, header.cur_payload_size);
      // past-the-end
      separator = output.begin() + header.cur_payload_size + 1;

      if (header.total_payload_size == header.cur_payload_size) {
        _decoding = false;
        return etl::pair(WrapperDecodeResult::Finished, header);
      } else {
        return etl::pair(WrapperDecodeResult::Unfinished, header);
      }
      // following packets
    } else {
      if (!is_simple) {
        if (header.pkt_id != h.value().pkt_id) {
          return etl::pair(WrapperDecodeResult::UnexpectedPktId, header);
        }
        if (memcmp(header.src, h.value().src, 3) != 0) {
          return etl::pair(WrapperDecodeResult::UnexpectedSrc, header);
        }
      }
      if (header.total_payload_size != h.value().total_payload_size) {
        return etl::pair(WrapperDecodeResult::UnexpectedTotalPayloadSize, header);
      }
      if ((header.pkt_cur_count + 1) != h.value().pkt_cur_count) {
        return etl::pair(WrapperDecodeResult::UnexpectedPktCount, header);
      }
      // push_back_many(output, message + HEADER_SIZE, header.cur_payload_size);
      auto s_ = getSpan();
      if (s_.has_value()){
        auto s = s_.value();
        std::memcpy(s.begin(), message + HEADER_SIZE, header.cur_payload_size);
        separator = s.begin() + header.cur_payload_size + 1;
      }else {
        return etl::pair(WrapperDecodeResult::BadSpan, header);
      }
      if (output.size() >= header.total_payload_size) {
        _decoding = false;
        return etl::pair(WrapperDecodeResult::Finished, header);
      } else {
        return etl::pair(WrapperDecodeResult::Unfinished, header);
      }
    }
  }

  /**
   * decode with the same buffer
   * @param size the raw packet size
   * @param is_simple if it's follows `SimpleWrapper`
   * @return
   */
  etl::pair<MessageWrapper::WrapperDecodeResult, MessageWrapper::WrapperHeader> decode(size_t size, bool is_simple = false){
      if (separator == nullptr){
        return decode(output.begin(), size, is_simple);
      } else {
        return decode(separator, size, is_simple);
      }
  };

  /**
   * @brief decode the header of the message
   * @param message
   * @param size
   * @param is_simple skip parse the address field. i.e. SimpleWrapper. Message id would be ignored as well.
   * @return
   */
  static etl::optional<WrapperHeader> decodeHeader(const uint8_t *message, size_t size, bool is_simple = false) {
    if (size < HEADER_SIZE) {
      return etl::nullopt;
    }
    WrapperHeader header{};
    if (!is_simple) {
      memcpy(header.src, message, 3);
      memcpy(header.dst, message + 3, 3);
      header.pkt_id        = message[6];
      header.pkt_cur_count = message[7];
      // decode 8 & 9
      auto host_total_payload_size = __ntohs(*reinterpret_cast<const uint16_t *>(message + 8));
      header.total_payload_size    = host_total_payload_size;
      header.cur_payload_size      = message[10];
    } else {
      size_t offset        = 0;
      header.pkt_cur_count = message[offset];
      offset += 1;
      // decode 8 & 9
      auto host_total_payload_size = __ntohs(*reinterpret_cast<const uint16_t *>(message + offset));
      header.total_payload_size    = host_total_payload_size;
      offset += 2;
      header.cur_payload_size = message[offset];
    }
    return etl::make_optional(header);
  }

  [[nodiscard]] const etl::optional<etl::span<uint8_t>> getOutput() {
    if (!separator) {
      return etl::nullopt;
    } else if (separator <= output.begin()) {
      return etl::nullopt;
    } else {
      auto s = etl::span(output.begin(), separator);
      return etl::make_optional(s);
    }
  };

  etl::optional<etl::span<uint8_t>> getSpan() {
    if (!separator) {
      return etl::span(output.begin(), output.end());
    } else if (separator > output.end()) {
      return etl::nullopt;
    } else {
      auto s = etl::span(separator, output.end());
      return etl::make_optional(s);
    }
  }

  void reset() {
    header = {};
    std::fill(output.begin(), output.end(), 0);
    separator = nullptr;
    _decoding = false;
  }
};
}

#endif // SIMPLE_MESSAGE_WRAPPER_H

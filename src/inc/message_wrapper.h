//
// Created by Kurosu Chan on 2023/5/29.
//

#ifndef SIMPLE_MESSAGE_WRAPPER_H
#define SIMPLE_MESSAGE_WRAPPER_H

#include "ch32v003fun.h"
#include "unit.h"
#include <etl/optional.h>
#include <etl/variant.h>
#include <etl/vector.h>
#include <printf.h>

/// TODO: unit test
namespace MessageWrapper {
  // total size = 3 + 3 + 1 + 1 + 2 + 1 = 11
  // use __attribute__((packed)) to avoid padding
  struct WrapperHeader  {
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
  constexpr size_t HEADER_SIZE = sizeof (WrapperHeader);
  constexpr size_t MAX_ENCODER_OUTPUT_SIZE = 254;
  constexpr size_t MAX_PAYLOAD_SIZE = MAX_ENCODER_OUTPUT_SIZE - HEADER_SIZE;
  constexpr size_t MAX_DECODER_OUTPUT_SIZE = 512;
  enum class WrapperDecodeResult {
    Finished,
    Unfinished,
    BadHeader,
    UnexpectedSrc,
    UnexpectedPktId,
    UnexpectedPktCount,
    UnexpectedTotalPayloadSize,
    TotalPayloadSizeTooLarge,
  };

  const char * decodeResultToString(WrapperDecodeResult result);

  class Encoder {
    etl::vector<char, MAX_ENCODER_OUTPUT_SIZE> output;
    WrapperHeader header{};
    char *message = nullptr;
    size_t total_message_size = 0;
    size_t cur_left = 0;
  public:
    void setPayload(const char *message, size_t size);

    etl::optional<etl::vector<char, MAX_ENCODER_OUTPUT_SIZE>> next();

    /*
     * @param src 3 bytes
     * @param dst 3 bytes
     * @param pkt_id 1 byte
     */
    Encoder(const char *src, const char *dst, uint8_t pkt_id);

    void reset(const char *src, const char *dst, uint8_t pkt_id);
  };

  class Decoder {
    /// if decoding then header from first packet is determined
    bool _decoding = false;
    etl::vector<char, MAX_DECODER_OUTPUT_SIZE> output;
    WrapperHeader header{};
    public:
    /// pure function
    static etl::optional<WrapperHeader> decodeHeader(const char *message, size_t size);
    static void printHeader(const WrapperHeader &header);
    WrapperDecodeResult decode(const char *message, size_t size);
    [[nodiscard]]
    const etl::vector<char, MAX_DECODER_OUTPUT_SIZE>& getOutput() const;

    void reset();
  };

  etl::optional<WrapperHeader> getHeader(const char *message, size_t size);
}


#endif //SIMPLE_MESSAGE_WRAPPER_H
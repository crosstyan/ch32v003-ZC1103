//
// Created by Kurosu Chan on 2023/5/29.
//
#include "message_wrapper.h"

// TODO: optimize this
// eats 132 bytes of Flash
etl::optional<MessageWrapper::WrapperHeader> MessageWrapper::Decoder::decodeHeader(const uint8_t *message, size_t size, bool is_simple) {
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

etl::pair<MessageWrapper::WrapperDecodeResult, MessageWrapper::WrapperHeader>
MessageWrapper::Decoder::decode(const uint8_t *message, size_t size, bool is_simple) {
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
    output.clear();
    push_back_many(output, message + HEADER_SIZE, header.cur_payload_size);
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
    push_back_many(output, message + HEADER_SIZE, header.cur_payload_size);
    if (output.size() >= header.total_payload_size) {
      _decoding = false;
      return etl::pair(WrapperDecodeResult::Finished, header);
    } else {
      return etl::pair(WrapperDecodeResult::Unfinished, header);
    }
  }
}

void MessageWrapper::Decoder::reset() {
  header = {};
  output.clear();
  _decoding = false;
}

const etl::vector<uint8_t, MessageWrapper::MAX_DECODER_OUTPUT_SIZE> &MessageWrapper::Decoder::getOutput() const {
  return output;
}

void MessageWrapper::Decoder::printHeader(const MessageWrapper::WrapperHeader &header) {
  printf("src=\"%02x:%02x:%02x\"; ", header.src[0], header.src[1], header.src[2]);
  printf("dst=\"%02x:%02x:%02x\"; ", header.dst[0], header.dst[1], header.dst[2]);
  printf("pkt_id=%d; ", header.pkt_id);
  printf("pkt_cur_count=%d; ", header.pkt_cur_count);
  printf("total_payload_size=%d; ", header.total_payload_size);
  printf("cur_payload_size=%d; ", header.cur_payload_size);
  printf("\n");
}

const char *MessageWrapper::decodeResultToString(MessageWrapper::WrapperDecodeResult result) {
  switch (result) {
    case WrapperDecodeResult::BadHeader:
      return "BadHeader";
    case WrapperDecodeResult::Finished:
      return "Finished";
    case WrapperDecodeResult::Unfinished:
      return "Unfinished";
    case WrapperDecodeResult::UnexpectedPktId:
      return "UnexpectedPktId";
    case WrapperDecodeResult::UnexpectedTotalPayloadSize:
      return "UnexpectedTotalPayloadSize";
    case WrapperDecodeResult::UnexpectedPktCount:
      return "UnexpectedPktCount";
    case WrapperDecodeResult::UnexpectedSrc:
      return "UnexpectedSrc";
    case WrapperDecodeResult::TotalPayloadSizeTooLarge:
      return "TotalPayloadSizeTooLarge";
    default:
      return "Unknown";
  }
};

uint16_t MessageWrapper::getUniquePktId(const WrapperHeader &header) {
  return (header.pkt_id << 8) | header.pkt_cur_count;
}

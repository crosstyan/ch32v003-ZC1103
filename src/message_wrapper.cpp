//
// Created by Kurosu Chan on 2023/5/29.
//
#include "message_wrapper.h"

void MessageWrapper::printHeader(const MessageWrapper::WrapperHeader &header) {
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
    case WrapperDecodeResult::BadSpan:
      return "BadSpan";
    default:
      return "Unknown";
  }
};

uint16_t MessageWrapper::getUniquePktId(const WrapperHeader &header) {
  return (header.pkt_id << 8) | header.pkt_cur_count;
}


etl::optional<MessageWrapper::WrapperHeader> MessageWrapper:: decodeHeader(const uint8_t *message, size_t size, bool is_simple) {
  printf("?!?\n");
  using namespace MessageWrapper;
  if (size < HEADER_SIZE) {
    return etl::nullopt;
  }
  WrapperHeader header{};
  printf("??!?\n");
  if (!is_simple) {
    memcpy(header.src, message, 3);
    memcpy(header.dst, message + 3, 3);
    header.pkt_id        = message[6];
    header.pkt_cur_count = message[7];
    // decode 8 & 9
    printf(">?<\n");
    auto val = *(message + 8);
    auto val2 = *(message+9);
    uint8_t temp[2];
    std::memcpy(temp, message+8, 2);
    printf("val1: %d val2:%d\n", val, val2);
    auto val16 = *reinterpret_cast<const uint16_t *>(temp);
    printf("val two value: %d", val16);
    printf("addr: %d\n", message + 8);
    auto host_total_payload_size = __ntohs(val16);
    printf("sz: %d\n", host_total_payload_size);
    header.total_payload_size    = host_total_payload_size;
    header.cur_payload_size      = message[10];
    printf("???!?\n");
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
  printf("????!?\n");
  return etl::make_optional(header);
}

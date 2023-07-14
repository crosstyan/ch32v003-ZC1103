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

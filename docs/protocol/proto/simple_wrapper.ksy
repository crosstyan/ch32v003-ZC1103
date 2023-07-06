meta:
  id: simple_wrapper
  title: Simple Wrapper
  endian: be
doc: |
  A simpler wrapper to wrap protobuf messages without `src` and `dst`.
  The expected lower layer is BLE characteristic or simple serial that already know who they want to send to.
seq:
  - id: header
    type: header
  - id: payload
    size: header.cur_payload_size
    doc: |
      The payload of the message. 
      The size of the payload is specified in the header `cur_payload_size`.
types:
  header:
    seq:
      - id: pkt_cur_count
        type: u1
        doc: The current packet count of the message. 
      - id: total_payload_size
        type: u2
        doc: The total payload size of the message.
      - id: cur_payload_size
        type: u1
        doc: | 
          The current payload size of the message. 
          The size of all the packets for same `pkt_id` must add up to `total_payload_size`.
      - id: payload_type
        type: u1
        enum: payload_type

enums:
  payload_type:
    0x01: protobuf
    0x86: set_current
    0x06: ping

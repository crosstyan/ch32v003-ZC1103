# https://stackoverflow.com/questions/71898221/kaitai-struct-any-way-to-make-entire-body-type-dependent-on-presence-type-of-fi

meta:
  id: message_wrapper
  title: Message Wrapper
  endian: be
doc: |
  A wrapper type for messages transmitted over the physic layer.
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
      - id: src
        # no type
        size: 3
        doc: The source address of the message.
      - id: dst
        size: 3
        doc: The destination address of the message.
      - id: pkt_id
        type: u1
        doc: |
          The packet id of the message. Basically it's just a counter. The packet id won't change for a continuous message
          but the `pkt_cur_count` will increase.
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

# https://stackoverflow.com/questions/71898221/kaitai-struct-any-way-to-make-entire-body-type-dependent-on-presence-type-of-fi

meta:
  id: message_wrapper
  title: Protocol that wraps a actual message
  endian: be
seq:
  - id: header
    type: header
  - id: payload
    # we can specify a size that spans automatically to the end of the stream.
    size-eos: true
types:
  header:
    seq:
      - id: src
        type: u3
      - id: dst
        type: u3
      - id: pkt_id
        type: u1
      - id: pkt_cur_count
        type: u1
      - id: total_payload_size
        type: u2

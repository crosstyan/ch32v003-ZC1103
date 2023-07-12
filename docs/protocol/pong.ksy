meta:
  id: pong
  title: Pong Message
  endian: be
  imports: 
    - message_wrapper
doc: | 
  Pong is the Reading that comes with voltage reading when the Ping is sent
seq:
  # - id: header
  #   contents: message_wrapper.header
  - id: magic
    contents: [0x21]
  - id: payload
    type: u2
    doc: |
      Raw ADC reading.

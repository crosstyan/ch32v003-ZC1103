meta:
  id: command
  title: Command Message
  endian: be
  imports: 
    - message_wrapper
doc: | 
  command message is needed when only a single byte is needed to represent the
  command with no other parameters
seq:
  # - id: header
  #   contents: message_wrapper.header
  - id: magic
    contents: [0x11]
  - id: command
    type: u1
    enum: command
    doc: |
      command is the command to be sent to the Spot

# https://github.com/kaitai-io/kaitai_struct/issues/70
# https://github.com/kaitai-io/kaitai_struct_tests/blob/master/formats/enum_fancy.ksy
enums:
  command:
    0x06: 
      id: ping
      doc: |
        `ping` is the dummy message to check if the Spot is alive.  This message would
        be wrapper with `message_wrapper`.  Because the interesting content (`src` and
        `dst`) is in the header of `message_wrapper`, it's only content a magic.

        Only the base (server) could initiate a `ping` message with the `dst` set to
        broadcast address.  The Spot would reply with a `ping` message with the `src`
        set to its own address and `dst` set to the server address.
    0x00:
      id: start
      doc: start the track
    0x01:
      id: stop
      doc: stop the track

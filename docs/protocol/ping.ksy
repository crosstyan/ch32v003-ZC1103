meta:
  id: ping
  title: Ping Message
  endian: be
  imports: 
    - message_wrapper
doc: |
  `ping` is the dummy message to check if the Spot is alive.  This message would
  be wrapper with `message_wrapper`.  Because the interesting content (`src` and
  `dst`) is in the header of `message_wrapper`, it's only content a magic.

  Only the base (server) could initiate a `ping` message with the `dst` set to
  broadcast address.  The Spot would reply with a `ping` message with the `src`
  set to its own address and `dst` set to the server address.
seq:
  # - id: header
  #   contents: message_wrapper.header
  - id: magic
    contents: [0x06]

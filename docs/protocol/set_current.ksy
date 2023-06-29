meta:
  id: set_current
  title: SetCurrent
  endian: be
  imports:
    - spot
seq:
  - id: magic
    contents: [0x86]
  - id: current_id
    type: u2

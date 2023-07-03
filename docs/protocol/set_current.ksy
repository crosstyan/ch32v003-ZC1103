meta:
  id: set_current
  title: Set Current Position Command
  endian: be
  imports: 
    - message_wrapper
doc: |
  `set_current` is a command to set the current position of the Spot.
  The `current_id` should be less than `total` (refer to `spot_config`).
  `current_id` would be written to EEPROM.
seq:
  # - id: header
  #   contents: message_wrapper.header
  - id: magic
    contents: [0x86]
  - id: current_id
    type: u2

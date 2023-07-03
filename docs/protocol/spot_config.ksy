meta:
  id: spot_config
  title: SpotConfig
  endian: be
  imports:
    - fixed_16_16
doc: |
  `spot_config` is a configuration parameters **setter** for the Spot.  When
  `current` field is negative the device would read its current position from
  EEPROM (emulated).  `current` should always be less than `total`.
seq:
  - id: magic
    contents: [0x80]
  - id: circle_length
    type: fixed_16_16
  - id: line_length
    type: fixed_16_16
  # total number of spot
  - id: total
    type: u2
  # current spot id
  - id: current
    type: s2
  - id: update_interval
    type: u2

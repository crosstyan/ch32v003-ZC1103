meta:
  id: spot_config
  title: SpotConfig
  endian: be
  imports:
    - spot

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

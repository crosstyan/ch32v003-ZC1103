meta:
  id: spot
  title: Spot
  endian: be
seq:
  - id: magic
    # Enum is another option
    # See also: 
    #   - Enums (named integer constants)
    #   - Checking for "magic" signatures
    contents: [0x76]
  - id: tracks_count
    type: u1
  # https://doc.kaitai.io/user_guide.html#repeat-until-size-limit
  - id: tracks
    type: track
    repeat: expr
    repeat-expr: tracks_count

type:
  # the actual type spec follows cnl
  # https://github.com/johnmcfarlane/cnl
  # https://github.com/johnmcfarlane/cnl/issues/287
  # using fixed_16_16 = cnl::scaled_integer<uint32_t, cnl::power< -16>>;
  # one should use cnl::unwrap then htonl 
  # use ntohl then cnl::wrap<fixed_16_16> to deserialize
  fixed_16_16:
    seq:
      - id: digit
        type: s2
      - id: exponent
        type: u2
  speed_map:
    seq:
      - id: distance
        type: u2 # uint16_t
      - id: speed
        type: fixed_16_16
  track:
    seq:
      - id: track_id
        type: u1
      - id: track_color
        # only use the lower 3 bits (gbr)
        type: u1
      - id: speeds_count
        type: u1
      - id: speeds
        type: speed_map
        repeat: expr
        repeat-expr: speeds_count

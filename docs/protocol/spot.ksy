meta:
  id: spot
  title: Spot
  endian: be
  imports:
    - fixed_16_16
seq:
  - id: magic
    # Enum is another option
    # See also: 
    #   - Enums (named integer constants)
    #   - Checking for "magic" signatures
    contents: [0x76]
  - id: num_tracks
    type: u1
  # https://doc.kaitai.io/user_guide.html#repeat-until-size-limit
  - id: tracks
    type: track
    repeat: expr
    repeat-expr: num_tracks

types:
  # https://github.com/crosstyan/ch32v003-ZC1103/blob/3aa9ccb4255a65d6e5171883ae46da1052b0d67f/src/inc/spot.h#L23
  fixed_8_8:
    doc: | 
      Fixed Point Number Q8.8 who can represent number from 0 to 255.99609375
      with precision 0.00390625.
    seq:
      - id: digit
        type: s1
      - id: exponent
        type: u1
  # the actual type spec follows cnl
  # https://github.com/johnmcfarlane/cnl
  # https://github.com/johnmcfarlane/cnl/issues/287
  # using fixed_16_16 = cnl::scaled_integer<uint32_t, cnl::power< -16>>;
  # one should use cnl::unwrap then htonl 
  # use ntohl then cnl::wrap<fixed_16_16> to deserialize
  speed_map:
    seq:
      - id: distance
        type: u2 
      - id: speed
        type: fixed_8_8
  track:
    seq:
      - id: track_id
        type: u1
      - id: track_color
        # only use the lower 3 bits (gbr)
        type: u1
        # https://doc.kaitai.io/ksy_style_guide.html#attr-id
      - id: num_speeds
        type: u1
      - id: speeds
        type: speed_map
        repeat: expr
        repeat-expr: num_speeds

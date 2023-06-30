meta:
  title: Fixed Point Number Q16.16
  id: fixed_16_16
  # actual endian is dependent on the machine
  # for transmission purpose I choose to follow the network byte order
  endian: be
doc: |
  `fixed_16_16` is a 32-bit fixed-point number format, 
  implementing with [Compositional Numeric Library](https://github.com/johnmcfarlane/cnl)'s
  `cnl::scaled_integer<uint32_t, cnl::power< -16>>`.
  The sequence here is just a placeholder and should be ignored.
  User should treat it as a primitive type like `uint32_t`.
doc-ref: 
  - https://github.com/johnmcfarlane/cnl
  - https://chummersone.github.io/qformat.html
  - https://en.wikipedia.org/wiki/Q_(number_format)
seq:
  - id: digit
    type: s2
  - id: exponent
    type: u2

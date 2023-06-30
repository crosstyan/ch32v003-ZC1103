# Custom protocol

## Why not [protobuf](https://protobuf.dev)?

[nanopb](https://github.com/nanopb/nanopb) while quite lightweight, is still too
heavy for 16KB of ROM, who eats almost 4KB of ROM with encoding and decoding.
Besides fixed point numbers and shorts (`uint16_t`) are not supported natively.

## Protocols

I'm using [kaitai](https://kaitai.io/) to describe the protocol
and handwrite the encoder and decoder in C++.

### [Spot](spot.ksy)

![Spot](figures/spot.png)

## Compile the protocol

```bash
# graphviz or html
# don't try to generate languages code since fixed point numbers should be parsed manually
kaitai-struct-compiler -t graphviz spot.ksy
dot -Tpng spot.dot > figures/spot.png
kaitai-struct-compiler -t graphviz message_wrapper.ksy
dot -Tpng message_wrapper.dot > figures/message_wrapper.png
kaitai-struct-compiler -t graphviz spot_config.ksy
dot -Tpng spot_config.dot > figures/spot_config.png
kaitai-struct-compiler -t graphviz set_current.ksy
dot -Tpng set_current.dot > figures/set_current.png
```

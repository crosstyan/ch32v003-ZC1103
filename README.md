```
mkdir build
cd build
cmake .. -DTOOLCHAIN_PREFIX=/opt/xpack-riscv-none-elf-gcc-12.2.0-3 -G Ninja -DCMAKE_BUILD_TYPE=Release
```

Debug view.

```bash
minichlink -b -T
```

check the [protocol](docs/protocol) for more details.

See also

- [SPI 24L01+ RX demo](https://github.com/cnlohr/ch32v003fun/tree/master/examples/spi_24L01_rx)

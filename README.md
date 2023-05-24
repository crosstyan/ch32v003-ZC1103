```
mkdir build
cd build
cmake .. -DTOOLCHAIN_PREFIX=/opt/xpack-riscv-none-elf-gcc-12.2.0-3 -G Ninja -DCMAKE_BUILD_TYPE=Release
```

Debug view.

```bash
minichlink -b -T
```

See also

- [AdiHamulic/CH32V003-SPI-DMA](https://github.com/AdiHamulic/CH32V003-SPI-DMA---ILI9341)

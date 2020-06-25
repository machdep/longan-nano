# longan nano

This is an example app for Longan Nano (GD32V) board. It uses mdepx rtos.

This app reads data from CO2 sensor and shows it on the LCD display.

Bill of materials:

1) Longan Nano
   https://www.aliexpress.com/wholesale?SearchText=longan+nano
2) MH-Z19B sensor
   https://www.aliexpress.com/wholesale?SearchText=MH-Z19B
3) SiPEED debugger
   https://www.aliexpress.com/wholesale?SearchText=RISC-V+Debugger

Connect your MH-Z19B Co2 sensor as this:

| mh-z18b | longan nano |
| ------- | ----------- |
| TX      | A3          |
| RX      | A2          |
| Vin     | 5V          |
| GND     | GND         | 

### Build project
    $ git clone --recursive https://github.com/machdep/longan-nano
    $ cd longan-nano
    $ export CROSS_COMPILE=/opt/riscv/bin/riscv32-unknown-elf-
    $ make clean all

### Build openocd
    $ git clone https://github.com/bukinr/riscv-openocd
    $ cd riscv-openocd
    $ ./bootstrap
    $ ./configure
    $ make

### Program the chip
    $ /path/to/riscv-openocd/src/openocd -f ./sipeed-jtag.cfg -f ./openocd.cfg -c "program obj/longan-nano.elf; exit"

Press the reset button on the longan nano after programming.

![alt text](https://raw.githubusercontent.com/machdep/longan-nano/master/images/longan-nano.jpg)

avrdude -c usbtiny -p t84 -V -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
avrdude -c usbtiny -p t84 -U flash:w:TinyHVSP_v0.5.hex

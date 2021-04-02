# TinyHVSP - High-Voltage Serial Programmer based on ATtiny24/44/84
Stand-alone high-voltage serial programmer and fuse resetter for ATtiny 13/25/45/85. This project was superseded by the more versatile [TinyCalibrator](https://github.com/wagiminator/ATtiny84-TinyCalibrator).

- Project Video (YouTube): https://youtu.be/2Q7D2RwkOMg
- Design Files (EasyEDA): https://easyeda.com/wagiminator/z-attiny84-tinyhvprogrammer

![TinyHVSP_pic1.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny84-TinyHVSP/master/documentation/TinyHVSP_pic1.jpg)

# Hardware
The TinyHVSP is supplied with 5V via a [Micro USB connector](https://aliexpress.com/wholesale?SearchText=micro+usb+2pin+dip). The [ATtiny24/44/84](http://ww1.microchip.com/downloads/en/devicedoc/Atmel-7701_Automotive-Microcontrollers-ATtiny24-44-84_Datasheet.pdf) was chosen as the microcontroller for the TinyHVSP because it has the necessary number of GPIO pins. To generate the 12V for the High-Voltage Serial Programmer, an inexpensive [MT3608](https://datasheet.lcsc.com/szlcsc/XI-AN-Aerosemi-Tech-MT3608_C84817.pdf) boost converter IC was used. The 12V is controlled by a BJT and applied to the RESET pin of the target ATtiny if necessary. The remaining programming lines to the target are protected against a short circuit with resistors. The user interface utilizes one button and a [128x64 pixels OLED display](http://aliexpress.com/wholesale?SearchText=128+64+0.96+oled+new+4pin).

![TinyHVSP_pic3.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny84-TinyHVSP/master/documentation/TinyHVSP_pic3.jpg)

# Software
## High-Voltage Serial Programmer
The code for the High-Voltage Serial Programmer (HVSP) is quite unspectacular. Simply put, for each action, a series of instructions are sent over the data lines to the target ATtiny and the corresponding response is read. The process and the instructions are well described in the data sheet.

![hvsp.png](https://raw.githubusercontent.com/wagiminator/ATtiny84-TinyCalibrator/main/documentation/TinyCalibrator_hvsp.png)

## I²C OLED Implementation
The I²C protocol implementation is based on a crude bitbanging method. It was specifically designed for the limited resources of ATtiny10 and ATtiny13, but it works with some other AVRs (including the ATtiny24/44/84) as well. The functions for the OLED are adapted to the SSD1306 OLED module, but they can easily be modified to be used for other modules. In order to save resources, only the basic functionalities which are needed for this application are implemented. For a detailed information on the working principle of the I²C OLED implementation visit [TinyOLEDdemo](https://github.com/wagiminator/attiny13-tinyoleddemo).

## Compiling and Uploading
### If using the Arduino IDE
- Make sure you have installed [ATtinyCore](https://github.com/SpenceKonde/ATTinyCore).
- Go to **Tools -> Board -> ATtinyCore** and select **ATtiny24/44/84(a) (No bootloader)**.
- Go to **Tools** and choose the following board options:
  - **Chip:**           ATtiny24(a) or 44(a) or 84(a) (depending on your chip)
  - **Clock:**          8 MHz (internal)
  - **Millis/Micros:**  disabled
  - Leave the rest at the default settings
- Connect your programmer to your PC and to the ICSP header of the device.
- Go to **Tools -> Programmer** and select your ISP programmer (e.g. [USBasp](https://aliexpress.com/wholesale?SearchText=usbasp)).
- Go to **Tools -> Burn Bootloader** to burn the fuses.
- Open TinyHVSP sketch and click **Upload**.

### If using the precompiled hex-file
- Make sure you have installed [avrdude](https://learn.adafruit.com/usbtinyisp/avrdude).
- Connect your programmer to your PC and to the ICSP header of the device.
- Open a terminal.
- Navigate to the folder with the hex-file.
- Execute the following command (if necessary replace "t84" with the chip you use and "usbasp" with the programmer you use):
  ```
  avrdude -c usbasp -p t84 -U lfuse:w:0xe2:m -U hfuse:w:0xd7:m -U efuse:w:0xff:m -U flash:w:tinyhvsp.hex
  ```

### If using the makefile (Linux/Mac)
- Make sure you have installed [avr-gcc toolchain and avrdude](http://maxembedded.com/2015/06/setting-up-avr-gcc-toolchain-on-linux-and-mac-os-x/).
- Connect your programmer to your PC and to the ICSP header of the device.
- Open the makefile and change the chip if you are not using an ATtiny84 and the programmer if you are not using usbasp.
- Open a terminal.
- Navigate to the folder with the makefile and the Arduino sketch.
- Run "make install" to compile, burn the fuses and upload the firmware.

# Operating Instructions
1. Connect a 5V power supply to the micro USB port.
2. Place the ATtiny13/25/45/85 in the IC socket. Use an [SOP adapter](https://aliexpress.com/wholesale?SearchText=sop-8+150mil+200mil+adapter) for SMD parts.
3. Press OK-button and follow the instructions on the display.

![TinyHVSP_pic5.png](https://raw.githubusercontent.com/wagiminator/ATtiny84-TinyHVSP/master/documentation/TinyHVSP_pic5.png)

# References, Links and Notes
1. [TinyCalibrator](https://github.com/wagiminator/ATtiny84-TinyCalibrator)
2. [AVR HVSP by Paul Willoughby](http://www.rickety.us/2010/03/arduino-avr-high-voltage-serial-programmer/)
3. [AVR HV Programmer by Jeff Keyzer](https://mightyohm.com/blog/2008/09/arduino-based-avr-high-voltage-programmer/)
4. [PiggyFuse by Ralph Doncaster](https://nerdralph.blogspot.com/2018/05/piggyfuse-hvsp-avr-fuse-programmer.html)
5. [I²C OLED Tutorial](https://github.com/wagiminator/attiny13-tinyoleddemo)
6. [ATtiny24/44/84 Datasheet](http://ww1.microchip.com/downloads/en/devicedoc/Atmel-7701_Automotive-Microcontrollers-ATtiny24-44-84_Datasheet.pdf)
7. [ATtiny13A Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/doc8126.pdf)
8. [ATtiny25/45/85 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf)

![TinyHVSP_pic4.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny84-TinyHVSP/master/documentation/TinyHVSP_pic4.jpg)

# License
![license.png](https://i.creativecommons.org/l/by-sa/3.0/88x31.png)

This work is licensed under Creative Commons Attribution-ShareAlike 3.0 Unported License. 
(http://creativecommons.org/licenses/by-sa/3.0/)

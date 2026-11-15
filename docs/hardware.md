# Hardware and connection

[← Documentation index](README.md)

## Connection details

Communication can be established thru one of the two sockets: CN-CNT or CN-NMODE. If you have an existing Panasonic CZ-TAW1 WiFi interface that you want to replace with HeishaMon, it is only a matter of plugging the cable out from CZ-TAW1 and reconnecting to your HeishaMon device. However it is not possible to use HeishaMon and the original CZ-TAW1 module together as an active device. It is however possible to put HeishaMon on "Listen Only" mode which will allow HeishaMon and the original CZ-TAW1 module to co-exist. The only downside to this is that HeishaMon is unable to send commands and use the optional PCB option.

Communication parameters: TTL 5V UART 9600,8,E,1  \
 \
CN-CNT Pin-out (from top to bottom) \
1 - +5V (250mA)  \
2 - 0-5V TX (from heatpump) \
3 - 0-5V RX (to heatpump)\
4 - +12V (250mA) \
5 - GND \
 \
CN-NMODE Pin-out (from left to right) \
"Warning! As printed on the PCB, the left pin is pin 4 and right pin is pin 1. Do not count 1 to 4 from left!  \
4 - +5V (250mA)  \
3 - 0-5V TX (from heatpump) \
2 - 0-5V RX (to heatpump) \
1 - GND

HeishaMon will receive power from the Panasonic over the cable (5v power).

## Long distance connection
It it possible to connect the HeishaMon over a long distance. Up to 5 meter is working with normal cabling. For longer distances a TTL-to-RS485 configuration as show in the picture below is possible. The however requires HeishaMon to be powered externally using 5v power (for example from an USB cable).

![TTL-over-RS485 HeishaMon long distance](../optional-long-distance-heishamon.png)


## Where to get connectors
[RS-Online orders](../Connectors_RSO.md)

[Conrad orders](../Connectors_Conrad.md)

Use some 24 AWG shielded 4-conductors cable.


## The HeishaMon hardware itself
The PCB needed to connect to the heatpump is just a ESP8266 or a ESP32 (for the Heishamon Large it uses the ESP32-S3-MINI-1-N4R2) with some level shifting to get the 5v tx/rx down down 3.3v. That is the only thing to make it work.
For the ESP8266 the Heishamon software uses the schematic as described in the [ESP8266 hardware design guide](https://documentation.espressif.com/esp8266_hardware_design_guidelines_en.pdf) figure 1-10c on page 12. The GPIO5 shown there is the enable on boot (or disabled when listen only is selected). But it is also possible to user a proper schmitt buffer instead of basic level shifting.
The rest of Heishamon software is just adding more stuff, like Dallas 1wire, Opentherm etc.

To make things easy you can order a completed PCB from some project members: \
[Lectronz shop](https://lectronz.com/products/heishamon-communication-pcb) from Igor Ybema (aka TheHogNL) based in the Netherlands

## Building the arduino image yourself
boards: \
esp8266 by esp8266 community version 3.0.2 [Arduino](https://github.com/esp8266/Arduino/releases/tag/3.0.2)

All the [libs we use](../LIBSUSED.md) necessary for compiling.

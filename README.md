# ESP32-Marauder-Cheap-Yellow-Display
<p align="center"><img alt="Marauder logo" src="https://github.com/justcallmekoko/ESP32Marauder/blob/master/pictures/marauder3L.jpg?raw=true" width="300"></p>

## ESP32 Marauder Cheap Yellow Display
## Web flasher instructions:
### Visit -> http://espressif.github.io/esptool-js<br>
Change baud rate -> 115200<br>
Change flash address -> 10000<br>
Choose <i>esp32_marauder_v0_13_5_20240203_cyd.bin</i> (use _inverted.bin if your colors come out wrong)<br>
Program
<br><br>
Tested on https://amazon.com/dp/B0BVFXR313 or https://amazon.com/dp/B0CLR7MQ91<br>
## Edit: Now working without hw mods thanks to https://github.com/ggaljoen/TFT_eSPI
</p>
Add libraries to your arduino libraries folder<br><br>
Configure arduino as explained here<br> https://github.com/justcallmekoko/ESP32Marauder/wiki/arduino-ide-setup#these-next-steps-only-apply-if-you-plan-to-build-the-full-esp32-marauder-firmware-from-source<br>

### Make sure upload speed is set to 115200 in Arduino. I use version 1.8.19

I couldn't get this CYD marauder fork to work https://github.com/smoochiee/ESP32Marauder but after a while came up with this.<br>

### Big thanks to https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display and the community Discord especially @cod5fgzj and obviously JustCallMeKoko for making this a thing

<p align="center">
  <img src="https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/screenshots/2.gif" alt="Yes">
</p>
<p align="center">
  <img src="https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/screenshots/1.gif" alt="Yes">
</p>

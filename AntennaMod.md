# Enhancing ESP Board Reception without an External Antenna Connector

If you've purchased an ESP board that lacks a connector for an external antenna and you're experiencing the limits of reception, there's a solution that doesn't involve the complex task of moving a tiny 0603 sized resistor to utilize the external antenna socket on some boards.

This workaround is relatively simple and the required parts are inexpensive, often found on platforms like eBay or AliExpress for just a few dollars.

## Required Components

![Components](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/1.jpeg)
1. **ESP/CYD of your choosing**
2. **UFL to reverse polarity SMA lead with UFL connector removed and 5mm of shield and center exposed and tinned with solder**
3. **Reverse polarity SMA 2.4GHz antenna**
## Step-by-Step Guide
<!--
1. **Prepare Your Workspace and Tools:** Make sure you have a steady hand and the right tools. A fine soldering tip and some magnification can help.

2. **Locate the Antenna Connection Points:** Identify the points on your ESP board where the antenna connects. This will be your working area.

3. **Attach the Antenna:** Carefully solder the antenna's lead to the designated point on the ESP board. Ensure a solid connection without bridging adjacent contacts.

4. **Test the Connection:** Before proceeding further, it's crucial to test the connection. Use a multimeter to ensure there's no short circuit.

5. **Seal and Protect:** Once you've confirmed the functionality, consider using some non-conductive lacquer to protect the exposed soldered area.

6. **Final Testing:** With the external antenna now attached, test the ESP board's reception in its intended environment. You should notice a significant improvement in signal strength and stability.
-->
![Step 1](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/2.jpeg)
![Step 2](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/3.jpeg)
![Step 3](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/4.jpeg)
![Step 4](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/5.jpeg)
![Step 5](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/6.jpeg)
![Final Result](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/7.jpeg)

## Conclusion

By following these steps, you can significantly improve the reach of your Cheap-Yellow-Display. For more infomation/examples, click [here.](https://community.home-assistant.io/t/how-to-add-an-external-antenna-to-an-esp-board)

# Add an antenna the easy way

You can exchange the stock wroom-32 for a wroom-32u with built-in ipex/u.fl connector.

## Materials

- **WROOM-32U Module**  
  - Available on [AliExpress](https://www.aliexpress.us/item/1005006566368554.html) or [Amazon](https://a.co/d/2cFy1Jf)
- **SMA Antenna**  
  - Suggested: [Amazon Link](https://a.co/d/41h6VpV)
- **UFL to SMA Adapter**  
  - [Available on Amazon](https://a.co/d/7glUbBC)
- **Flux**  
  - [I like Amtech](https://northridgefix.com/product/amtech-nc-559-v2-tf-flux-10ml-syringe-plunger-2-needle-sizes/)
- **Solder Paste**
  - [I use Mechanic](https://www.aliexpress.us/item/1005007333777376.html)
- **Hot Air Station**
## Procedure

### Step 1: Preparation

1. **Setup**: Clean your workspace. Secure the board to prevent movement during the process.
2. **Flux**: Liberally apply flux around the ESP-WROOM module. Struggling? Use more flux.

### Step 2: Module Removal

1. **Heat Application**: Set the hot air station to approximately 350-400°C. Apply heat evenly across the module to avoid damage to adjacent components.

   ![Heat Application](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/8.JPG)

2. **Remove the Module**: Once the solder is molten, carefully lift the ESP-WROOM module using tweezers. If it resists, apply additional heat as needed.

    ![Remove the Module](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/9.JPG)
   
4. **Clean the Pads**: Add solder to the esp pads on the CYD, use solder braid to remove it, clean with IPA, and then lay down solder paste.

    ![Clean the Pads](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/10.JPG)

### Step 3: Install the WROOM-32U

1. **Position the Module**: If you don't use solder paste simply ensure correct alignment and solder a pin at each corner. 
2. **Hot Air**: If you chose the hot air route, apply heat evenly until the paste flows. After the initial set, apply more heat and give the module a tap with tweezers ever so slightly for it to seat fully.
3. **Inspect the Joints**: Thoroughly inspect the solder joints for any defects such as cold joints or bridges. A microscope is invaluable for this and many other challenges you'll face.

  ![New module](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/11.jpg)

### Step 4: Reflash Marauder

1. **CYM WebFlasher**:
   https://fr4nkfletcher.github.io/Adafruit_WebSerial_ESPTool/
  
### Step 5: Attach the Antenna

1. **Select an Antenna**: Choose an appropriate antenna for your application. Recommended options are available on [Amazon](https://a.co/d/41h6VpV).
2. **Connect the Antenna**: Attach the UFL to SMA adapter to the WROOM-32U, then connect the SMA antenna.
3. **Test the Board**: Power on the board and have fun. Don't be a dick!
    
    ![Alfa](https://github.com/Fr4nkFletcher/ESP32-Marauder-Cheap-Yellow-Display/blob/master/img/12.jpg)
   

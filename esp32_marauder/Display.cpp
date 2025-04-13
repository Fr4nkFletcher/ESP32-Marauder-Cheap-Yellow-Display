#include "Display.h"
#include "lang_var.h"

#ifdef HAS_SCREEN

Display::Display()
{
}

// Function to prepare the display and the menus
void Display::RunSetup()
{
  run_setup = false;

  // Need to declare new
  display_buffer = new LinkedList<String>();

  #ifdef SCREEN_BUFFER
    screen_buffer = new LinkedList<String>();
  #endif

  tft.init();

  #ifndef MARAUDER_M5STICKC
    tft.setRotation(0); // Portrait
  #endif
  #ifdef MARAUDER_M5STICKC
    tft.setRotation(1);
  #endif
  #ifdef MARAUDER_REV_FEATHER
    tft.setRotation(1);
  #endif

  tft.setCursor(0, 0);

  #ifdef HAS_SCREEN
    #if defined(CYD_28)
      uint16_t calData[5] = { 350, 3465, 188, 3431, 2 };
    #elif defined(CYD_24)
      uint16_t calData[5] = { 481, 3053, 433, 3296, 3 };
    #elif defined(CYD_24CAP)
      uint16_t calData[5] = { 405, 3209, 297, 3314, 2 };
    #elif defined(CYD_24G)
      uint16_t calData[5] = { 405, 3209, 297, 3314, 2 };
    #elif defined(CYD_32)
      uint16_t calData[5] = { 251, 3539, 331, 3534, 6 };
    #elif defined(CYD_35)
      uint16_t calData[5] = { 309, 3465, 297, 3552, 6 };
    #elif defined(TFT_DIY)
      uint16_t calData[5] = { 339, 3470, 237, 3438, 2 };
    #endif
    #if !defined(CYD_32CAP) && !defined(CYD_35CAP)
      tft.setTouch(calData);
    #endif
  #endif

  clearScreen();

  #ifdef KIT
    pinMode(KIT_LED_BUILTIN, OUTPUT);
  #endif

  #ifdef MARAUDER_REV_FEATHER
    pinMode(7, OUTPUT);
    delay(10);
    digitalWrite(7, HIGH);
  #endif
}

void Display::drawFrame()
{
  tft.drawRect(FRAME_X, FRAME_Y, FRAME_W, FRAME_H, TFT_BLACK);
}

void Display::tftDrawRedOnOffButton() {
  tft.fillRect(REDBUTTON_X, REDBUTTON_Y, REDBUTTON_W, REDBUTTON_H, TFT_RED);
  tft.fillRect(GREENBUTTON_X, GREENBUTTON_Y, GREENBUTTON_W, GREENBUTTON_H, TFT_DARKGREY);
  drawFrame();
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text03, GREENBUTTON_X + (GREENBUTTON_W / 2), GREENBUTTON_Y + (GREENBUTTON_H / 2));
  this->SwitchOn = false;
}

void Display::tftDrawGreenOnOffButton() {
  tft.fillRect(GREENBUTTON_X, GREENBUTTON_Y, GREENBUTTON_W, GREENBUTTON_H, TFT_GREEN);
  tft.fillRect(REDBUTTON_X, REDBUTTON_Y, REDBUTTON_W, REDBUTTON_H, TFT_DARKGREY);
  drawFrame();
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text04, REDBUTTON_X + (REDBUTTON_W / 2) + 1, REDBUTTON_Y + (REDBUTTON_H / 2));
  this->SwitchOn = true;
}

void Display::tftDrawGraphObjects(byte x_scale)
{
  tft.fillRect(11, 5, x_scale+1, 120, TFT_BLACK); // positive start point
  tft.fillRect(11, 121, x_scale+1, 119, TFT_BLACK); // negative start point
  tft.drawFastVLine(10, 5, HEIGHT_1 - 10, TFT_WHITE); // y axis (adjusted for screen height)
  tft.drawFastHLine(10, HEIGHT_1 - 1, 310, TFT_WHITE); // x axis
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(3, 6); tft.print("+");
  tft.setCursor(3, HEIGHT_1 - 12); tft.print("0");
}

void Display::tftDrawEapolColorKey()
{
  tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.fillRect(14, 0, 15, 8, TFT_CYAN); tft.setCursor(30, 0); tft.print(" - EAPOL"); 
}

void Display::tftDrawColorKey()
{
  tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.fillRect(14, 0, 15, 8, TFT_GREEN); tft.setCursor(30, 0); tft.print(" - Beacons"); 
  tft.fillRect(14, 8, 15, 8, TFT_RED); tft.setCursor(30, 8); tft.print(" - Deauths");
  tft.fillRect(14, 16, 15, 8, TFT_BLUE); tft.setCursor(30, 16); tft.print(" - Probes");
}

void Display::tftDrawXScaleButtons(byte x_scale)
{
  tft.drawFastVLine(234, 0, 20, TFT_WHITE);
  tft.setCursor(208, 21); tft.setTextColor(TFT_WHITE); tft.setTextSize(1); tft.print("X Scale:"); tft.print(x_scale);
  key[0].initButton(&tft, 220, 10, 20, 20, TFT_BLACK, TFT_CYAN, TFT_BLACK, "-", 2);
  key[1].initButton(&tft, 249, 10, 20, 20, TFT_BLACK, TFT_CYAN, TFT_BLACK, "+", 2);
  key[0].setLabelDatum(1, 5, MC_DATUM);
  key[1].setLabelDatum(1, 5, MC_DATUM);
  key[0].drawButton();
  key[1].drawButton();
}

void Display::tftDrawYScaleButtons(byte y_scale)
{
  tft.drawFastVLine(290, 0, 20, TFT_WHITE);
  tft.setCursor(265, 21); tft.setTextColor(TFT_WHITE); tft.setTextSize(1); tft.print("Y Scale:"); tft.print(y_scale);
  key[2].initButton(&tft, 276, 10, 20, 20, TFT_BLACK, TFT_MAGENTA, TFT_BLACK, "-", 2);
  key[3].initButton(&tft, 305, 10, 20, 20, TFT_BLACK, TFT_MAGENTA, TFT_BLACK, "+", 2);
  key[2].setLabelDatum(1, 5, MC_DATUM);
  key[3].setLabelDatum(1, 5, MC_DATUM);
  key[2].drawButton();
  key[3].drawButton();
}

void Display::tftDrawChannelScaleButtons(int set_channel, bool lnd_an)
{
  if (lnd_an) {
    tft.drawFastVLine(178, 0, 20, TFT_WHITE);
    tft.setCursor(145, 21); tft.setTextColor(TFT_WHITE); tft.setTextSize(1); tft.print(text10); tft.print(set_channel);

    key[4].initButton(&tft, // channel - box
                          164,
                          10, // x, y, w, h, outline, fill, text
                          EXT_BUTTON_WIDTH,
                          EXT_BUTTON_WIDTH,
                          TFT_BLACK, // Outline
                          TFT_BLUE, // Fill
                          TFT_BLACK, // Text
                          "-",
                          2);
    key[5].initButton(&tft, // channel + box
                          193,
                          10, // x, y, w, h, outline, fill, text
                          EXT_BUTTON_WIDTH,
                          EXT_BUTTON_WIDTH,
                          TFT_BLACK, // Outline
                          TFT_BLUE, // Fill
                          TFT_BLACK, // Text
                          "+",
                          2);
  }

  else {
    key[4].initButton(&tft, // channel - box
                          (EXT_BUTTON_WIDTH / 2) * 6,
                          (STATUS_BAR_WIDTH * 2) + CHAR_WIDTH - 1, // x, y, w, h, outline, fill, text
                          EXT_BUTTON_WIDTH,
                          EXT_BUTTON_WIDTH,
                          TFT_BLACK, // Outline
                          TFT_BLUE, // Fill
                          TFT_BLACK, // Text
                          "-",
                          2);
    key[5].initButton(&tft, // channel + box
                          (EXT_BUTTON_WIDTH / 2) * 10,
                          (STATUS_BAR_WIDTH * 2) + CHAR_WIDTH - 1, // x, y, w, h, outline, fill, text
                          EXT_BUTTON_WIDTH,
                          EXT_BUTTON_WIDTH,
                          TFT_BLACK, // Outline
                          TFT_BLUE, // Fill
                          TFT_BLACK, // Text
                          "+",
                          2);
  }

  key[4].setLabelDatum(1, 5, MC_DATUM);
  key[5].setLabelDatum(1, 5, MC_DATUM);

  key[4].drawButton();
  key[5].drawButton();
}

void Display::tftDrawExitScaleButtons(bool lnd_an)
{
  //tft.drawFastVLine(178, 0, 20, TFT_WHITE);
  //tft.setCursor(145, 21); tft.setTextColor(TFT_WHITE); tft.setTextSize(1); tft.print("Channel:"); tft.print(set_channel);
  if (lnd_an) {

    key[6].initButton(&tft, // Exit box
                      137,
                      10, // x, y, w, h, outline, fill, text
                      EXT_BUTTON_WIDTH,
                      EXT_BUTTON_WIDTH,
                      TFT_ORANGE, // Outline
                      TFT_RED, // Fill
                      TFT_BLACK, // Text
                      "X",
                      2);
  }

  else {
    key[6].initButton(&tft, // Exit box
                      EXT_BUTTON_WIDTH / 2,
                      (STATUS_BAR_WIDTH * 2) + CHAR_WIDTH - 1, // x, y, w, h, outline, fill, text
                      EXT_BUTTON_WIDTH,
                      EXT_BUTTON_WIDTH,
                      TFT_ORANGE, // Outline
                      TFT_RED, // Fill
                      TFT_BLACK, // Text
                      "X",
                      2);
  }
  key[6].setLabelDatum(1, 5, MC_DATUM);
  key[6].drawButton();
}

void Display::twoPartDisplay(String center_text)
{
  tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  tft.fillRect(0, 16, HEIGHT_1, 144, TFT_YELLOW);
  tft.setTextWrap(true);
  tft.setFreeFont(NULL);
  tft.setCursor(0, 82);
  tft.println(center_text);
  tft.setFreeFont(MENU_FONT);
  tft.setTextWrap(false);
}

void Display::touchToExit()
{
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.fillRect(0, 32, HEIGHT_1, 16, TFT_LIGHTGREY);
  tft.drawCentreString(text11, TFT_WIDTH / 2, 32, 2);
}

void Display::clearScreen()
{
  //Serial.println(F("clearScreen()"));
  #ifndef MARAUDER_V7
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
  #else
    tft.fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
    tft.setCursor(0, 0);
  #endif
}

#ifdef SCREEN_BUFFER
void Display::scrollScreenBuffer(bool down) {
  if (!down) {
    this->screen_buffer->shift();
  }
}
#endif

void Display::processAndPrintString(TFT_eSPI& tft, const String& originalString) {
  uint16_t text_color = TFT_GREEN;
  uint16_t background_color = TFT_BLACK;
  String new_string = originalString;
  if (new_string.startsWith(RED_KEY)) {
    text_color = TFT_RED;
    new_string.remove(0, strlen(RED_KEY));
  } else if (new_string.startsWith(GREEN_KEY)) {
    text_color = TFT_GREEN;
    new_string.remove(0, strlen(GREEN_KEY));
  } else if (new_string.startsWith(CYAN_KEY)) {
    text_color = TFT_CYAN;
    new_string.remove(0, strlen(CYAN_KEY));
  } else if (new_string.startsWith(WHITE_KEY)) {
    text_color = TFT_WHITE;
    new_string.remove(0, strlen(WHITE_KEY));
  } else if (new_string.startsWith(MAGENTA_KEY)) {
    text_color = TFT_MAGENTA;
    new_string.remove(0, strlen(MAGENTA_KEY));
  }
  tft.setTextColor(text_color, background_color);
  tft.print(new_string);
}

void Display::displayBuffer(bool do_clear)
{
  if (this->display_buffer->size() > 0)
  {
    int print_count = 1;
    while ((display_buffer->size() > 0) && (print_count > 0))
    {
      #ifndef SCREEN_BUFFER
        xPos = 0;
        if ((display_buffer->size() > 0) && (!loading))
        {
          printing = true;
          delay(print_delay_1);
          yDraw = scroll_line(TFT_RED);
          tft.setCursor(xPos, yDraw);
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.print(display_buffer->shift());
          printing = false;
          delay(print_delay_2);
        }
        if (!tteBar)
          blank[(18+(yStart - TOP_FIXED_AREA) / TEXT_HEIGHT)%19] = xPos;
        else
          blank[(18+(yStart - TOP_FIXED_AREA_2) / TEXT_HEIGHT)%19] = xPos;
      #else
        xPos = 0;
        if (this->screen_buffer->size() >= MAX_SCREEN_BUFFER)
          this->scrollScreenBuffer();
        screen_buffer->add(display_buffer->shift());
        for (int i = 0; i < this->screen_buffer->size(); i++) {
          tft.setCursor(xPos, (i * 12) + (SCREEN_HEIGHT / 6));
          String spaces = String(' ', TFT_WIDTH / CHAR_WIDTH);

          tft.print(spaces);
          tft.setCursor(xPos, (i * 12) + (SCREEN_HEIGHT / 6));

          this->processAndPrintString(tft, this->screen_buffer->get(i));
        }
      #endif
      print_count--;
    }
  }
}

void Display::showCenterText(String text, int y)
{
  tft.setCursor((SCREEN_WIDTH - (text.length() * (6 * BANNER_TEXT_SIZE))) / 2, y);
  tft.println(text);
}

void Display::initScrollValues(bool tte)
{
  yDraw = YMAX - BOT_FIXED_AREA - TEXT_HEIGHT;
  xPos = 0;
  if (!tte)
  {
    yStart = TOP_FIXED_AREA;
    yArea = YMAX - TOP_FIXED_AREA - BOT_FIXED_AREA;
  }
  else
  {
    yStart = TOP_FIXED_AREA_2;
    yArea = YMAX - TOP_FIXED_AREA_2 - BOT_FIXED_AREA;
  }
  for(uint8_t i = 0; i < 18; i++) blank[i] = 0;
}

int Display::scroll_line(uint32_t color) {
  int yTemp = yStart;
  if (!tteBar)
  {
    tft.fillRect(0, yStart, blank[(yStart - TOP_FIXED_AREA) / TEXT_HEIGHT], TEXT_HEIGHT, color);
    yStart += TEXT_HEIGHT;
    if (yStart >= YMAX - BOT_FIXED_AREA) yStart = TOP_FIXED_AREA + (yStart - YMAX + BOT_FIXED_AREA);
  }
  else
  {
    tft.fillRect(0, yStart, blank[(yStart - TOP_FIXED_AREA_2) / TEXT_HEIGHT], TEXT_HEIGHT, color);
    yStart += TEXT_HEIGHT;
    if (yStart >= YMAX - BOT_FIXED_AREA) yStart = TOP_FIXED_AREA_2 + (yStart - YMAX + BOT_FIXED_AREA);
  }
  scrollAddress(yStart);
  return yTemp;
}

void Display::setupScrollArea(uint16_t tfa, uint16_t bfa) {
  #ifdef HAS_ILI9341
    const uint8_t SCROLL_DEF_CMD = ILI9341_VSCRDEF;
  #endif
  #ifdef HAS_ST7796
    const uint8_t SCROLL_DEF_CMD = ST7796_VSCRDEF;
  #endif
  #ifdef HAS_ST7789
    const uint8_t SCROLL_DEF_CMD = ST7789_VSCRDEF;
  #endif

  #ifdef SCROLL_DEF_CMD
    tft.writecommand(SCROLL_DEF_CMD);
    tft.writedata(tfa >> 8);
    tft.writedata(tfa);
    tft.writedata((YMAX - tfa - bfa) >> 8);
    tft.writedata(YMAX - tfa - bfa);
    tft.writedata(bfa >> 8);
    tft.writedata(bfa);
  #endif
}

void Display::scrollAddress(uint16_t vsp) {
  #ifdef HAS_ILI9341
    const uint8_t SCROLL_PTR_CMD = ILI9341_VSCRSADD;
  #endif
  #ifdef HAS_ST7796
    const uint8_t SCROLL_PTR_CMD = ST7796_VSCRSADD;
  #endif
  #ifdef HAS_ST7789
    const uint8_t SCROLL_PTR_CMD = ST7789_VSCRSADD;
  #endif

  #ifdef SCROLL_PTR_CMD
    tft.writecommand(SCROLL_PTR_CMD);
    tft.writedata(vsp >> 8);
    tft.writedata(vsp);
  #endif
}

void Display::jpegRender(int xpos, int ypos) {
  uint16_t *pImg;
  int16_t mcu_w = JpegDec.MCUWidth;
  int16_t mcu_h = JpegDec.MCUHeight;
  int32_t max_x = JpegDec.width;
  int32_t max_y = JpegDec.height;
  int32_t min_w = minimum(mcu_w, max_x % mcu_w);
  int32_t min_h = minimum(mcu_h, max_y % mcu_h);
  int32_t win_w = mcu_w;
  int32_t win_h = mcu_h;
  uint32_t drawTime = millis();
  max_x += xpos;
  max_y += ypos;

  while (JpegDec.readSwappedBytes()) {
    pImg = JpegDec.pImage;
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;
    if (win_w != mcu_w)
    {
      for (int h = 1; h < win_h - 1; h++)
      {
        memcpy(pImg + h * win_w, pImg + (h + 1) * mcu_w, win_w << 1);
      }
    }
    if (mcu_x < tft.width() && mcu_y < tft.height())
    {
      tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    }
    else if ((mcu_y + win_h) >= tft.height()) JpegDec.abort();
  }
  drawTime = millis() - drawTime;
}

void Display::jpegInfo() {
  // Intentionally left empty as in both versions
}

void createArray(const char *filename) {
  fs::File jpgFile = SPIFFS.open(filename, "r");
  if (!jpgFile) {
    Serial.print("ERROR: File \""); Serial.print(filename); Serial.println ("\" not found!");
    return;
  }
  uint8_t data;
  byte line_len = 0;
  Serial.println("");
  Serial.println(F("// Generated by a JPEGDecoder library example sketch:"));
  Serial.println(F("// https://github.com/Bodmer/JPEGDecoder"));
  Serial.println("");
  Serial.println(F("#if defined(__AVR__)"));
  Serial.println(F("  #include <avr/pgmspace.h>"));
  Serial.println(F("#endif"));
  Serial.println("");
  Serial.print(F("const uint8_t "));
  while (*filename != '.') Serial.print(*filename++);
  Serial.println(F("[] PROGMEM = {"));
  while (jpgFile.available()) {
    data = jpgFile.read();
    Serial.print("0x"); if (abs(data) < 16) Serial.print("0");
    Serial.print(data, HEX); Serial.print(",");
    line_len++;
    if (line_len >= 32) {
      line_len = 0;
      Serial.println();
    }
  }
  Serial.println("};\r\n");
  jpgFile.close();
}

#ifdef ESP8266
void Display::listFiles(void) {
  Serial.println();
  Serial.println(F("SPIFFS files found:"));
  fs::Dir dir = SPIFFS.openDir("/");
  String line = "=====================================";
  Serial.println(line);
  Serial.println(F("  File name               Size"));
  Serial.println(line);
  while (dir.next()) {
    String fileName = dir.fileName();
    Serial.print(fileName);
    int spaces = 21 - fileName.length();
    while (spaces--) Serial.print(" ");
    fs::File f = dir.openFile("r");
    String fileSize = (String) f.size();
    spaces = 10 - fileSize.length();
    while (spaces--) Serial.print(" ");
    Serial.println(fileSize + " bytes");
  }
  Serial.println(line);
  Serial.println();
  delay(1000);
}
#endif

#ifdef ESP32
void Display::listFiles(void) {
  listDir(SPIFFS, "/", 0);
}

void Display::listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.println();
  Serial.println(F("SPIFFS files found:"));
  Serial.printf("Listing directory: %s\n", "/");
  String line = "=====================================";
  Serial.println(line);
  Serial.println(F("  File name               Size"));
  Serial.println(line);
  fs::File root = fs.open(dirname);
  if (!root) {
    Serial.println(F("Failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }
  fs::File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("DIR : ");
      String fileName = file.name();
      Serial.print(fileName);
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      String fileName = file.name();
      Serial.print("  " + fileName);
      int spaces = 20 - fileName.length();
      while (spaces--) Serial.print(" ");
      String fileSize = (String) file.size();
      spaces = 10 - fileSize.length();
      while (spaces--) Serial.print(" ");
      Serial.println(fileSize + " bytes");
    }
    file = root.openNextFile();
  }
  Serial.println(line);
  Serial.println();
  delay(1000);
}
#endif

void Display::updateBanner(String msg)
{
  this->buildBanner(msg, current_banner_pos);
}

void Display::buildBanner(String msg, int xpos)
{
  this->tft.fillRect(0, STATUS_BAR_WIDTH, SCREEN_WIDTH, TEXT_HEIGHT, TFT_BLACK);
  this->tft.setFreeFont(NULL);
  this->tft.setTextSize(BANNER_TEXT_SIZE);
  this->tft.setTextColor(TFT_WHITE, TFT_BLACK);
  this->showCenterText(msg, STATUS_BAR_WIDTH);
}

void Display::main(uint8_t scan_mode)
{ 
  if ((scan_mode == LV_JOIN_WIFI) || (scan_mode == LV_ADD_SSID)) {
    lv_task_handler();
  return;
  }
    
  
}
// End SPIFFS_functions

#endif
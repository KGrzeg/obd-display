#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "Adafruit_miniTFTWing.h"

#include "BluetoothSerial.h"
#include "ELMduino.h"

#include "utils.h"
#include "images.h"
#include "OBDClient.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define ELM_PORT SerialBT
#define DEBUG_PORT Serial

#define TFT_RST 4
#define TFT_CS 5
#define TFT_DC 2
#define BRIGHTNESS_PIN 34

#define BMW_ORANGE 0xf880
#define MAX_TEMP 115
#define MIN_VOLTAGE 9
#define MAX_VOLTAGE 16

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
BluetoothSerial SerialBT;
ELM327 myELM327;
OBDClient elmoo(myELM327);

bool connected = false;
bool demo_mode = false; // set in source code

uint16_t potValue = 0;
struct
{
  uint32_t rpm;
  float voltage;
  uint16_t temperature;
} gui_data;

void testroundrects()
{
  tft.fillScreen(ST77XX_BLACK);
  uint16_t color = 100;
  int x = 0;
  int y = 0;
  int w = tft.width() - 2;
  int h = tft.height() - 2;
  while ((w > 10) && (h > 10))
  {
    tft.drawRoundRect(x, y, w, h, 5, color);
    x += 2;
    y += 3;
    w -= 4;
    h -= 6;
    color += 1100;
  }
}

template <typename T>
void my_log(T msg)
{
  tft.println(msg);
  DEBUG_PORT.println(msg);
}

void setup_screen()
{
  tft.initR(INITR_18BLACKTAB); // initialize a ST7735S chip, mini display
  tft.setRotation(0);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 0);
  tft.setTextWrap(true);

  tft.fillScreen(ST77XX_BLACK);
}

void connect_to_obd()
{
  if (demo_mode)
  {
    my_log("Fake connecting to BlueTooth dongle...");
    delay(100);
    my_log("BT connected!");
    delay(100);
    my_log("OBD connected!");
    delay(100);
    my_log("Connected to ELM327");
    connected = true;
    return;
  }

  elmoo.connect(ELM_PORT);
  connected = elmoo.isConnected();

  if (connected){
    elmoo.startFetching();
  } else {
    my_log("failed to connect :<");
  }
  // uint8_t address[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
  // my_log("Connecting to BT via MAC address AA:BB:CC:11:22:33...");

  // if (!ELM_PORT.connect(address))
  // {
  //   my_log("Couldn't connect to BT [MAC]");

  //   my_log("Connecting to BT via name OBDII...");
  //   if (!ELM_PORT.connect("OBDII"))
  //   {
  //     my_log("Couldn't connect to BT [name]]");
  //     return;
  //   }
  // }

  // my_log("OBD connected!");
  // my_log("Connecting to OBD scanner...");

  // if (!myELM327.begin(ELM_PORT, true, 2000))
  // {
  //   my_log("Couldn't connect to OBD scanner");
  //   return;
  // }

  // connected = true;
  // my_log("Connected to ELM327");
}

void read_data()
{
  potValue = analogRead(BRIGHTNESS_PIN);

  if (demo_mode)
  {
    gui_data.rpm = map(potValue, 0, 4095, 0, 6000);
    gui_data.voltage = map(potValue, 0, 4095, MIN_VOLTAGE * 100, MAX_VOLTAGE * 100) / 100.f;
    gui_data.temperature = map(potValue, 0, 4095, 15, MAX_TEMP);
  }
  else
  {
    // float tempRPM = myELM327.rpm();

    // if (myELM327.nb_rx_state == ELM_SUCCESS)
    // {
    //   gui_data.rpm = (uint32_t)tempRPM;
    // }
    // else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
    //   myELM327.printError();

    // // TODO: Read something, check how to request multiple values at once
    // gui_data.voltage = 13.7f;
    // gui_data.temperature = 35;
    elmoo.update();
    if (elmoo.isDataReady())
    {
      gui_data.voltage = elmoo.get_batteryVoltage();
      gui_data.rpm = elmoo.get_rpm();
      gui_data.temperature = elmoo.get_oilTemp();
    }
  }
}

void drawBitmapInvertY(Adafruit_ST7735 &tft,
                       int16_t x,
                       int16_t y,
                       const uint8_t bitmap[],
                       int16_t w,
                       int16_t h,
                       int16_t max_h,
                       uint16_t color)
{
  int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
  uint8_t b = 0;

  tft.startWrite();
  for (int16_t j = max_h - 1; j >= (max_h - h); j--, y--)
  {
    for (int16_t i = 0; i < w; i++)
    {
      if (i & 7)
        b <<= 1;
      else
        b = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
      if (b & 0x80)
        tft.writePixel(x + i, y, color);
    }
  }
  tft.endWrite();
}

void draw_temperature_widget(int16_t x, int16_t y, int16_t value)
{
  int16_t temperature_fullness = static_cast<float>(value) / MAX_TEMP * temp_fill.height;
  tft.drawBitmap(x,
                 y,
                 temp_background.data,
                 temp_background.width,
                 temp_background.height,
                 BMW_ORANGE);
  drawBitmapInvertY(tft,
                    x + 13,
                    y + temp_fill.height,
                    temp_fill.data,
                    temp_fill.width,
                    temperature_fullness,
                    temp_fill.height,
                    BMW_ORANGE);

  tft.setCursor(x + 29, y + 5);
  tft.setTextColor(BMW_ORANGE);
  tft.printf("%u*", value);
}

void draw_accumulator_widget(int16_t x, int16_t y, float value)
{
  // int16_t voltage_fullness = static_cast<float>(value) / 100 * accumulator_fill.height;
  int16_t voltage_fullness = map(value * 100, MIN_VOLTAGE * 100, MAX_VOLTAGE * 100, 0, accumulator_fill.height);
  tft.drawBitmap(x,
                 y,
                 accumulator_background.data,
                 accumulator_background.width,
                 accumulator_background.height,
                 BMW_ORANGE);
  drawBitmapInvertY(tft,
                    x + 30,
                    y + 9 + accumulator_fill.height,
                    accumulator_fill.data,
                    accumulator_fill.width,
                    voltage_fullness,
                    accumulator_fill.height,
                    BMW_ORANGE);

  tft.setCursor(x + 3, y + 13);
  tft.setTextColor(BMW_ORANGE);
  tft.printf("%4.1f", value);
}

void redraw_screen()
{
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);

  if (demo_mode)
  {
    tft.print("DEMO MODE");
  }
  else
  {
    tft.print("LIVE MODE");
  }

  my_log("RPM: ");
  my_log(gui_data.rpm);

  my_log("Battery: ");
  my_log(gui_data.voltage);

  my_log("Temperature: ");
  my_log(gui_data.temperature);

  draw_temperature_widget(8, 110, gui_data.temperature);
  draw_accumulator_widget(75, 122, gui_data.voltage);
}

void setup()
{
  pinMode(BRIGHTNESS_PIN, INPUT);

  DEBUG_PORT.begin(115200);
  ELM_PORT.begin("ESP32test", true);
  SerialBT.setPin("1234");

  setup_screen();
  connect_to_obd();

  delay(1000); // give some time to read messages

  if (!demo_mode && !connected)
  {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 0);
    my_log("Failed to connect to OBD");
    my_log("Halt execution");
    while (1)
    {
    };
  }
}

void loop()
{
  read_data();
  redraw_screen();
  delay(500);
}

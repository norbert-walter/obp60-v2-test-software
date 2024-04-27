/// @file    main.cpp
/// @brief   Test fuctions for multi function display OBP60
/// @example main.cpp

#include <Arduino.h>
#include "driver/twai.h"  // Driver for CAN bus
#include <FastLED.h>      // Driver for WS2812 RGB LED
#include <PCF8574.h>      // Driver for PCF8574 output modul from Horter
#include <Wire.h>         // I2C
#include <RTClib.h>       // Driver for DS1388 RTC
#include <GxEPD2_BW.h>    // E-Ink display

// FreeFonts from Adafruit_GFX
#include "Ubuntu_Bold8pt7b.h"
#include "Ubuntu_Bold12pt7b.h"
#include "Ubuntu_Bold16pt7b.h"
#include "Ubuntu_Bold20pt7b.h"
// OBP logo
#include "Logo_OBP_400x300_sw.h"

// How many leds in your strip?
#define NUM_FLED 1  // Flash LED
#define NUM_BL 6    // Backlight

// #define DATA_PIN
#define DATA_PIN1 7   // Flash LED
#define DATA_PIN2 15  // Backlight

// Horter I2C module
#define DOUT1_I2C 0x20          // First digital output modul PCF8574 from Horter
PCF8574 pcf8574_Out(DOUT1_I2C); // First digital output modul PCF8574 from Horter

// RTC DS1388
RTC_DS1388 ds1388;

// Define the array of leds
CRGB fled[NUM_FLED];      // Flash LED
CRGB backlight[NUM_BL];   // Backlight

// CAN bus definitions
#define RX_PIN 3
#define TX_PIN 46

// E-Ink pin definition
#define OBP_SPI_CS 39     // CS
#define OBP_SPI_DC 40     // DC
#define OBP_SPI_RST 41    // RST
#define OBP_SPI_BUSY 42   // BUSY
#define GxEPD_WIDTH 400   // Display width
#define GxEPD_HEIGHT 300  // Display height

// SPI pin definitions for E-Ink display class
// GxEPD2_BW<GxEPD2_420_GYE042A87, GxEPD2_420_GYE042A87::HEIGHT> display(GxEPD2_420_GYE042A87(OBP_SPI_CS, OBP_SPI_DC, OBP_SPI_RST, OBP_SPI_BUSY)); // GYE042A87, 400x300, SSD1683 (no inking)
GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> display(GxEPD2_420_GDEY042T81(OBP_SPI_CS, OBP_SPI_DC, OBP_SPI_RST, OBP_SPI_BUSY)); // GDEY042T81, 400x300, SSD1683 (no inking)

int i = 0;  // Loop counter

void setup() {

  // Init digital pins
  delay(1000);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);  //Power line on for 5V and 3.3V       
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH); // Set 183DIR on high = transmit
  delay(100);

  // Init RGB LEDs
  FastLED.addLeds<WS2812B, DATA_PIN1, GRB>(fled, NUM_FLED);
  FastLED.addLeds<WS2812B, DATA_PIN2, GRB>(backlight, NUM_BL);

  // Init CAN
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);  // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config  = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  // Init PCF8574 digital outputs
  Wire.setClock(10000UL);   // Set I2C clock on 10 kHz
  if(pcf8574_Out.begin()){  // Initialize PCF8574
    pcf8574_Out.write8(255);// Clear all outputs
  }

  // Init DS1388 RTC
  if(ds1388.begin()){
    uint year = ds1388.now().year();
    if(year < 2023){
      ds1388.adjust(DateTime(__DATE__, __TIME__));  // Set date and time from PC file time
    }
  }

  // Init E-Ink display
  display.init(115200);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  display.fillScreen(GxEPD_WHITE);
  display.nextPage();
  display.drawBitmap(0, 0, gImage_Logo_OBP_400x300_sw, display.width(), display.height(), GxEPD_BLACK);
  display.nextPage();

  // Init serial ports
  Serial.begin(115200);                     // USB serial port
  Serial1.begin(9600, SERIAL_8N1, 2, 1);    // GPS serial port (input)
  Serial2.begin(9600, SERIAL_8N1, 8, 17);   // NMEA0183 serial port (output)

  // Ready to start
  tone(16, 4000); // Buzzer GPIO16 4kHz
  delay(200);     // Duration 200ms
  noTone(16);     // Disable beep
  Serial.println("Boot Beep 4kHz");

  // Scann I2C bus 
  byte error, address;
  int nDevices;
  Serial.println("Scanning I2C bus...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(10000);

}

void touchReadAll(){
  Serial.print("Read Touch:");
  Serial.print(touchRead(14));
  Serial.print(" ");
  Serial.print(touchRead(13));
  Serial.print(" ");
  Serial.print(touchRead(12));
  Serial.print(" ");
  Serial.print(touchRead(11));
  Serial.print(" ");
  Serial.print(touchRead(10));
  Serial.print(" ");
  Serial.println(touchRead(9));
}

int touchRequest(){
  int touchThreshold = 50000;
  int returnCode = 0;
//  Serial.print("ReturnCode:");
  if(touchRead(14) > touchThreshold){
    returnCode += 1;
  }
  if(touchRead(13) > touchThreshold){
    returnCode += 2;
  }
  if(touchRead(12) > touchThreshold){
    returnCode += 4;
  }
  if(touchRead(11) > touchThreshold){
    returnCode += 8;
  }
  if(touchRead(10) > touchThreshold){
    returnCode += 16;
  }
  if(touchRead(9) > touchThreshold){
    returnCode += 32;
  }
//  Serial.println(returnCode);
  return returnCode;
}

void loop() {

  touchReadAll();

  int touchResult = touchRequest();
  FastLED.setBrightness(255);
  if(touchResult == 1){
     fled[0] = CRGB::Red;
     FastLED.show();
  }
  if(touchResult == 2){
     fled[0] = CRGB::Green;
     FastLED.show();
  }
  if(touchResult == 4){
     fled[0] = CRGB::Blue;
     FastLED.show();
  }
  if(touchResult == 8){
     fled[0] = CRGB::White;
     FastLED.show();
  }
  if(touchResult == 16){
    backlight[0] = CRGB::White;
    backlight[1] = CRGB::White;
    backlight[2] = CRGB::White;
    backlight[3] = CRGB::White;
    backlight[4] = CRGB::White;
    backlight[5] = CRGB::White;
    FastLED.setBrightness(255);
    FastLED.show();
  }
  if(touchResult == 32){
    fled[0] = CRGB::Black;
    backlight[0] = CRGB::Black;
    backlight[1] = CRGB::Black;
    backlight[2] = CRGB::Black;
    backlight[3] = CRGB::Black;
    backlight[4] = CRGB::Black;
    backlight[5] = CRGB::Black;
    FastLED.show();
  }

  if(touchResult > 0){
    tone(16, 4000); // Im Hauptteil wird nun mit dem Befehl "tone ( x , y )" ein Ton abgegeben.
    delay(100);
  }
  else{
    noTone(16); // Der Ton wird abgeschaltet
  }

  if (Serial1.available()) {
    char data = Serial1.read();
    Serial.write(data);   // Write USB
    Serial2.write(data);  // Write NMEA0183
    if(data == 0x0A){     // If carridge return
      // Read an show RTC date and time
      Wire.setClock(100000UL);  // Set I2C clock on 10 kHz
      if(ds1388.begin()){       // Check the module is present
        DateTime now = ds1388.now();
        Serial.print(now.year(), DEC);
        Serial.print('/');
        Serial.print(now.month(), DEC);
        Serial.print('/');
        Serial.print(now.day(), DEC);
        Serial.print(' ');
        Serial.print(now.hour(), DEC);
        Serial.print(':');
        Serial.print(now.minute(), DEC);
        Serial.print(':');
        Serial.print(now.second(), DEC);
        Serial.print(' ');
        }
    }
  }
/*
  // Plug & Play safe I2C bus communication
  Wire.setClock(100000UL);  // Set I2C clock on 10 kHz
  if(pcf8574_Out.begin()){  // Check the module is present
    pcf8574_Out.write8(~i); // Loop counter output
    i++;                    // Increment loop counter
  }
*/

  // Receive next CAN frame from queue
  twai_message_t message;
  
  if (twai_receive(&message, 0) == ESP_OK) {
    // Board LED
/*
    boardled[0] = CRGB::Blue;
    FastLED.setBrightness(255);
    FastLED.show();
    delay(1);
    boardled[0] = CRGB::Black;
    FastLED.show();
*/
    // Read Values
    Serial.println();
    Serial.print("0x");
    Serial.print(message.identifier, HEX);
    Serial.print(",");
    Serial.print(message.extd);
    Serial.print(",");
    Serial.print(message.rtr);
    Serial.print(",");
    Serial.print(message.data_length_code);
    
    for(int i=0;i<message.data_length_code;i++) {
      Serial.print(",0x");
      if (message.data[i]<=0x0F) {
        Serial.print(0);
      }
      Serial.print(message.data[i], HEX);
    }
    Serial.println();
  }

}

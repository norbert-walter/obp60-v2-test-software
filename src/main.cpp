/// @file    Blink.ino
/// @brief   Blink the first LED of an LED strip
/// @example Blink.ino

#include <FastLED.h>
#include <PCF8574.h>
#include <Wire.h>

// How many leds in your strip?
#define NUM_FLED 1  // Flash LED
#define NUM_BL 6    // Backlight

// #define DATA_PIN
#define DATA_PIN1 7   // Flash LED
#define DATA_PIN2 15  // Backlight

// Horter I2C module
#define DOUT1_I2C 0x20          // First digital output modul PCF8574
PCF8574 pcf8574_Out(DOUT1_I2C); // First digital output modul PCF8574

// Define the array of leds
CRGB fled[NUM_FLED];
CRGB backlight[NUM_BL];

int i = 0;  // Loop counter

void setup() {
  
  Wire.setClock(10000UL); // Set I2C clock on 10 kHz
  pcf8574_Out.begin();    // Initialize PCF8574
  pcf8574_Out.write8(255);// Clear all outputs

  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);  //Power line on
        
  Serial.begin(115200);       // USB serial port
  Serial1.begin(9600, SERIAL_8N1, 2, 1);  // GPS serial port
  Serial2.begin(9600, SERIAL_8N1, 8, 17);  // NMEA0183 serial port
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);  // Set 183DIR on high = transmit 

  FastLED.addLeds<WS2812B, DATA_PIN1, GRB>(fled, NUM_FLED);
  FastLED.addLeds<WS2812B, DATA_PIN2, GRB>(backlight, NUM_BL);

  //I2C.begin(SDA, SCL, 100000);
  /*
  bool status = bme.begin(0x76, &I2CBME);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
*/
  tone(16, 4000); // Im Hauptteil wird nun mit dem Befehl "tone ( x , y )" ein Ton abgegeben.
  delay(200); // mit einer Dauer von 1 Sekunde
  noTone(16); // Der Ton wird abgeschaltet
  Serial.println("Boot Beep 4kHz");
}

void touchReadAll(){
  Serial.println("Read Touch:");
  Serial.println(touchRead(14));
  Serial.println(touchRead(13));
  Serial.println(touchRead(12));
  Serial.println(touchRead(11));
  Serial.println(touchRead(10));
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

 // touchReadAll();

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
  }
  else{
    noTone(16); // Der Ton wird abgeschaltet
  }

  if (Serial1.available()) {
    char data = Serial1.read();
    Serial.write(data);   // Write USB
    Serial2.write(data);  // Write NMEA0183
  }

  Wire.setClock(10000UL); // Set I2C clock on 10 kHz
  pcf8574_Out.write8(~i); // Loop counter output
  i++;                    // Increment loop counter

}

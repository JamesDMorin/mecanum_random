/***************************************************************************
  This is a library for the AMG88xx GridEYE 8x8 IR camera

  This sketch makes a 64 pixel thermal camera with the GridEYE sensor
  and a 128x128 tft screen https://www.adafruit.com/product/2088

  Designed specifically to work with the Adafruit AMG88 breakout
  ----> http://www.adafruit.com/products/3538

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Dean Miller for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "gripper_controller_pinout.h"
#include "util.h"
#include "wireless.h"

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
extern GripperMessage gripperMessage;


unsigned long delayTime;
short pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint16_t displayPixelWidth, displayPixelHeight;

void thermal_cam_setup() {
  Serial.println(F("AMG88xx thermal camera!"));

  tft.init(240, 320);   // initialize a ST7735S chip, black tab
  tft.fillScreen(ST77XX_BLACK);

  displayPixelWidth = tft.width() / 8;
  displayPixelHeight = tft.height() / 8;

  tft.setRotation(3);
    
  Serial.println("-- Thermal Camera Test --");
  delay(100); // let sensor boot up

}

void thermal_cam_read() {
  //read all the pixels
  Serial.print("[");
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
      pixels[i-1] = gripperMessage.pixels[i-1];
      Serial.print(pixels[i-1]);
      Serial.print(", ");
      if( i%8 == 0 ) Serial.println();
  }
  Serial.println("]");
  

  for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
    uint8_t colorIndex = map(pixels[i], MINTEMP, MAXTEMP, 0, 255);
    colorIndex = (uint8_t)constrain((int16_t)colorIndex, (int16_t)0, (int16_t)255);

    //draw the pixels!
    tft.fillRect(displayPixelHeight * floor(i / 8), displayPixelWidth * (i % 8),
        displayPixelHeight, displayPixelWidth, camColors[colorIndex]);
  }
}
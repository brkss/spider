#include <TFT_eSPI.h>
#include <SPI.h>

// Fixed Pins
#define I2S_DOUT 22
#define I2S_BCLK 26
#define I2S_LRC  25

#define ENC_A 27
#define ENC_B 14
#define ENC_SW 13

#define TFT_CS   5
#define TFT_DC   4
#define TFT_RST  2
#define TFT_SCLK 18
#define TFT_MOSI 23

TFT_eSPI tft = TFT_eSPI();

void setup() {
  Serial.begin(115200);
  Serial.println("Starting TFT Hello World...");
  
  // Initialize TFT display
  tft.begin();
  tft.setRotation(1); // Landscape orientation
  
  // Set display parameters
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(3);
  
  // Center the text
  int16_t x = (tft.width() - tft.textWidth("Hello World")) / 2;
  int16_t y = (tft.height() - 24) / 2; // 24 is approximate height for text size 3
  
  // Print Hello World
  tft.drawString("Hello World", x, y);
  
  Serial.println("Hello World displayed on TFT screen!");
}

void loop() {
  // Empty loop - just display the text
  delay(1000);
}

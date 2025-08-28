/*
  v0_baseline — ESP32-WROOM Pad w/ Volume & TFT (fix1)
  - MAX98357A @ 44.1 kHz (I2S), gentle continuous pad
  - KY-040 encoder: rotate → Volume [0..100], click unused
  - ST7735: shows "VOL xx%" (uses VSPI pins explicitly)
  - DC blocker + soft clip (tanh) on master
  - No dynamic allocation inside audio loop
  - If TFT is still white, change ST7735_TAB to INITR_GREENTAB or INITR_REDTAB.
*/

#include <Arduino.h>
#include <driver/i2s.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// -------- Fixed Pins (exact) --------
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

// Choose your ST7735 variant tab if needed:
#define ST7735_TAB INITR_BLACKTAB
//#define ST7735_TAB INITR_GREENTAB
//#define ST7735_TAB INITR_REDTAB

// -------- Audio constants --------
static const int   SR   = 44100;
static const float TAU  = 6.28318530718f;
static const int   CHUNK = 256;     // samples per channel per write
static const int   DMA_LEN = 256;   // frames per DMA buffer
static const int   DMA_CNT = 8;

// -------- TFT over VSPI (explicit pins to avoid white screen) --------
SPIClass vspi(VSPI);
Adafruit_ST7735 tft(&vspi, TFT_CS, TFT_DC, TFT_RST);

// -------- Encoder state --------
static volatile int32_t encDelta = 0;
static uint8_t encPrev = 0;
static const int8_t ENC_TAB[16] = {
  0, -1,  1, 0,
  1,  0,  0, -1,
 -1,  0,  0, 1,
  0,  1, -1, 0
};

inline void readEncoderStep() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  uint8_t cur = (a << 1) | b;
  uint8_t idx = (encPrev << 2) | cur;
  encDelta += ENC_TAB[idx];
  encPrev = cur;
}
inline int32_t takeEncoderDelta() {
  noInterrupts();
  int32_t d = encDelta;
  encDelta = 0;
  interrupts();
  return d;
}

// -------- DC Blocker --------
struct DCBlock {
  float y1 = 0.0f, x1 = 0.0f;
  float R  = 0.995f; // ~20 Hz HPF at 44.1k
  inline float process(float x) {
    float y = x - x1 + R * y1;
    x1 = x; y1 = y;
    return y;
  }
};

// -------- Pad (3 detuned sines + gentle shimmer) --------
struct Pad {
  float p1=0, p2=0, p3=0;
  float lfoA=0, lfoB=0;
  inline float step() {
    const float base = 140.0f;     // slightly lower for small speaker
    const float det  = 0.011f;     // ±1.1%
    const float vibDepth = 0.22f;  // Hz
    const float vibRate  = 0.18f;  // Hz
    const float ampRate  = 0.07f;  // Hz

    lfoA += TAU * (vibRate / SR); if (lfoA >= TAU) lfoA -= TAU;
    lfoB += TAU * (ampRate / SR); if (lfoB >= TAU) lfoB -= TAU;

    float vib = vibDepth * sinf(lfoA);
    float amp = 0.72f + 0.25f * (0.5f + 0.5f * sinf(lfoB)); // 0.72..0.97

    float f1 = base + vib;
    float f2 = base * (1.0f + det) + vib;
    float f3 = base * (1.0f - det) + vib;

    p1 += TAU * (f1 / SR); if (p1 >= TAU) p1 -= TAU;
    p2 += TAU * (f2 / SR); if (p2 >= TAU) p2 -= TAU;
    p3 += TAU * (f3 / SR); if (p3 >= TAU) p3 -= TAU;

    float s = (sinf(p1) + sinf(p2) + sinf(p3)) * (1.0f/3.0f);
    // low-level 2nd harmonic for body
    s = 0.88f * s + 0.12f * sinf(2.0f * p1);
    return s * amp;
  }
};

// -------- Soft clip (tanh-ish) --------
inline float softClip(float x) {
  const float drive = 1.2f; // a touch more output
  return tanhf(drive * x);
}

// -------- Audio globals --------
static DCBlock dc;
static Pad pad;
static float volTarget = 70.0f; // start louder so you can hear it
static float volSmooth = 0.0f;  // smoothed linear gain
static float fade = 0.0f;       // startup fade-in 0..1
static int16_t buf[CHUNK * 2];  // stereo

// -------- TFT UI --------
static int lastVolShown = -1;

void tftInit() {
  vspi.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS); // SCLK, MISO(-1), MOSI, SS
  delay(50);
  tft.initR(ST7735_TAB);
  delay(120);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
}
void tftShowVolume(int vol) {
  if (vol == lastVolShown) return;
  lastVolShown = vol;
  char line[16];
  snprintf(line, sizeof(line), "VOL %3d%%", vol);
  tft.setTextSize(3);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(line, 0, 0, &x1, &y1, &w, &h);
  int x = (tft.width()  - (int)w) / 2;
  int y = (tft.height() - (int)h) / 2;
  tft.fillRect(x - 4, y - 4, w + 8, h + 8, ST77XX_BLACK);
  tft.setCursor(x, y);
  tft.print(line);
}

// -------- I2S --------
void i2sInit() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SR;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
#if defined(I2S_COMM_FORMAT_STAND_I2S)
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
#else
  cfg.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
#endif
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = DMA_CNT;
  cfg.dma_buf_len = DMA_LEN;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_BCLK;
  pins.ws_io_num = I2S_LRC;
  pins.data_out_num = I2S_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_set_clk(I2S_NUM_0, SR, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

// -------- Setup --------
void setup() {
  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  encPrev = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);

  // TFT
  tftInit();
  tftShowVolume((int)volTarget);

  // I2S
  i2sInit();
}

// Map volume % → perceptual gain; smooth to settle ~1–2 s
inline void updateVolumeSmoothing() {
  float target = volTarget * 0.01f; // 0..1
  target = target * target;         // more resolution at low end
  const float alpha = 1.0f - expf(-(float)CHUNK / (SR * 0.7f)); // tau≈0.7s
  volSmooth += alpha * (target - volSmooth);
}

// -------- Main Loop --------
void loop() {
  // --- Encoder/UI ---
  readEncoderStep();
  int32_t steps = takeEncoderDelta();
  if (steps != 0) {
    // Compress chatter: roughly 2 "logical" steps per detent
    int delta = (steps > 0) ? 1 : -1;
    volTarget += delta;
    if (volTarget < 0)   volTarget = 0;
    if (volTarget > 100) volTarget = 100;
  }
  tftShowVolume((int)volTarget);

  // --- Audio: render & push one CHUNK ---
  updateVolumeSmoothing();
  if (fade < 1.0f) { fade += (float)CHUNK / (SR * 0.8f); if (fade > 1.0f) fade = 1.0f; }

  for (int i = 0; i < CHUNK; ++i) {
    float s = pad.step();                  // raw pad
    float x = s * volSmooth * fade;        // gain + fade
    x = dc.process(x);                     // DC block
    x = softClip(x);                       // clip
    int16_t v = (int16_t)floorf(x * 32767.0f);
    int j = i * 2;
    buf[j + 0] = v;  // L
    buf[j + 1] = v;  // R
  }

  size_t written = 0;
  i2s_write(I2S_NUM_0, (const char*)buf, sizeof(buf), &written, portMAX_DELAY);
}



// ------------------------
// Spider v-0.1
// ------------------------
#include <Arduino.h>
#include <driver/i2s.h>


// -----------------------------
// Fixed Pins (given)
// -----------------------------
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

// ---------------------
// Audio Parameters 
// ---------------------
static const int SAMPLE_RATE = 44100; // 44.1 kHz
static const float TONE_FREQ_HZ = 220.0f; // A4
static const int AMPLITUDE = 4000; // 16-bit peak for the MAX98357A 
// stereo frames : L/R 16 bit little endian 
static const size_t FRAME_PER_CHUNK = 512;

// Sine wave lookup table high quality, fixed point 
static const int TABLE_SIZE = 1024;
int16_t sineLUT[TABLE_SIZE];

// Fixed point phase accumilator 
uint32_t phase = 0;
uint32_t phaseInc = 0;

void initSineTable(){
  for (int i =0; i < TABLE_SIZE; ++i){
    float theta = (2.0f * PI * i) / TABLE_SIZE;
    sineLUT[i] = (int16_t)(sinf(theta) * AMPLITUDE); 
  }
  double inc = (double)TONE_FREQ_HZ * (double)TABLE_SIZE / (double)SAMPLE_RATE;
  phaseInc = (uint32_t)(inc * 65536.0 + 0.5);
}

void i2sInit(){
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // stereo frames
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, // standard I2S
#else
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S),
#endif
    .intr_alloc_flags = 0,
    .dma_buf_count = 6,
    .dma_buf_len = 256, // samples per channel per buffer
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num  = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  // Install and start
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  // Make sure clock is exactly right (some cores require this):
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

void setup() {
  initSineTable();
  i2sInit();
}

void loop() {
  static int16_t buffer[FRAME_PER_CHUNK * 2]; // stereo -> mono
  for (size_t i = 0; i < FRAME_PER_CHUNK ;++i){
    uint16_t idx = (phase >> 16) & (TABLE_SIZE - 1);
    int16_t s = sineLUT[idx];
    phase += phaseInc;

    // duplicate for both chanels MAX98357A expect stereo
    buffer[2*i + 0] = s;
    buffer[2*i + 1] = s;
    size_t byteWritten = 0;
    i2s_write(I2S_NUM_0, (const char*)buffer, sizeof(buffer), &byteWritten, portMAX_DELAY);
  }
}

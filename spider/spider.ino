// ------------------------
// Spider v-0.1
// Features :
// - Musical Clock BPM
// - Sequencer 
// ------------------------
#include <Arduino.h>
#include <driver/i2s.h>

// -----------------------------
// Fixed Pins 
// -----------------------------
#define I2S_DOUT 22
#define I2S_BCLK 26
#define I2S_LRC  25

// ---------------------
// Audio Parameters
// ---------------------
static const int   SAMPLE_RATE = 44100; // 44.1 kHz
static const int   AMPLITUDE   = 8000;
static const size_t FRAMES_PER_CHUNK = 512;

// ---------------------
// Wavetable Sinewave 
// ---------------------
static const int TABLE_SIZE = 1024;
int16_t sineLUT[TABLE_SIZE];

// Phase accumulator
uint32_t phase = 0;
uint32_t phaseInc = 0;

// ========== Helpers: frequency & LUT ==========
static inline float midiToHz(int m) { 
  return 440.0f * powf(2.0f, (m - 69) / 12.0f); 
}

static inline void setToneFreq(float hz) {
  double inc = (double)hz * (double)TABLE_SIZE / (double)SAMPLE_RATE;
  phaseInc = (uint32_t)(inc * 65536.0 + 0.5);
}

void initSineTable() {
  for (int i = 0; i < TABLE_SIZE; ++i) {
    float theta = (2.0f * PI * i) / TABLE_SIZE;
    sineLUT[i] = (int16_t)(sinf(theta) * AMPLITUDE);
  }
}

// ========== Musical clock (BPM) ==========
static float  BPM = 120.0f;     // tempo
static const int PPQ = 24;      // pulses per quarter (classic MIDI)
static double samplesPerTick = 0.0;
static double tickAccumulator = 0.0;

static inline void setBPM(float bpm) {
  BPM = bpm;
  samplesPerTick = (double)SAMPLE_RATE * 60.0 / (double)BPM / (double)PPQ;
}

static inline void clockInit() {
  setBPM(BPM);
  tickAccumulator = 0.0;
}

// ========== 16 step sequencer ==========
struct Step { int midi; bool gate; };

static const int STEPS = 16;
static Step seq[STEPS] = {
  {57,true},{60,true},{64,true},{65,true},
  {67,true},{70,true},{73,true},{75,true},
  {72,true},{70,true},{67,true},{65,true},
  {64,true},{60,true},{58,true},{57,true},
};


static int  currentStep     = 0;
static bool gateActive      = true;     // mutes audio when false
static const int TICKS_PER_STEP = PPQ / 4;  // 16th-notes (24/4 = 6)
static int tickCounter = 0;

static inline void applyStep(int s) {
  gateActive = seq[s].gate;
  if (gateActive) {
    float hz = midiToHz(seq[s].midi);
    setToneFreq(hz);
  } else {
    // keep phase running; just mute output
    gateActive = false;
  }
}

// Advance clock by N rendered samples; fire step changes on grid.
static inline void clockAdvance(size_t samplesJustRendered) {
  tickAccumulator += (double)samplesJustRendered;

  while (tickAccumulator >= samplesPerTick) {
    tickAccumulator -= samplesPerTick;

    if (++tickCounter >= TICKS_PER_STEP) {
      tickCounter = 0;
      currentStep = (currentStep + 1) % STEPS;
      applyStep(currentStep);
    }
  }
}

// ========== I2S ==========
void i2sInit() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S),
#endif
    .intr_alloc_flags = 0,
    .dma_buf_count = 6,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num   = I2S_BCLK,
    .ws_io_num    = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

void setup() {
  initSineTable();
  i2sInit();

  // Start audible immediately (before the first tick):
  applyStep(0);     // sets gate + frequency from step 0
  clockInit();      // derive timing
}

void loop() {
  static int16_t buffer[FRAMES_PER_CHUNK * 2]; // stereo

  // --- render a chunk aka a phase ---
  for (size_t i = 0; i < FRAMES_PER_CHUNK; ++i) {
    uint16_t idx = (phase >> 16) & (TABLE_SIZE - 1);
    int16_t s = sineLUT[idx];
    phase += phaseInc;

    if (!gateActive) s = 0;  // rest
    buffer[2*i + 0] = s;     // L
    buffer[2*i + 1] = s;     // R
  }

  // --- output the whole chunk at once ---
  size_t bytesWritten = 0;
  i2s_write(I2S_NUM_0, (const char*)buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);

  // --- advance tempo clock by the number of frames just rendered ---
  clockAdvance(FRAMES_PER_CHUNK);
}

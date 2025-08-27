// ------------------------
// Spider v-0.1
// Features :
// - Musical Clock BPM
// - Sequencer 
// ------------------------
#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

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

// One-pole low-pass: y += a * (x - y)
struct LP1 { float a = 0.0f; float y = 0.0f; };
// One-pole high-pass via complementary LP
struct HP1 { LP1 lp; };

// Noise 
static uint32_t rngState = 1;
static inline float whiteNoise(){
  rngState ^= rngState << 13;
  rngState ^= rngState << 17;
  rngState ^= rngState << 5;

  // map int32 [-1, 1]
  return (int32_t)rngState * (1.0f / 2147483648.0f); // max int lol 
}



static inline void lp_set(LP1& f, float fc_hz, float sr) {
  f.a = 1.0f - expf(-2.0f * PI * fc_hz / sr);
}

static inline float lp(LP1& f, float x) {
  f.y += f.a * (x - f.y);
  return f.y;
}

static inline void hp_set(HP1& f, float fc_hz, float sr) {
  lp_set(f.lp, fc_hz, sr);
}

static inline float hp(HP1& f, float x) {
  // was: lp(f.lp, x)  -> should call the function you defined: lp_process(...)
  return x - lp(f.lp, x);
}

// ---------------------
// DRUMS 
// ---------------------

struct Kick {
  bool on = false;
  float env = 0.0f, envCoef = 0.0f; // amplitude envolope 
  float phase = 0.0f;
  float freq = 0.0f, freqCoef = 0.0f; // exponential pitch drop
};

struct Snare {
  bool  on = false;
  float nEnv = 0.0f, nCoef = 0.0f;  // noise envelope
  float tEnv = 0.0f, tCoef = 0.0f;  // tone envelope
  float tPhase = 0.0f;
  HP1   hp700;  // remove low boom
  LP1   lp8k;   // tame extreme highs
};

struct Hat {
  bool on = false;
  float env = 0.0f, coef = 0.0f;
  HP1 hp6k, hp10k;   // two cascaded HPs → sizzly top
};

Kick kick;
Snare snr;
Hat hat;


// Call to start a hit
static inline void kick_trigger() {
  kick.on = true;
  kick.env = 1.0f;
  // 120→40 Hz in ~50 ms, amp decays ~150 ms
  kick.freq = 120.0f;
  kick.freqCoef = powf(40.0f / 120.0f, 1.0f / (SAMPLE_RATE * 0.050f));
  kick.envCoef  = expf(-1.0f / (SAMPLE_RATE * 0.150f));
  kick.phase = 0.0f;
}

// Hat 
static inline void hat_init() {
  hp_set(hat.hp6k,  6000.0f, SAMPLE_RATE);
  hp_set(hat.hp10k, 10000.0f, SAMPLE_RATE);
}

static inline void hat_trigger(bool openHat=false) {
  hat.on  = true;
  hat.env = 1.0f;
  // closed ≈ 40 ms, open ≈ 200 ms
  float t = openHat ? 0.200f : 0.040f;
  hat.coef = expf(-1.0f / (SAMPLE_RATE * t));
}

static inline float hat_process() {
  if (!hat.on) return 0.0f;
  float n = whiteNoise();
  n = hp(hat.hp6k, n);
  n = hp(hat.hp10k, n);
  n *= hat.env;
  hat.env *= hat.coef;
  if (hat.env < 0.0003f) hat.on = false;
  return n;
}

// snare 
static inline void snare_init() {
  hp_set(snr.hp700, 700.0f, SAMPLE_RATE);
  lp_set(snr.lp8k,  8000.0f, SAMPLE_RATE);
}

static inline void snare_trigger() {
  snr.on  = true;
  snr.nEnv = 1.0f;  snr.nCoef = expf(-1.0f / (SAMPLE_RATE * 0.180f)); // ~180 ms
  snr.tEnv = 1.0f;  snr.tCoef = expf(-1.0f / (SAMPLE_RATE * 0.060f)); // ~60 ms
  snr.tPhase = 0.0f;
}

static inline float snare_process() {
  if (!snr.on) return 0.0f;

  // Bright-ish noise band (HP 700 Hz, then LP 8 kHz)
  float n = whiteNoise();
  n = hp(snr.hp700, n);
  n = lp(snr.lp8k, n);
  n *= snr.nEnv;
  snr.nEnv *= snr.nCoef;

  // Little body around ~180 Hz
  float tone = sinf(2.0f * PI * snr.tPhase) * snr.tEnv;
  snr.tPhase += 180.0f / (float)SAMPLE_RATE;
  if (snr.tPhase >= 1.0f) snr.tPhase -= 1.0f;
  snr.tEnv *= snr.tCoef;

  float out = 0.8f * n + 0.2f * tone;
  if (snr.nEnv < 0.0004f && snr.tEnv < 0.0004f) snr.on = false;
  return out;
}


// DRUMS pattrens 
bool patKick [16] = {1,0,0,0, 1,0,0,0, 1,0,0,0, 1,0,0,0};   // four-on-the-floor
bool patSnare[16] = {0,0,0,0, 1,0,0,0, 0,0,0,0, 1,0,0,0};   // backbeat on 5 & 13
bool patHat  [16] = {1,0,1,0, 1,0,1,0, 1,0,1,0, 1,0,1,0};   // 8th hats

static inline void triggerDrumsAtStep(int s) {
  if (patKick [s]) kick_trigger();
  if (patSnare[s]) snare_trigger();
  if (patHat[s]) hat_trigger(false); // closed hat
}

// Render one sample (floating, -1..1)
static inline float kick_process() {
  if (!kick.on) return 0.0f;
  float s = sinf(2.0f * PI * kick.phase) * kick.env;

  kick.phase += kick.freq / (float)SAMPLE_RATE;
  if (kick.phase >= 1.0f) kick.phase -= 1.0f;

  kick.freq *= kick.freqCoef;
  kick.env  *= kick.envCoef;

  if (kick.env < 0.0005f) kick.on = false;
  return s;
}

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
      triggerDrumsAtStep(currentStep);
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

  // init filters
  snare_init();
  hat_init();
}

void loop() {
  static int16_t buffer[FRAMES_PER_CHUNK * 2]; // stereo

  // --- render a chunk aka a phase ---
  for (size_t i = 0; i < FRAMES_PER_CHUNK; ++i) {
    uint16_t idx = (phase >> 16) & (TABLE_SIZE - 1);
    float mel = (phaseInc ? (sineLUT[idx] / 32768.0f) : 0.0f); // melody
    phase += phaseInc;

    // drums 
    float dKick  = kick_process();
    float dSnare = snare_process();
    float dHat   = hat_process();
    
    // mix 
    float mix = 0.0f * mel + 0.90f * dKick + 0.00f * dSnare + 0.60f * dHat;
    // safety convertion 
    if (mix >  1.0f) mix = 1.0f;
    if (mix < -1.0f) mix = -1.0f;
    int16_t s16 = (int16_t)(mix * 30000.0f);  // master level


    
    buffer[2*i + 0] = s16;     // L
    buffer[2*i + 1] = s16;     // R
  }

  // --- output the whole chunk at once ---
  size_t bytesWritten = 0;
  i2s_write(I2S_NUM_0, (const char*)buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);

  // --- advance tempo clock by the number of frames just rendered ---
  clockAdvance(FRAMES_PER_CHUNK);
}

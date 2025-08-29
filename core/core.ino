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

enum Page { PAGE_VOLUME = 0, PAGE_TEMPO, PAGE_COUNT };
struct Params {
  int volumePercent; // 0..100
  int tempoBPM;      // 30..240 (clamped)
};

// Sine/Triangle blend oscillator
struct Osc {
  float phase = 0.0f;
  float freqHz = 0.0f;
  float triMix = 0.30f; // 0..1: 0=sine, 1=triangle
};

// One-pole low-pass: y += a*(x - y), a from cutoff
struct OnePoleLP {
  float a = 0.0f;
  float y = 0.0f;
};

// Gentle pad: 3 detuned osc → amp-LFO → LPF
struct Pad {
  Osc v1, v2, v3;
  float baseHz = 160.0f;
  float det = 0.012f;        // ±1.2%
  float triMix = 0.30f;      // sine/triangle blend
  // Amp LFO
  float ampLfoPhase = 0.0f;
  float ampLfoRateHz = 0.08f;
  float ampLfoDepth = 0.25f; // 0..1
  // LPF
  OnePoleLP lp;
  float lpfCutHz = 1800.0f;
  bool inited = false;
};

// --- [NEW] Musical clock types/state (place with other structs/globals) ---
struct Clock16 {
  uint64_t smpNow = 0;          // running sample counter
  uint64_t nextTickAt = 0;      // absolute sample index of next 1/16 tick
  uint32_t samplesPerTick = 0;  // samples per 1/16 note
  uint16_t step16 = 0;          // 0..15
  uint16_t bpm = 104;           // cached tempo
};

static Clock16 gClock;

// --- [NEW] Scales: enums, defs, and table ---
enum Scale : uint8_t { SCALE_A_MINOR = 0, SCALE_D_DORIAN, SCALE_C_LYDIAN, SCALE_COUNT };

struct ScaleDef {
  uint8_t rootPC;        // 0..11 (C=0, C#=1, ... A=9, B=11)
  uint8_t degrees[7];    // diatonic degrees as pitch-class offsets from root
  uint8_t count;         // always 7 here
};

static const ScaleDef SCALES[(int)SCALE_COUNT] = {
  /* A minor (Aeolian) */ { 9, {0,2,3,5,7,8,10}, 7 },
  /* D dorian          */ { 2, {0,2,3,5,7,9,10}, 7 },
  /* C lydian          */ { 0, {0,2,4,6,7,9,11}, 7 }
};

inline const ScaleDef& scaleDef(Scale s) { return SCALES[(int)s]; }

// -------- Audio constants --------
static const int   SR   = 44100;
static const float TAU  = 6.28318530718f;
static const int   CHUNK = 256;     // samples per channel per write
static const int   DMA_LEN = 256;   // frames per DMA buffer
static const int   DMA_CNT = 8;

// -------- TFT over VSPI (explicit pins to avoid white screen) --------
SPIClass vspi(VSPI);
Adafruit_ST7735 tft(&vspi, TFT_CS, TFT_DC, TFT_RST);

// --- Globals for Page/UI/Button ---
static Page currentPage = PAGE_VOLUME;
static Params params = { 70, 100 };      // defaults
static volatile bool btnRaw = false;     // set in ISR
static unsigned long btnLastMs = 0;      // debounce
static int lastPageShown = -1;           // UI cache
static int lastValShown  = -1;

// --- [NEW] Button ISR & Debounce ---
void IRAM_ATTR onEncButton() { btnRaw = true; }

// -------- Encoder state --------
static volatile int32_t encDelta = 0;
static uint8_t encPrev = 0;
static const int8_t ENC_TAB[16] = {
  0, -1,  1, 0,
  1,  0,  0, -1,
 -1,  0,  0, 1,
  0,  1, -1, 0
};


// call in loop; returns true once per valid click
bool consumeButtonClick() {
  if (!btnRaw) return false;
  btnRaw = false;
  unsigned long now = millis();
  if (now - btnLastMs < 180) return false; // debounce
  btnLastMs = now;
  return true;
}

inline const char* pageName(Page p){
  switch(p){
    case PAGE_VOLUME: return "Volume";
    case PAGE_TEMPO:  return "Tempo";
    default:          return "?";
  }
}

void nextPage(){
  currentPage = (Page)((currentPage + 1) % PAGE_COUNT);
  lastPageShown = -1; // force UI refresh
  lastValShown  = -1;
}

// --- [NEW] Helpers: clamping & page cycling ---
inline int clampVolume(int v){ if(v<0) return 0; if(v>100) return 100; return v; }
inline int clampTempo (int b){ if(b<30) return 30; if(b>240) return 240; return b; }

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
// --- [REPLACED] Oscillator, LPF helpers, and Pad synth ---



inline float osc_sine_tri(Osc& o, float sr) {
  // advance
  o.phase += (TAU * (o.freqHz / sr));
  if (o.phase >= TAU) o.phase -= TAU;

  // components
  float s = sinf(o.phase);

  float u = o.phase / TAU;              // 0..1
  u = u - floorf(u);
  float tri = 4.0f * fabsf(u - 0.5f) - 1.0f; // -1..1

  float mix = o.triMix;
  return (1.0f - mix) * s + mix * tri;
}

inline void lp_set(OnePoleLP& f, float cutoffHz, float sr) {
  f.a = 1.0f - expf(-(TAU * cutoffHz) / sr);
}
inline float lp_process(OnePoleLP& f, float x) {
  f.y += f.a * (x - f.y);
  return f.y;
}



// --- [NEW] Clock helpers & tick stub ---
inline uint32_t clock_spt_from_bpm(int bpm){
  if (bpm < 30) bpm = 30;
  return (uint32_t)((SR * 60.0f) / (bpm * 4.0f) + 0.5f); // 1/16 = quarter/4
}

inline void clock_init(Clock16& c, int bpm){
  c.bpm = (uint16_t)bpm;
  c.samplesPerTick = clock_spt_from_bpm(bpm);
  c.smpNow = 0;
  c.nextTickAt = c.samplesPerTick;
  c.step16 = 0;
}

inline void clock_update_tempo_if_changed(Clock16& c, int bpm){
  if (bpm != (int)c.bpm){
    c.bpm = (uint16_t)bpm;
    c.samplesPerTick = clock_spt_from_bpm(bpm);
    c.nextTickAt = c.smpNow + c.samplesPerTick; // re-arm from "now"
  }
}

// Called on every 1/16 step (no audible click; default no-op)
inline void onStep16(uint16_t step){
  (void)step; // placeholder for future actions
}

inline void clock_advance_block(Clock16& c, uint32_t blockSamples){
  uint64_t blockEnd = c.smpNow + blockSamples;
  while (c.nextTickAt <= blockEnd){
    onStep16(c.step16);
    c.step16 = (c.step16 + 1u) & 0x0Fu;
    c.nextTickAt += c.samplesPerTick;
  }
  c.smpNow = blockEnd;
}

inline int quantizeMidi(int midi, Scale s) {
  const ScaleDef& sd = scaleDef(s);
  auto pc12 = [](int m){ int x = m % 12; return (x < 0) ? x + 12 : x; };

  const int inPC = pc12(midi);
  int bestDiff = 127;      // signed semitone diff to add to midi
  int bestAbs  = 128;      // |diff|
  for (int i = 0; i < sd.count; ++i) {
    int targetPC = (sd.rootPC + sd.degrees[i]) % 12;
    int d = targetPC - inPC;              // -11..+11 after wrapping:
    if (d > 6)  d -= 12;
    if (d < -6) d += 12;
    int ad = (d >= 0) ? d : -d;
    // Tie-break upward (prefer >=)
    if (ad < bestAbs || (ad == bestAbs && d > bestDiff)) {
      bestAbs = ad;
      bestDiff = d;
    }
  }
  int out = midi + bestDiff;
  if (out < 0) out = 0; if (out > 127) out = 127;
  return out;
}

inline int pickAmbientNote(Scale s, int centerMidi) {
  const ScaleDef& sd = scaleDef(s);
  auto pc12 = [](int m){ int x = m % 12; return (x < 0) ? x + 12 : x; };

  int base = quantizeMidi(centerMidi, s);

  // Small diatonic step pattern (center-biased, deterministic cycle)
  static const int OFFS[] = { 0, 1, -1, 2, -2, 3, -3, 4, -4, 1, 0, -1 };
  static uint8_t idx = 0;
  int steps = OFFS[idx++ % (sizeof(OFFS)/sizeof(OFFS[0]))];

  auto nextUpDelta = [&](int currentPC)->int {
    int best = 12; // minimal positive semitones to next allowed PC
    for (int i = 0; i < sd.count; ++i) {
      int pc = (sd.rootPC + sd.degrees[i]) % 12;
      int d = (pc - currentPC + 12) % 12;
      if (d == 0) d = 12;       // same PC => go to next octave
      if (d < best) best = d;
    }
    return best; // 1..12
  };
  auto prevDownDelta = [&](int currentPC)->int {
    int best = 12; // minimal positive semitones to previous allowed PC
    for (int i = 0; i < sd.count; ++i) {
      int pc = (sd.rootPC + sd.degrees[i]) % 12;
      int d = (currentPC - pc + 12) % 12;
      if (d == 0) d = 12;       // same PC => go to previous octave
      if (d < best) best = d;
    }
    return best; // 1..12 (to step downward)
  };

  int out = base;
  if (steps > 0) {
    for (int k = 0; k < steps; ++k) {
      int curPC = pc12(out);
      out += nextUpDelta(curPC);
    }
  } else if (steps < 0) {
    for (int k = 0; k < -steps; ++k) {
      int curPC = pc12(out);
      out -= prevDownDelta(curPC);
    }
  }

  // Optional gentle clamp to a practical range (leave plenty of room)
  if (out < 24) out = 24;      // C1
  if (out > 108) out = 108;    // C8-

  return out;
}


inline void pad_init(Pad& p, float sr) {
  p.v1.freqHz = p.baseHz;
  p.v2.freqHz = p.baseHz * (1.0f + p.det);
  p.v3.freqHz = p.baseHz * (1.0f - p.det);
  p.v1.triMix = p.v2.triMix = p.v3.triMix = p.triMix;
  lp_set(p.lp, p.lpfCutHz, sr);
  p.inited = true;
}

inline float pad_step(Pad& p, float sr) {
  if (!p.inited) pad_init(p, sr);

  // slow amp LFO
  p.ampLfoPhase += TAU * (p.ampLfoRateHz / sr);
  if (p.ampLfoPhase >= TAU) p.ampLfoPhase -= TAU;
  float lfo = 0.5f + 0.5f * sinf(p.ampLfoPhase);                 // 0..1
  float amp = (1.0f - p.ampLfoDepth) + p.ampLfoDepth * lfo;      // 1-depth .. 1

  // 3-voice detuned pad
  float s = (osc_sine_tri(p.v1, sr) + osc_sine_tri(p.v2, sr) + osc_sine_tri(p.v3, sr)) * (1.0f / 3.0f);
  s *= amp;

  // gentle LPF
  s = lp_process(p.lp, s);
  return s;
}
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

// --- Apply encoder rotation to active page ---
void applyEncoderDelta(int steps){
  if (steps == 0) return;
  int delta = (steps > 0) ? 1 : -1;   // compress chatter
  if (currentPage == PAGE_VOLUME) {
    params.volumePercent = clampVolume(params.volumePercent + delta);
    // route to existing volume smoothing path
    volTarget = (float)params.volumePercent;
  } else { // PAGE_TEMPO
    params.tempoBPM = clampTempo(params.tempoBPM + delta);
  }
}


// --- UI draw for flat pages ---
void uiMaybeRedraw(){
  int shownVal = (currentPage == PAGE_VOLUME) ? params.volumePercent : params.tempoBPM;
  if (lastPageShown == (int)currentPage && lastValShown == shownVal) return;

  char line[32];
  snprintf(line, sizeof(line), "PAGE: %s VAL: %d", pageName(currentPage), shownVal);

  tft.setTextSize(2);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(line, 0, 0, &x1, &y1, &w, &h);
  int x = (tft.width()  - (int)w) / 2;
  int y = (tft.height() - (int)h) / 2;

  tft.fillRect(x - 6, y - 6, w + 12, h + 12, ST77XX_BLACK);
  tft.setCursor(x, y);
  tft.print(line);

  lastPageShown = (int)currentPage;
  lastValShown  = shownVal;
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
  attachInterrupt(digitalPinToInterrupt(ENC_SW), onEncButton, FALLING);
  currentPage = PAGE_VOLUME;
  params.volumePercent = 70;  // matches your default
  params.tempoBPM = 100;
  volTarget = (float)params.volumePercent; // ensure audio path sees it
  lastPageShown = -1; lastValShown = -1;   // force first draw
  encPrev = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);

  // TFT
  tftInit();
  tftShowVolume((int)volTarget);

  // musical clock init 
  params.tempoBPM = 104;                 // default tempo per request
  clock_init(gClock, params.tempoBPM);   // start the 1/16 clock

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
  applyEncoderDelta(steps);

  if (consumeButtonClick()) {
    nextPage();
  }
  uiMaybeRedraw();
  tftShowVolume((int)volTarget);

  // --- Audio: render & push one CHUNK ---
  updateVolumeSmoothing();
  if (fade < 1.0f) { fade += (float)CHUNK / (SR * 0.8f); if (fade > 1.0f) fade = 1.0f; }

  clock_update_tempo_if_changed(gClock, params.tempoBPM);

  for (int i = 0; i < CHUNK; ++i) {
    float s = pad_step(pad, SR);          // <— new gentle pad source

    float x = s * volSmooth * fade;       // gain + fade
    x = dc.process(x);                    // DC block
    x = softClip(x);                      // soft clip

    int16_t v = (int16_t)floorf(x * 32767.0f);
    int j = i * 2;
    buf[j + 0] = v;  // L
    buf[j + 1] = v;  // R
  }

  size_t written = 0;
  i2s_write(I2S_NUM_0, (const char*)buf, sizeof(buf), &written, portMAX_DELAY);
  clock_advance_block(gClock, (uint32_t)CHUNK);
}

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

#include "esp_system.h"


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



// --- [NEW] Master makeup gain to boost overall loudness ---
static const float MASTER_GAIN = 1.35f;  // ~+2.6 dB headroom into soft clip

enum DelayMode : uint8_t { DELAY_1_8 = 0, DELAY_1_4 };
enum Scale : uint8_t { SCALE_C_MINOR = 0, SCALE_A_MINOR, SCALE_D_DORIAN, SCALE_C_LYDIAN, SCALE_COUNT };
enum Page { PAGE_VOLUME = 0, PAGE_TEMPO, PAGE_SPACE, PAGE_MOOD, PAGE_TONE, PAGE_ENERGY, PAGE_SCENE, PAGE_COUNT };

static inline int pickAmbientNote(Scale s, int centerMidi);

struct Params {
  int volumePercent; // 0..100
  int tempoBPM;      // 30..240 (clamped)
  int spacePercent;  // 0..100  (delay send/mix)
  int moodPercent;   // 0..100
  int tonePercent;   // 0..100
  int energyPercent; // 0..100
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
  float baseHz = 180.0f;
  float det = 0.0f;        // ±1.2%
  float triMix = 0.18f;      // sine/triangle blend
  // Amp LFO
  float ampLfoPhase = 0.0f;
  float ampLfoRateHz = 0.05f;
  float ampLfoDepth = 0.18f; // 0..1
  // LPF
  OnePoleLP lp;
  float lpfCutHz = 1200.0f;
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

// --- Pink noise (Paul Kellet approx) + band limit (HP ~150 Hz, LP ~5 kHz)
struct PinkNoise {
  float b0=0, b1=0, b2=0;
  OnePoleLP hpSplit;   // used to make a 1-pole high-pass: x - lp(x)
  OnePoleLP lpAir;     // top-end low-pass (air band)
  bool inited=false;
};
static PinkNoise gPink;
static const float NOISE_DRY_GAIN = 0.0f;   // keep 0.0 for FX-only
static const float NOISE_FX_GAIN  = 0.12f;  // try 0.08–0.20



// --- Drone pitch random-walk state ---
static float pitchTargetHz = 160.0f;     // target frequency
static float pitchSmoothHz = 160.0f;     // smoothed frequency
static uint32_t nextPitchChange = 0;     // millis() when to step next

#if defined(DELAY_MAX_SAMPLES)
  #undef DELAY_MAX_SAMPLES
#endif
#define DELAY_MAX_SAMPLES 32768  // ~0.74s @ 44.1k (1/4 note ≥ ~81 BPM), ≈64 KB

struct Delay {
  int16_t buf[DELAY_MAX_SAMPLES];
  uint32_t size = DELAY_MAX_SAMPLES, w = 0, dSamp = 22050;
  OnePoleLP tone;
  float baseSend = 0.0f, baseWet = 0.0f;  // <— NEW: remember base amounts
  float send = 0.0f,  wet  = 0.0f;
  float fb = 0.0f;
  uint16_t bpmCached = 104;
  DelayMode mode = DELAY_1_8;
  bool inited = false;
};

static Delay gDelay;

// Tone / Mood 
struct TiltEQ {
  OnePoleLP split6k;
  float gainLow = 1.0f, gainHigh = 1.0f;
  bool inited = false;
};
static TiltEQ gTilt;

// --- Sceen Cross Fade
struct SceneTargets {
  Scale     scale;
  float     triMix;
  float     lpfCutHz;
  int       spacePercent;   // 0..100
  DelayMode delayMode;      // DELAY_1_8 / DELAY_1_4
  int       tempoBPM;       // 30..240
};

struct SceneXfade {
  bool      active = false;
  uint64_t  tStartSmp = 0, tEndSmp = 0;  // absolute sample times
  SceneTargets start, target;
};

struct SceneScheduler {
  uint64_t nextSceneAt = 0;
  uint32_t minIntvSmp = 0;  // 120s
  uint32_t maxIntvSmp = 0;  // 240s
  bool     inited = false;
};

static SceneXfade     gScene;
static SceneScheduler gSceneSch;

// Explicit prototypes so Arduino's auto-prototyper doesn't mangle them
static inline void  tone_init(TiltEQ& t);
static inline void  tone_set(TiltEQ& t, int tonePct);
static inline float tone_process(TiltEQ& t, float x);

// Energy 
// --- [NEW] Energy mapping state + prototypes (place near other types/globals) ---
struct EnergyCtrl {
  // note trigger probabilities (per 1/16)
  float pStrong = 0.40f;
  float pEven   = 0.22f;
  float pOdd    = 0.08f;
  // pad movement
  float ampLfoRateHz = 0.08f;
  float ampLfoDepth  = 0.25f;
  // gesture Poisson
  float    gestureLambdaHz = 1.0f / 12.0f; // avg 12s
  uint64_t nextGestureAt   = 0;
};
static EnergyCtrl gEnergy;



// Prototypes (Arduino auto-proto can miss these)
static inline uint32_t samplesFromExp(float lambdaHz);
static inline void     energy_set(EnergyCtrl& e, Pad& pad, int energyPct);
static inline void     maybeTriggerGesture(EnergyCtrl& e, uint64_t smpNow);

// --- [NEW] Light Reverb (4 combs + 2 allpasses) ---
#define RV_COMB1 1117
#define RV_COMB2 1189
#define RV_COMB3 1277
#define RV_COMB4 1361
#define RV_AP1    225
#define RV_AP2    341

static int16_t rvCombBuf1[RV_COMB1];
static int16_t rvCombBuf2[RV_COMB2];
static int16_t rvCombBuf3[RV_COMB3];
static int16_t rvCombBuf4[RV_COMB4];
static int16_t rvApBuf1[RV_AP1];
static int16_t rvApBuf2[RV_AP2];

struct CombLite {
  int16_t* buf = nullptr; uint16_t size = 0, w = 0;
  float fb = 0.62f, damp = 0.22f, filt = 0.0f; // LP in feedback path
};
struct AllpassLite {
  int16_t* buf = nullptr; uint16_t size = 0, w = 0;
  float g = 0.5f;
};
struct ReverbLite {
  CombLite c[4];
  AllpassLite ap1, ap2;
  float baseSend = 0.0f, baseWet = 0.0f; // <— NEW
  float send = 0.0f,  wet  = 0.0f;
  bool inited = false;
};
static ReverbLite gReverb;

// --- [NEW] Lightweight Pluck: voice/pool & tiny RNG (place with other types/globals) ---
enum EnvState : uint8_t { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };

struct ADSR {
  float a = 0.003f, d = 0.080f, s = 0.00f, r = 0.060f;
  float env=0.0f;
  EnvState st=ENV_IDLE;
  bool gate=false;
};

struct PluckVoice {
  bool  active=false;
  int   midi=60;
  float freqHz=261.63f;
  float phase=0.0f;
  float vel=0.7f;
  ADSR  eg;
  float pan=0.0f;   // NEW: -1..+1 (L..R), set when note starts
};

struct PluckPool {
  PluckVoice v[4];
  float gain=0.35f;   // headroom vs pad
  bool  inited=false;
};

static PluckPool gPluck;
static Scale gScale = SCALE_A_MINOR;      // use your quantizer scales
static uint32_t gRng32 = 0xA3C59AC3u;     // tiny xorshift RNG state


// Gestures
// --- [NEW] Gesture controller (envelopes + overlay) ---
struct GestureCtrl {
  // scheduling
  float lambdaHz = 1.0f/15.0f;  // modulated by Energy each block
  uint64_t nextAt = 0;

  // shape
  bool active = false;
  uint64_t tStart = 0, tAtkEnd = 0, tRelStart = 0, tEnd = 0;
  float env = 0.0f;             // smoothed 0..1
  float cutBoostOct = 0.5f;     // 0.25..0.80
  float fxSendBoost = 0.5f;     // 0.30..0.70
  float fxWetBoost  = 0.35f;    // 0.20..0.50
};
static GestureCtrl gGesture;

// --- [NEW] Scales: enums, defs, and table ---
struct ScaleDef {
  uint8_t rootPC;        // 0..11 (C=0, C#=1, ... A=9, B=11)
  uint8_t degrees[7];    // diatonic degrees as pitch-class offsets from root
  uint8_t count;         // always 7 here
};

static const ScaleDef SCALES[(int)SCALE_COUNT] = {
  /* C minor           */ { 0, {0,2,4,6,7,9,11}, 7 },
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
    case PAGE_SPACE:  return "Space";
    case PAGE_MOOD:   return "Mood";
    case PAGE_TONE:   return "Tone";
    case PAGE_ENERGY: return "Energy";
    case PAGE_SCENE:  return "Scene";   // <—
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
  if (!gPluck.inited) pluck_init(gPluck);
  // vary probabilities every bar (16 steps)
  static uint16_t lastBar = 0;
  uint16_t bar = step / 16;
  static float wob = 1.0f;
  if (bar != lastBar) {
    lastBar = bar;
    // wobble 0.9..1.1
    wob = 0.9f + 0.2f * rand01();
  }


  float p = ((step & 3u) == 0u) ? gEnergy.pStrong * wob
         : ((step & 1u) == 0u) ? gEnergy.pEven   * wob
         :                       gEnergy.pOdd    * wob;

  p = constrain(p, 0.02f, 0.95f);

  if (rand01() < p){
    int center = 60; // C4 center
    int note = pickAmbientNote(gScale, center);
    float vel = 0.45f + 0.50f * rand01();
    pluck_noteOn(gPluck, note, vel);
  }
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

inline int pickAmbientNote(Scale s, int /*centerMidi*/) {
  static int last = 60;               // remember only to avoid tiny steps
  const ScaleDef& sd = scaleDef(s);

  // Build a small pool of allowed MIDI notes in a comfy register (C2..C7)
  // Root is the scale root; degrees are added across several octaves.
  int pool[64]; int n = 0;
  for (int octave = 2; octave <= 6; ++octave) {          // 2..6 => 36..96
    int base = octave * 12 + sd.rootPC;                  // octave root
    for (int i = 0; i < sd.count; ++i) {
      int m = base + sd.degrees[i];
      if (m >= 36 && m <= 96) pool[n++] = m;
    }
  }

  // Pick a random candidate; avoid tiny stepwise motions
  int cand = pool[rng32() % n];

  // 10% chance to HOLD (modular "sample & hold" feel)
  if (rand01() < 0.10f) return last;

  // 15% chance to force a wide modular interval (±5 or ±7 semitones)
  if (rand01() < 0.15f) {
    cand += (rng32() & 1) ? +7 : -5;
  }

  // If too close to last (|Δ| < 3 semitones), push to a non-adjacent interval
  if (abs(cand - last) < 3) {
    cand += (rng32() & 1) ? +7 : -5;   // perfect 5th / 4th flavor
  }

  // Keep within range and re-quantize to scale
  if (cand < 36) cand = 36;
  if (cand > 96) cand = 96;
  cand = quantizeMidi(cand, s);

  last = cand;
  return cand;
}

// --- [NEW] Pluck helpers: RNG, ADSR, noteOn, render ---
inline uint32_t rng32(){ uint32_t x=gRng32; x^=x<<13; x^=x>>17; x^=x<<5; return gRng32=x; }
inline float rand01(){ return (rng32() * (1.0f/4294967296.0f)); } // [0,1)

inline float midiToHz(int m){ return 440.0f * powf(2.0f, (m - 69) * (1.0f/12.0f)); }

inline float adsr_step(ADSR& e, float sr){
  switch(e.st){
    case ENV_ATTACK: {
      float inc = (e.a <= 0.0f) ? 1.0f : 1.0f/(e.a*sr);
      e.env += inc;
      if (e.env >= 1.0f){ e.env = 1.0f; e.st = ENV_DECAY; }
    } break;
    case ENV_DECAY: {
      float dec = (e.d <= 0.0f) ? 1.0f : 1.0f/(e.d*sr);
      e.env -= dec*(1.0f - e.s);
      if (e.env <= e.s){ e.env = e.s; e.st = ENV_SUSTAIN; }
    } break;
    case ENV_SUSTAIN:
      // For pluck, immediately head to release if gate not held
      if (!e.gate) e.st = ENV_RELEASE;
      break;
    case ENV_RELEASE: {
      float dec = (e.r <= 0.0f) ? 1.0f : 1.0f/(e.r*sr);
      e.env -= dec*e.env;
      if (e.env <= 0.0008f){ e.env = 0.0f; e.st = ENV_IDLE; }
    } break;
    default: e.env = 0.0f; break;
  }
  return e.env;
}

inline void pluck_init(PluckPool& p){
  p.gain = 0.35f;
  for (int i=0;i<4;++i){ p.v[i] = PluckVoice(); }
  p.inited = true;
}

inline void pluck_noteOn(PluckPool& p, int midi, float vel){
  if (!p.inited) pluck_init(p);

  // voice allocate: free first, else quietest env
  int idx = -1; float minEnv = 1e9f;
  for (int i=0;i<4;++i){ if (!p.v[i].active){ idx=i; break; } }
  if (idx < 0){
    for (int i=0;i<4;++i){
      float e = p.v[i].eg.env;
      if (e < minEnv){ minEnv = e; idx = i; }
    }
  }

  PluckVoice& v = p.v[idx];
  // quantize to current scale
  int q = quantizeMidi(midi, gScale);
  v.midi   = q;
  v.freqHz = midiToHz(q);
  v.phase  = 0.0f;
  v.vel    = constrain(vel, 0.0f, 1.0f);
  v.eg.env = 0.0f;
  v.eg.st  = ENV_ATTACK;
  v.eg.gate= false;          // pluck: short gate
  v.active = true;
}

inline float pluck_step(PluckPool& p, float sr){
  if (!p.inited) return 0.0f;
  float sum = 0.0f;
  for (int i=0;i<4;++i){
    PluckVoice& v = p.v[i];
    if (!v.active) continue;

    float env = adsr_step(v.eg, sr);
    if (v.eg.st == ENV_IDLE){ v.active=false; continue; }

    // ultra-light osc (sine)
    v.phase += (TAU * (v.freqHz / sr));
    if (v.phase >= TAU) v.phase -= TAU;
    float s = sinf(v.phase);

    sum += s * env * v.vel;
  }
  return sum * p.gain;
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

  // pre-filter soft saturation (tames spiky highs without harshness)
  s = tanhf(0.6f * s);



  // gentle LPF
  s = lp_process(p.lp, s);
  return s;
}

// --- [NEW] Tone EQ (tilt around ~6 kHz) ---


inline void tone_init(TiltEQ& t){
  lp_set(t.split6k, 6000.0f, SR); // split at ~6 kHz
  t.gainLow = 1.0f; t.gainHigh = 1.0f;
  t.inited = true;
}

// tonePercent 0..100 → tilt -3..+3 dB (complementary low/high)
inline void tone_set(TiltEQ& t, int tonePct){
  if (!t.inited) tone_init(t);
  if (tonePct < 0) tonePct = 0; if (tonePct > 100) tonePct = 100;
  float u = tonePct * (1.0f/100.0f);         // 0..1
  float tiltDb = 6.0f*(u - 0.5f);            // -3..+3
  float gH = powf(10.0f, tiltDb/20.0f);      // linear
  float gL = 1.0f / gH;                      // complementary
  // keep within safe bounds to avoid harshness
  if (gH > 1.5f) gH = 1.5f; if (gH < 0.67f) gH = 0.67f;
  if (gL > 1.5f) gL = 1.5f; if (gL < 0.67f) gL = 0.67f;
  t.gainHigh = gH; t.gainLow = gL;
}

inline float tone_process(TiltEQ& t, float x){
  if (!t.inited) tone_init(t);
  float low = lp_process(t.split6k, x);
  float high = x - low;
  return t.gainLow*low + t.gainHigh*high;
}

// Energy 

// --- [NEW] Energy mapping helpers ---
inline uint32_t samplesFromExp(float lambdaHz){
  if (lambdaHz <= 0.0f) lambdaHz = 1e-6f;
  float r = 1.0f - rand01(); if (r < 1e-6f) r = 1e-6f;
  float sec = -logf(r) / lambdaHz;
  uint32_t s = (uint32_t)(sec * SR + 0.5f);
  if (s < 1u) s = 1u;
  return s;
}

inline void energy_set(EnergyCtrl& e, Pad& pad, int energyPct){
  if (energyPct < 0) energyPct = 0; if (energyPct > 100) energyPct = 100;
  float u  = energyPct * (1.0f/100.0f);
  float u1 = sqrtf(u); // musical curve

  // Scheduler probabilities (clamped)
  e.pStrong = constrain(0.40f * (0.7f + 1.2f * u1), 0.10f, 0.85f);
  e.pEven   = constrain(0.22f * (0.7f + 1.2f * u1), 0.05f, 0.55f);
  e.pOdd    = constrain(0.08f * (0.7f + 1.2f * u1), 0.02f, 0.30f);

  // Pad movement
  e.ampLfoRateHz = 0.04f + (0.18f - 0.04f) * u1;  // 0.04..0.18 Hz
  e.ampLfoDepth  = 0.15f + (0.32f - 0.15f) * u1;  // 0.15..0.32
  pad.ampLfoRateHz = e.ampLfoRateHz;
  pad.ampLfoDepth  = e.ampLfoDepth;

  // Gesture Poisson rate (≈ every 30s → 6s)
  e.gestureLambdaHz = (1.0f/30.0f) + ((1.0f/6.0f) - (1.0f/30.0f)) * u1;
  // Initialize next event if not scheduled yet
  if (e.nextGestureAt == 0) {
    // gClock may not be visible here; will init on first maybeTriggerGesture() call
  }
}

inline void maybeTriggerGesture(EnergyCtrl& e, uint64_t smpNow){
  if (e.nextGestureAt == 0) {
    e.nextGestureAt = smpNow + samplesFromExp(e.gestureLambdaHz);
    return;
  }
  if (smpNow >= e.nextGestureAt){
    // (stub) future: brief brighten/swell gesture tied to pad/FX
    e.nextGestureAt = smpNow + samplesFromExp(e.gestureLambdaHz);
  }
}

// MOOD 
inline void mood_set(Pad& pad, ReverbLite& rv, int moodPct){
  if (moodPct < 0) moodPct = 0; if (moodPct > 100) moodPct = 100;
  float k = moodPct * (1.0f/100.0f);   // 0..1

  // LPF cutoff: log map 400 → 8000 Hz
  // darker & smoother: 300 → ~5 kHz max
  float cut= 300.0f * powf(16.0f, k); // caps ~4800 Hz
  pad.triMix = 0.08f + 0.37f * k;      // 0.08..0.45 (stays mostly sine)
  lp_set(pad.lp, pad.lpfCutHz, SR);
 
  // Osc shape: sine→triangle blend (gentle range)
  pad.triMix = 0.10f + 0.60f * k;      // 0.10 .. 0.70
  pad.v1.triMix = pad.v2.triMix = pad.v3.triMix = pad.triMix;

  // Reverb damping: darker (more damp) at low mood → airier at high mood
  float moodDamp = 0.35f - 0.20f * k;  // 0.35 .. 0.15
  // Combine with current Space-based damp (average to avoid jumps)
  for (int i=0;i<4;i++){
    rv.c[i].damp = 0.5f * rv.c[i].damp + 0.5f * moodDamp;
  }
}

// REVERB 
inline void reverb_init(ReverbLite& r){
  r.c[0].buf = rvCombBuf1; r.c[0].size = RV_COMB1; r.c[0].w = 0; r.c[0].filt = 0;
  r.c[1].buf = rvCombBuf2; r.c[1].size = RV_COMB2; r.c[1].w = 0; r.c[1].filt = 0;
  r.c[2].buf = rvCombBuf3; r.c[2].size = RV_COMB3; r.c[2].w = 0; r.c[2].filt = 0;
  r.c[3].buf = rvCombBuf4; r.c[3].size = RV_COMB4; r.c[3].w = 0; r.c[3].filt = 0;
  for (uint16_t i=0;i<RV_COMB1;i++) rvCombBuf1[i]=0;
  for (uint16_t i=0;i<RV_COMB2;i++) rvCombBuf2[i]=0;
  for (uint16_t i=0;i<RV_COMB3;i++) rvCombBuf3[i]=0;
  for (uint16_t i=0;i<RV_COMB4;i++) rvCombBuf4[i]=0;

  r.ap1.buf = rvApBuf1; r.ap1.size = RV_AP1; r.ap1.w = 0; r.ap1.g = 0.5f;
  r.ap2.buf = rvApBuf2; r.ap2.size = RV_AP2; r.ap2.w = 0; r.ap2.g = 0.5f;
  for (uint16_t i=0;i<RV_AP1;i++) rvApBuf1[i]=0;
  for (uint16_t i=0;i<RV_AP2;i++) rvApBuf2[i]=0;

  r.inited = true;
}

inline float comb_step(CombLite& c, float x){
  int16_t rd = c.buf[c.w];            // delay = buffer length
  float y = rd * (1.0f/32767.0f);
  c.filt += c.damp * (y - c.filt);    // tone in feedback
  float wr = x + c.fb * c.filt;
  if (wr > 1.0f) wr = 1.0f; if (wr < -1.0f) wr = -1.0f;
  c.buf[c.w] = (int16_t)(wr * 32767.0f);
  c.w++; if (c.w >= c.size) c.w = 0;
  return y;
}

inline float allpass_step(AllpassLite& a, float x){
  int16_t rd = a.buf[a.w];
  float z = rd * (1.0f/32767.0f);
  float y = -x + z;
  float wr = x + a.g * y;
  if (wr > 1.0f) wr = 1.0f; if (wr < -1.0f) wr = -1.0f;
  a.buf[a.w] = (int16_t)(wr * 32767.0f);
  a.w++; if (a.w >= a.size) a.w = 0;
  return y;
}

inline void reverb_set_space(ReverbLite& r, int spacePct){
  float t = (spacePct <= 0) ? 0.0f : (spacePct >= 100 ? 1.0f : (spacePct * 0.01f));
  float k = sqrtf(t);
  float send = 0.70f * k;
  float wet  = 0.35f * k;

  float fb = 0.45f + 0.27f * k; if (fb > 0.75f) fb = 0.75f;
  float damp = 0.15f + 0.20f * k;
  for (int i=0;i<4;i++){ r.c[i].fb = fb; r.c[i].damp = damp; }

  r.baseSend = send; r.baseWet = wet;
  r.send = r.baseSend; r.wet = r.baseWet;               // live = base
}

inline float reverb_step(ReverbLite& r, float dry){
  if (!r.inited) reverb_init(r);
  float xin = r.send * dry;
  float y = 0.0f;
  y += comb_step(r.c[0], xin);
  y += comb_step(r.c[1], xin);
  y += comb_step(r.c[2], xin);
  y += comb_step(r.c[3], xin);
  y *= 0.25f;                 // average the combs
  y = allpass_step(r.ap1, y);
  y = allpass_step(r.ap2, y);
  return y;
}

// DELAY / SPACE
inline uint32_t delay_samples_from_bpm(int bpm, DelayMode m){
  if (bpm < 30) bpm = 30;
  float qnote = (SR * 60.0f) / (float)bpm;       // samples per 1/4
  float want  = (m == DELAY_1_8) ? (qnote * 0.5f) : qnote;
  uint32_t s  = (uint32_t)(want + 0.5f);
  if (s < 1u) s = 1u;
  if (s >= DELAY_MAX_SAMPLES) s = DELAY_MAX_SAMPLES - 1u;
  return s;
}

inline void delay_init(Delay& dl, int bpm, DelayMode m, float toneCutHz){
  dl.size = DELAY_MAX_SAMPLES;
  dl.w = 0;
  for (uint32_t i=0;i<dl.size;++i) dl.buf[i] = 0;
  dl.mode = m;
  dl.bpmCached = (uint16_t)bpm;
  dl.dSamp = delay_samples_from_bpm(bpm, m);
  lp_set(dl.tone, toneCutHz, SR);
  dl.inited = true;
}

inline void delay_update_tempo_if_changed(Delay& dl, int bpm){
  if (!dl.inited) return;
  if (bpm != (int)dl.bpmCached){
    dl.bpmCached = (uint16_t)bpm;
    dl.dSamp = delay_samples_from_bpm(bpm, dl.mode);
  }
}

inline void delay_set_mode(Delay& dl, DelayMode m){
  dl.mode = m;
  dl.dSamp = delay_samples_from_bpm(dl.bpmCached, m);
}

inline void delay_set_space(Delay& dl, int spacePct){
  float t = (spacePct <= 0) ? 0.0f : (spacePct >= 100 ? 1.0f : (spacePct * 0.01f));
  float k = sqrtf(t);

  float send = 0.75f * k;
  float wet  = 0.55f * k;
  float fb   = 0.55f * k; if (fb > 0.6f) fb = 0.6f;

  dl.baseSend = send; dl.baseWet = wet; dl.fb = fb;
  dl.send = dl.baseSend; dl.wet = dl.baseWet;          // live = base

  // keep reverb tied to Space as before
  reverb_set_space(gReverb, spacePct);
}

inline float delay_step(Delay& dl, float dry){
  if (!dl.inited) return 0.0f;

  uint32_t r = (dl.w + dl.size - dl.dSamp) % dl.size;
  int16_t r16 = dl.buf[r];
  float delayed = (float)r16 * (1.0f / 32767.0f);

  // feedback tone
  float fbSample = lp_process(dl.tone, delayed);

  // input to buffer: dry*send + fb*fbSample, gently clamped
  float toWrite = (dry * dl.send) + (fbSample * dl.fb);
  if (toWrite > 1.0f) toWrite = 1.0f;
  if (toWrite < -1.0f) toWrite = -1.0f;
  dl.buf[dl.w] = (int16_t)(toWrite * 32767.0f);

  dl.w++; if (dl.w >= dl.size) dl.w = 0;
  return delayed; // wet tap output
}



// -------- Soft clip (tanh-ish) --------
inline float softClip(float x) {
  const float drive = 1.8f; // a touch more output
  return tanhf(drive * x);
}

// -------- Audio globals --------
static DCBlock dc;
static DCBlock dcL, dcR;
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


// crossfade / scene change 
// --- [NEW] Scene update: schedule, pick targets, and crossfade (call this once per loop) ---
inline void scene_update(uint64_t nowSmp){
  // init scheduler window: 120–240 s
  if (!gSceneSch.inited){
    gSceneSch.minIntvSmp = (uint32_t)(120.0f * SR + 0.5f);
    gSceneSch.maxIntvSmp = (uint32_t)(240.0f * SR + 0.5f);
    gSceneSch.nextSceneAt = nowSmp + gSceneSch.minIntvSmp +
                            (uint32_t)((gSceneSch.maxIntvSmp - gSceneSch.minIntvSmp) * rand01());
    gSceneSch.inited = true;
  }

  // Helper lambdas
  auto lerp = [](float a, float b, float t){ return a + (b - a) * t; };
  auto smooth = [&](float t){ t = (t < 0.f)?0.f:((t>1.f)?1.f:t); return t*t*(3.f - 2.f*t); };
  auto expLerp = [&](float aHz, float bHz, float t){ // log-space interpolation
    float la = logf(aHz), lb = logf(bHz);
    return expf(lerp(la, lb, t));
  };

  // Start a new scene if due and not already fading
  if (!gScene.active && nowSmp >= gSceneSch.nextSceneAt){
    // capture start from current live state
    gScene.start.scale        = gScale;
    gScene.start.triMix       = pad.triMix;
    gScene.start.lpfCutHz     = pad.lpfCutHz;
    gScene.start.spacePercent = params.spacePercent;
    gScene.start.delayMode    = gDelay.mode;
    gScene.start.tempoBPM     = params.tempoBPM;

    // pick targets
    // scale
    int r = (int)(rand01() * 3.0f); if (r < 0) r = 0; if (r > 2) r = 2;
    gScene.target.scale = (r == 0) ? SCALE_A_MINOR : (r == 1) ? SCALE_D_DORIAN : SCALE_C_LYDIAN;
    // tri mix 0.15..0.65
    gScene.target.triMix = 0.15f + 0.50f * rand01();
    // LPF cutoff 400..8000 (log)
    gScene.target.lpfCutHz = 400.0f * powf(20.0f, rand01());
    // space 10..70
    gScene.target.spacePercent = 10 + (int)(60.0f * rand01() + 0.5f);
    // delay mode
    gScene.target.delayMode = (rand01() < 0.5f) ? DELAY_1_8 : DELAY_1_4;
    // tempo ±5%
    float f = 0.95f + 0.10f * rand01();
    int tBpm = (int)(params.tempoBPM * f + 0.5f);
    if (tBpm < 30) tBpm = 30; if (tBpm > 240) tBpm = 240;
    gScene.target.tempoBPM = tBpm;

    // crossfade 10–20 s
    float durSec = 10.0f + 10.0f * rand01();
    uint64_t durSmp = (uint64_t)(durSec * SR + 0.5f);
    gScene.tStartSmp = nowSmp;
    gScene.tEndSmp   = nowSmp + (durSmp ? durSmp : 1);
    gScene.active    = true;

    // show UI hint once at start
    tft.setTextSize(2);
    const char* msg = "Scene...fading";
    int16_t x1,y1; uint16_t w,h;
    tft.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
    int x = (tft.width() - (int)w)/2, y = 4; // top banner
    tft.fillRect(0, 0, tft.width(), h + 10, ST77XX_BLACK);
    tft.setCursor(x, y);
    tft.print(msg);

    // schedule next automatic scene
    gSceneSch.nextSceneAt = nowSmp + gSceneSch.minIntvSmp +
                            (uint32_t)((gSceneSch.maxIntvSmp - gSceneSch.minIntvSmp) * rand01());
  }

  // If active, apply crossfade ramps
  if (gScene.active){
    float t = 0.0f;
    if (gScene.tEndSmp > gScene.tStartSmp){
      uint64_t num = (nowSmp > gScene.tEndSmp) ? (gScene.tEndSmp - gScene.tStartSmp)
                                               : (nowSmp - gScene.tStartSmp);
      uint64_t den = (gScene.tEndSmp - gScene.tStartSmp);
      t = (float)num / (float)den;
    }
    float e = smooth(t);

    // triMix (linear, safe range)
    float tri = lerp(gScene.start.triMix, gScene.target.triMix, e);
    pad.triMix = tri; pad.v1.triMix = tri; pad.v2.triMix = tri; pad.v3.triMix = tri;

    // LPF cutoff (log interpolation)
    float cut = expLerp(gScene.start.lpfCutHz, gScene.target.lpfCutHz, e);
    pad.lpfCutHz = cut; lp_set(pad.lp, pad.lpfCutHz, SR);

    // Space / FX mix (int lerp)
    int spc = (int)(lerp((float)gScene.start.spacePercent, (float)gScene.target.spacePercent, e) + 0.5f);
    if (spc < 0) spc = 0; if (spc > 100) spc = 100;
    if (spc != params.spacePercent){ params.spacePercent = spc; delay_set_space(gDelay, params.spacePercent); }

    // Tempo glide (small steps; rest of system tracks)
    int bpm = (int)(lerp((float)gScene.start.tempoBPM, (float)gScene.target.tempoBPM, e) + 0.5f);
    if (bpm < 30) bpm = 30; if (bpm > 240) bpm = 240;
    if (bpm != params.tempoBPM) params.tempoBPM = bpm;

    // Scale: switch early in fade so new notes follow target
    if (e >= 0.2f) gScene.start.scale = gScene.target.scale, gScale = gScene.target.scale;

    // DelayMode: switch near end to avoid combing
    if (e >= 0.8f && gDelay.mode != gScene.target.delayMode){
      delay_set_mode(gDelay, gScene.target.delayMode);
    }

    // End of fade
    if (t >= 1.0f){
      gScene.active = false;
      // clear banner area
      tft.fillRect(0, 0, tft.width(), 18, ST77XX_BLACK);
    }
  }
}

// Pink Noise 

inline void pink_init(PinkNoise& p){
  p.b0 = p.b1 = p.b2 = 0.0f;
  lp_set(p.hpSplit, 150.0f, SR);   // remove HVAC/rumble
  lp_set(p.lpAir,  5000.0f, SR);   // keep only airy band
  p.inited = true;
}

inline float pink_step(PinkNoise& p){
  if (!p.inited) pink_init(p);

  // white in [-1,1)
  float w = (int32_t)rng32() * (1.0f / 2147483648.0f);

  // Paul Kellet 3-pole pink approximation
  p.b0 = 0.99765f * p.b0 + 0.0990460f * w;
  p.b1 = 0.96300f * p.b1 + 0.2965164f * w;
  p.b2 = 0.57000f * p.b2 + 1.0526913f * w;
  float y = (p.b0 + p.b1 + p.b2 + 0.1848f * w) * 0.05f;  // scaled down

  // band limit: HP via split LP, then LP for air
  float low  = lp_process(p.hpSplit, y);
  float high = y - low;                    // ~150 Hz high-pass
  float air  = lp_process(p.lpAir, high);  // ~5 kHz low-pass
  return air;
}

// --- [NEW] Gesture update & apply (call once per block, AFTER scene_update) ---
inline void gesture_update_and_apply(uint64_t nowSmp){
  // keep lambda tied to current Energy mapping
  gGesture.lambdaHz = (gEnergy.gestureLambdaHz > 0.0f) ? gEnergy.gestureLambdaHz : (1.0f/15.0f);

  // seed next event if needed
  if (gGesture.nextAt == 0){
    gGesture.nextAt = nowSmp + samplesFromExp(gGesture.lambdaHz);
  }

  // start new gesture if due
  if (!gGesture.active && nowSmp >= gGesture.nextAt){
    float atk = 0.5f + 1.0f * rand01();      // 0.5..1.5 s
    float rel = 2.5f + 2.0f * rand01();      // 2.5..4.5 s
    uint64_t aS = (uint64_t)(atk * SR + 0.5f);
    uint64_t rS = (uint64_t)(rel * SR + 0.5f);

    gGesture.tStart    = nowSmp;
    gGesture.tAtkEnd   = gGesture.tStart + (aS ? aS : 1);
    gGesture.tRelStart = gGesture.tAtkEnd;
    gGesture.tEnd      = gGesture.tRelStart + (rS ? rS : 1);
    gGesture.env       = 0.0f;
    gGesture.active    = true;

    gGesture.cutBoostOct = 0.25f + 0.55f * rand01();  // 0.25..0.80
    gGesture.fxSendBoost = 0.30f + 0.40f * rand01();  // 0.30..0.70
    gGesture.fxWetBoost  = 0.20f + 0.30f * rand01();  // 0.20..0.50

    // schedule next
    gGesture.nextAt = nowSmp + samplesFromExp(gGesture.lambdaHz);
  }

  // raw env u(t)
  float u = 0.0f;
  if (gGesture.active){
    if (nowSmp < gGesture.tAtkEnd){
      uint64_t num = nowSmp - gGesture.tStart;
      uint64_t den = gGesture.tAtkEnd - gGesture.tStart;
      u = (den ? (float)num/(float)den : 1.0f);
    } else if (nowSmp < gGesture.tEnd){
      uint64_t num = nowSmp - gGesture.tRelStart;
      uint64_t den = gGesture.tEnd - gGesture.tRelStart;
      float d = (den ? (float)num/(float)den : 1.0f);
      u = 1.0f - d;
    } else {
      gGesture.active = false;
      gGesture.env = 0.0f;
    }
  }

 

  // smooth env (~50ms time constant at block-rate)
  const float alpha = 1.0f - expf(-(float)CHUNK / (SR * 0.05f));
  gGesture.env += alpha * (u - gGesture.env);
  float e = gGesture.env;

  // apply overlay (only while active or if smoothing tail > 0)
  if (e > 0.0005f){
    // Pad brighten: multiplicative in octaves
    float effCut = pad.lpfCutHz * powf(2.0f, gGesture.cutBoostOct * e);
    if (effCut > 8000.0f) effCut = 8000.0f;
    if (effCut < 400.0f)  effCut = 400.0f;
    lp_set(pad.lp, effCut, SR);  // retune 1-pole cutoff

    // FX sends/wets scaled from bases (non-destructive)
    gDelay.send  = gDelay.baseSend  * (1.0f + gGesture.fxSendBoost * e);
    gDelay.wet   = gDelay.baseWet   * (1.0f + gGesture.fxWetBoost  * e);
    gReverb.send = gReverb.baseSend * (1.0f + gGesture.fxSendBoost * e);
    gReverb.wet  = gReverb.baseWet  * (1.0f + gGesture.fxWetBoost  * e);
  } else {
    // restore to bases when env ~ 0
    gDelay.send  = gDelay.baseSend;   gDelay.wet  = gDelay.baseWet;
    gReverb.send = gReverb.baseSend;  gReverb.wet = gReverb.baseWet;
  }
}

static inline void panGains(float pan, float &gL, float &gR){
  // Equal-power (−3 dB) law, θ = 0.25π*(pan+1); 0.25π ≈ 0.78539816339
  float th = 0.78539816339f * (pan + 1.0f);
  gL = cosf(th);
  gR = sinf(th);
}
static inline void applyWidth(float &L,float &R,float width){
  if(width<0.2f) width=0.2f; if(width>1.2f) width=1.2f;
  float M=0.5f*(L+R), S=0.5f*(L-R)*width; L=M+S; R=M-S;
}

// --- Apply encoder rotation to active page ---
void applyEncoderDelta(int steps){
  if (steps == 0) return;
  int delta = (steps > 0) ? 1 : -1;

  if (currentPage == PAGE_VOLUME) {
    params.volumePercent = clampVolume(params.volumePercent + delta);
    volTarget = (float)params.volumePercent;
  } else if (currentPage == PAGE_TEMPO) {
    params.tempoBPM = clampTempo(params.tempoBPM + delta);
  } else if (currentPage == PAGE_SPACE) {
    params.spacePercent = constrain(params.spacePercent + delta, 0, 100);
    delay_set_space(gDelay, params.spacePercent);
  } else if (currentPage == PAGE_MOOD) {
    params.moodPercent = constrain(params.moodPercent + delta, 0, 100);
    mood_set(pad, gReverb, params.moodPercent);
  } else if (currentPage == PAGE_TONE) {
    params.tonePercent = constrain(params.tonePercent + delta, 0, 100);
    tone_set(gTilt, params.tonePercent);
  } else if (currentPage == PAGE_ENERGY) { // PAGE_ENERGY
    params.energyPercent = constrain(params.energyPercent + delta, 0, 100);
    energy_set(gEnergy, pad, params.energyPercent);
  } else if (currentPage == PAGE_SCENE){
    (void)delta;                       // any turn triggers immediately
    gSceneSch.nextSceneAt = gClock.smpNow;  // due now
    scene_update(gClock.smpNow);            // kick off crossfade ("Scene...fading" banner)
    return;
  }
}


// --- UI draw for flat pages ---
void uiMaybeRedraw(){
  int shownVal =
    (currentPage == PAGE_VOLUME) ? params.volumePercent :
    (currentPage == PAGE_TEMPO)  ? params.tempoBPM :
    (currentPage == PAGE_SPACE)  ? params.spacePercent :
    (currentPage == PAGE_MOOD)   ? params.moodPercent :
                                   params.tonePercent; // PAGE_TONE

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
  // randomize seed 
  gRng32 ^= esp_random() ^ ((uint32_t)millis() << 1);
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

  // delay init 
  params.spacePercent = 45;                 // more obvious by default
  delay_init(gDelay, params.tempoBPM, DELAY_1_4, 2800.0f);  // darker tone, longer tap
  delay_set_space(gDelay, params.spacePercent);

  // mood / tone init 
  params.moodPercent = 50;
  params.tonePercent = 50;
  tone_init(gTilt);
  params.tonePercent = 42;   // was 50; a touch warmer by default
  tone_set(gTilt, params.tonePercent);
  mood_set(pad, gReverb, params.moodPercent);
  tone_set(gTilt, params.tonePercent);

  
  // energy 
  params.energyPercent = 50;
  energy_set(gEnergy, pad, params.energyPercent);

  // noise 
  pink_init(gPink);

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
  delay_update_tempo_if_changed(gDelay, params.tempoBPM);

  const float CTR = 0.70710678f;  // −3 dB center for mono returns

  // --- Slow random walk for drone pitch ---
  uint32_t now = millis();
  if (now >= nextPitchChange) {
    // step target by -2..+2 Hz
    float step = (rand01() * 4.0f) - 2.0f;
    pitchTargetHz += step;

    // clamp to safe range 100..220 Hz
    if (pitchTargetHz < 100.0f) pitchTargetHz = 100.0f;
    if (pitchTargetHz > 220.0f) pitchTargetHz = 220.0f;

    // schedule next change in 3–7 seconds
    nextPitchChange = now + 3000 + (uint32_t)(rand01() * 4000.0f);
  }

  // smooth toward target (≈15s time constant)
  const float alphaPitch = 1.0f - expf(-(float)CHUNK / (SR * 15.0f));
  pitchSmoothHz += alphaPitch * (pitchTargetHz - pitchSmoothHz);

  // apply smoothed base pitch to pad oscillators
  pad.baseHz = pitchSmoothHz;
  pad.v1.freqHz = pad.baseHz;
  pad.v2.freqHz = pad.baseHz * (1.0f + pad.det);
  pad.v3.freqHz = pad.baseHz * (1.0f - pad.det);
  for (int i = 0; i < CHUNK; ++i) {
    // --- Pad (mono)
    float padMono = pad_step(pad, SR);

    // --- Plucks (mono, no panning)
    float plMono = 0.0f;
    for (int vi = 0; vi < 4; ++vi) {
      PluckVoice &v = gPluck.v[vi];
      if (!v.active) continue;

      float env = adsr_step(v.eg, SR);
      if (v.eg.st == ENV_IDLE) { v.active = false; continue; }

      v.phase += TAU * (v.freqHz / SR);
      if (v.phase >= TAU) v.phase -= TAU;

      // 2-osc mix: 80% sine + 20% rectangle (30% duty)
      float sSin = sinf(v.phase);
      float duty = 0.30f;
      float sSq  = (fmodf(v.phase / TAU, 1.0f) < duty) ? 1.0f : -1.0f;
      float sMix = 0.80f * sSin + 0.20f * sSq;

      float sV = sMix * env * v.vel;
      plMono += sV * gPluck.gain * 0.80f;   // keep your −20% gain
    }

    // Noise 
    float n = pink_step(gPink);
    float noiseDry = n * NOISE_DRY_GAIN;
    float noiseFx  = n * NOISE_FX_GAIN;

    // --- Build mono dry
    float dryMono = padMono * CTR + plMono + noiseDry;

    // --- FX bus (mono) with tone tilt
    float dryFX  = tone_process(gTilt, dryMono + noiseFx);
    float wetTap = delay_step(gDelay,  dryFX);
    float wetRev = reverb_step(gReverb, dryFX);
    float wetSum = gDelay.wet * wetTap + gReverb.wet * wetRev;

    // Center FX return into mono
    float outMono = dryMono + wetSum * CTR;

    // --- Master gain, DC block, clip
    float x = outMono * volSmooth * fade * MASTER_GAIN;
    // use per-channel DC if you like; same x to both is fine
    float xL = dcL.process(x);
    float xR = dcR.process(x);
    xL = softClip(xL);
    xR = softClip(xR);

    int j = i * 2;
    buf[j + 0] = (int16_t)floorf(xL * 32767.0f);  // Left (mono)
    buf[j + 1] = (int16_t)floorf(xR * 32767.0f);  // Right (mono)
  }

  size_t written = 0;
  i2s_write(I2S_NUM_0, (const char*)buf, sizeof(buf), &written, portMAX_DELAY);
  clock_advance_block(gClock, (uint32_t)CHUNK);
  maybeTriggerGesture(gEnergy, gClock.smpNow);
  scene_update(gClock.smpNow);
  gesture_update_and_apply(gClock.smpNow);
}


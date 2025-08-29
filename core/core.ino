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

enum DelayMode : uint8_t { DELAY_1_8 = 0, DELAY_1_4 };
enum Scale : uint8_t { SCALE_A_MINOR = 0, SCALE_D_DORIAN, SCALE_C_LYDIAN, SCALE_COUNT };
enum Page { PAGE_VOLUME = 0, PAGE_TEMPO, PAGE_SPACE, PAGE_COUNT };
struct Params {
  int volumePercent; // 0..100
  int tempoBPM;      // 30..240 (clamped)
  int spacePercent;  // 0..100  (delay send/mix)
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


#if defined(DELAY_MAX_SAMPLES)
  #undef DELAY_MAX_SAMPLES
#endif
#define DELAY_MAX_SAMPLES 32768  // ~0.74s @ 44.1k (1/4 note ≥ ~81 BPM), ≈64 KB

struct Delay {
  int16_t buf[DELAY_MAX_SAMPLES]; // mono ring buffer, no malloc
  uint32_t size = DELAY_MAX_SAMPLES;
  uint32_t w = 0;                 // write index
  uint32_t dSamp = 22050;         // delay time in samples
  OnePoleLP tone;                 // LP in feedback path
  float send = 0.0f;              // dry → delay input
  float wet  = 0.0f;              // wet tap mix
  float fb   = 0.0f;              // feedback amount
  uint16_t bpmCached = 104;
  DelayMode mode = DELAY_1_8;
  bool inited = false;
};

static Delay gDelay;

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
  float send = 0.0f; // dry → reverb in
  float wet  = 0.0f; // reverb tap → mix
  bool inited = false;
};
static ReverbLite gReverb;

// --- [NEW] Lightweight Pluck: voice/pool & tiny RNG (place with other types/globals) ---
enum EnvState : uint8_t { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };

struct ADSR {
  float a=0.008f, d=0.22f, s=0.20f, r=0.14f; // soft pluck
  float env=0.0f;
  EnvState st=ENV_IDLE;
  bool gate=false;
};

struct PluckVoice {
  bool  active=false;
  int   midi=60;
  float freqHz=261.63f;
  float phase=0.0f;
  float vel=0.7f;     // 0..1 subtle
  ADSR  eg;
};

struct PluckPool {
  PluckVoice v[4];
  float gain=0.35f;   // headroom vs pad
  bool  inited=false;
};

static PluckPool gPluck;
static Scale gScale = SCALE_A_MINOR;      // use your quantizer scales
static uint32_t gRng32 = 0xA3C59AC3u;     // tiny xorshift RNG state

// --- [NEW] Scales: enums, defs, and table ---
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
    case PAGE_SPACE:  return "Space";
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
   // lazy init OK here
  if (!gPluck.inited) pluck_init(gPluck);

  // Probability by grid strength
  float p = 0.0f;
  if ((step & 3u) == 0u)       p = 0.55f;   // strong beats (every 4th 1/16)
  else if ((step & 1u) == 0u)  p = 0.28f;   // even 1/8 positions
  else                         p = 0.10f;   // off 1/16s

  if (rand01() < p){
    int center = 60; // C4 center (simple & musical)
    // pick a nearby in-scale note
    int note = pickAmbientNote(gScale, center);
    float vel = 0.55f + 0.35f * rand01(); // subtle velocity 0.55..0.90
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

  // gentle LPF
  s = lp_process(p.lp, s);
  return s;
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
  float k = sqrtf(t);                 // audible at low values
  r.send = 0.70f * k;
  r.wet  = 0.35f * k;

  float fb = 0.45f + 0.27f * k;       // 0.45 → 0.72
  if (fb > 0.75f) fb = 0.75f;
  float damp = 0.15f + 0.20f * k;     // darker→airier with space

  for (int i=0;i<4;i++){ r.c[i].fb = fb; r.c[i].damp = damp; }
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

  dl.send = 0.75f * k;           // dry → delay input
  dl.wet  = 0.55f * k;           // wet mix back to output
  float fb = 0.55f * k;          // feedback kept safe
  if (fb > 0.6f) fb = 0.6f;
  dl.fb = fb;

  // NEW: tie Space to reverb as well
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
  }  else if (currentPage == PAGE_TEMPO) { // PAGE_TEMPO
    params.tempoBPM = clampTempo(params.tempoBPM + delta);
  } else { // PAGE_SPACE
    params.spacePercent += delta;
    if (params.spacePercent < 0) params.spacePercent = 0;
    if (params.spacePercent > 100) params.spacePercent = 100;
    // reflect mapping immediately
    delay_set_space(gDelay, params.spacePercent);
  }
}


// --- UI draw for flat pages ---
void uiMaybeRedraw(){
  int shownVal = (currentPage == PAGE_VOLUME) ? params.volumePercent
                : (currentPage == PAGE_TEMPO) ? params.tempoBPM
                : params.spacePercent; // PAGE_SPACE
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

  // delay init 
  params.spacePercent = 45;                 // more obvious by default
  delay_init(gDelay, params.tempoBPM, DELAY_1_4, 2800.0f);  // darker tone, longer tap
  delay_set_space(gDelay, params.spacePercent);

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


  for (int i = 0; i < CHUNK; ++i) {
    float dry = /*pad_step(pad, SR) +*/ pluck_step(gPluck, SR);

    float wetTap  = delay_step(gDelay,  dry);     // tempo-aware delay
    float wetRev  = reverb_step(gReverb, dry);    // light reverb

    float s = dry + gDelay.wet * wetTap + gReverb.wet * wetRev;

    float x = s * volSmooth * fade;
    x = dc.process(x);
    x = softClip(x);

    int16_t v = (int16_t)floorf(x * 32767.0f);
    int j = i * 2;
    buf[j + 0] = v;  // L
    buf[j + 1] = v;  // R
  }

  size_t written = 0;
  i2s_write(I2S_NUM_0, (const char*)buf, sizeof(buf), &written, portMAX_DELAY);
  clock_advance_block(gClock, (uint32_t)CHUNK);
}

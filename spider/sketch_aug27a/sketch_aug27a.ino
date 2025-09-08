/*
  ESP32-WROOM Ambient-House Generator — Single File .ino (DRAM+NOISE FIX)
  - DRAM reduced: smaller delay (0.55s), smaller sine table, tiny reverb.
  - Audio format: I2S_COMM_FORMAT_I2S_MSB for MAX98357A (less crackle).
  - Output safety: lower master, smoother saw, 3s startup fade-in.
  - Kept: single-knob pages, scenes, gestures, evolving ambient-house.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <driver/i2s.h>
#include <math.h>
#include <algorithm>

// -----------------------------
// Fixed Pins (exactly as requested)
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

#ifndef ST77XX_DARKGREY
#define ST77XX_DARKGREY 0x7BEF
#endif

// -----------------------------
// Build / audio constants
// -----------------------------
static const int SR          = 44100;
static const int BLOCK_SAMP  = 256;   // audio block size (frames)
static const int NUM_OUT_CH  = 2;     // interleaved stereo (L,R)
static const float DT        = 1.0f / (float)SR;
static const float TAU_F     = 6.2831853071795864769f;

// ---- UI / TFT ----
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
static const uint16_t COL_BG      = ST77XX_BLACK;
static const uint16_t COL_FG      = ST77XX_WHITE;
static const uint16_t COL_ACCENT  = ST77XX_CYAN;
static const uint16_t COL_METER   = ST77XX_GREEN;
static const uint16_t COL_BARBG   = 0x4208; // dark gray
static const uint8_t  UI_FPS      = 25;     // ~40ms
static const int      UI_W        = 128;
static const int      UI_H        = 160;

// -----------------------------
// Musical constants & scales
// -----------------------------
static int BASE_KEY = 57; // A3 (A minor default)
enum ScaleMode { SCALE_AEOLIAN=0, SCALE_DORIAN=1, SCALE_LYDIAN=2, SCALE_PHRYGIAN=3, SCALE_COUNT=4 };
static int SCALE_MODE = SCALE_AEOLIAN;

static const int8_t AEOLIAN_STEPS[7]  = {0,2,3,5,7,8,10};
static const int8_t DORIAN_STEPS[7]   = {0,2,3,5,7,9,10};
static const int8_t LYDIAN_STEPS[7]   = {0,2,4,6,7,9,11};
static const int8_t PHRYGIAN_STEPS[7] = {0,1,3,5,7,8,10};
static const char* scaleNames[SCALE_COUNT] = {"Aeolian", "Dorian", "Lydian", "Phrygian"};

// -----------------------------
// Pages (one knob, flat list)
// -----------------------------
enum Page {
  PAGE_MOOD=0, PAGE_ENERGY, PAGE_SPACE, PAGE_TONE, PAGE_SCALE,
  PAGE_TEMPO, PAGE_VOLUME, PAGE_SCENE, PAGE_COUNT
};
static const char* pageNames[PAGE_COUNT] = {
  "Mood", "Energy", "Space", "Tone", "Scale", "Tempo", "Volume", "Scene"
};
static const char* pageHints[PAGE_COUNT] = {
  "dark<->bright",
  "calm<->lively",
  "dry<->wide",
  "soft<->crisp",
  "rotate to choose, click to save",
  "+/- 10%",
  "safe soft-limit",
  "click: morph now"
};

// -----------------------------
// Encoder state (interrupt safe)
// -----------------------------
volatile int32_t encDelta = 0;
volatile uint32_t encLastMicros = 0;
volatile uint8_t lastAB = 0;
static uint32_t btnDownMs = 0;
static bool btnDown = false;

// -----------------------------
// Random helpers
// -----------------------------
static uint32_t rngState = 0xA3C59AC3u;
static inline uint32_t xorshift32() {
  uint32_t x = rngState;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  rngState = x ? x : 0xA3C59AC3u;
  return rngState;
}
static inline float frand() {
  return (xorshift32() & 0xFFFFFF) / 16777216.0f; // [0,1)
}
static uint32_t expMsRand(float mean_ms) { // exponential RV
  float u = max(1e-6f, frand());
  float t = -logf(u) * mean_ms;
  return (uint32_t)t;
}

// -----------------------------
// Fast trig via wavetable (sine) — smaller to save DRAM
// -----------------------------
static const int SINE_TBL_SZ = 1024; // was 2048
static float sineTable[SINE_TBL_SZ];
static inline float fast_sin(float ph) {
  while (ph >= TAU_F) ph -= TAU_F;
  while (ph < 0.0f)   ph += TAU_F;
  float idx = (ph * (float)SINE_TBL_SZ) * (1.0f / TAU_F);
  int i0 = (int)idx;
  float frac = idx - (float)i0;
  int i1 = (i0 + 1) & (SINE_TBL_SZ-1);
  return sineTable[i0 & (SINE_TBL_SZ-1)] * (1.0f - frac) + sineTable[i1] * frac;
}
static inline float fast_cos(float ph){ return fast_sin(ph + 0.25f*TAU_F); }

// ======================================================================
//                 DSP NAMESPACE (avoids Arduino prototypes)
// ======================================================================
namespace DSP {

// Filters & shapers
struct LP1 { float a=0.0f, y=0.0f; };
struct HP1 { float a=0.0f, y=0.0f, x1=0.0f; };
struct DCBlock { float R=0.995f, y=0.0f, x1=0.0f; };

static inline void lp_set(LP1& f, float fc, float sr) {
  float x = expf(-TAU_F*fc/sr);
  f.a = 1.0f - x;
}
static inline float lp_process(LP1& f, float x) {
  f.y += f.a * (x - f.y);
  return f.y;
}
static inline void hp_set(HP1& f, float fc, float sr) {
  float x = expf(-TAU_F*fc/sr);
  f.a = x;
}
static inline float hp_process(HP1& f, float x) {
  float y = f.a*(f.y + x - f.x1);
  f.y = y; f.x1 = x; return y;
}
static inline float dcblock_process(DCBlock& d, float x) {
  float y = x - d.x1 + d.R * d.y;
  d.x1 = x; d.y = y; return y;
}

// Delay — shrink to ~0.55s max to save DRAM
static const int DELAY_MAX_SAMP = (int)(SR * 0.55f); // ~24,255 samples ≈ 47KB
static int16_t delayBuf[DELAY_MAX_SAMP];
static int delayW = 0;
static LP1 delayToneLP;

static inline float delay_process(float in, int delaySamples, float fb, float mix) {
  if (delaySamples < 1) delaySamples = 1;
  if (delaySamples >= DELAY_MAX_SAMP) delaySamples = DELAY_MAX_SAMP-1;
  int r = delayW - delaySamples; if (r < 0) r += DELAY_MAX_SAMP;
  float y = (float)delayBuf[r] / 32768.0f;
  float fbSig = lp_process(delayToneLP, y);
  float w = in + fb * fbSig;
  w = constrain(w, -1.0f, 1.0f);
  delayBuf[delayW] = (int16_t)(w * 32767.0f);
  delayW++; if (delayW >= DELAY_MAX_SAMP) delayW = 0;
  return in*(1.0f - mix) + y*mix;
}

// Reverb — compact comb sizes to save DRAM (≈ 22KB total)
struct Comb {
  int16_t* buf; int len; int idx; float fb; LP1 damp; float z;
};
struct Allpass {
  int16_t* buf; int len; int idx; float g;
};
// Smaller buffers than before
static int16_t revBuf1[3001], revBuf2[3313], revBuf3[3559], revBuf4[3907];
static int16_t apBuf1[421], apBuf2[383];
static Comb revC[4];
static Allpass revA[2];

static inline float comb_process(Comb& c, float x) {
  int r = c.idx; float y = (float)c.buf[r]/32768.0f;
  float fbSig = lp_process(c.damp, c.z);
  c.z = y;
  float w = x + c.fb * fbSig;
  w = constrain(w, -1.0f, 1.0f);
  c.buf[c.idx] = (int16_t)(w * 32767.0f);
  c.idx++; if (c.idx >= c.len) c.idx = 0;
  return y;
}
static inline float allpass_process(Allpass& a, float x) {
  int r = a.idx; float y = (float)a.buf[r]/32768.0f;
  float z = y;
  float w = x + (-a.g) * y;
  w = constrain(w, -1.0f, 1.0f);
  a.buf[a.idx] = (int16_t)((w) * 32767.0f);
  a.idx++; if (a.idx >= a.len) a.idx = 0;
  return z + a.g * w;
}
static inline float reverb_process(float in) {
  float s = 0.25f * (comb_process(revC[0], in)
                   + comb_process(revC[1], in)
                   + comb_process(revC[2], in)
                   + comb_process(revC[3], in));
  s = allpass_process(revA[0], s);
  s = allpass_process(revA[1], s);
  return s;
}

static inline float softclip(float x) {
  const float x2 = x*x;
  return x * (27.0f + x2) / (27.0f + 9.0f*x2);
}

} // namespace DSP

using DSP::LP1; using DSP::HP1; using DSP::DCBlock;

// -----------------------------
// Voice structs
// -----------------------------
struct UniVoice {
  float phase=0, inc=0, pan=0.5f;
  float drift=0, driftRate=0;
};
static const int PAD_UNISON = 4;
struct Pad {
  UniVoice v[PAD_UNISON];
  float lfoPh=0, lfoRate=0.1f, lfoAmt=0.2f;
  LP1   lp; float cutoff=1200.0f;
  float sawMix=0.22f; // gentler default
  float amp=0.30f;    // slightly lower
};
struct Pluck {
  bool  on=false; float env=0, a=0.003f, d=0.25f;
  float phase=0, inc=0, noiseMix=0.18f; HP1 hp; LP1 lp;
  float pan=0.5f;
};
static const int PLUCK_N = 4;
struct Perc {
  bool on=false; float env=0, a=0.0008f, d=0.03f; HP1 hp;
};
struct Sub {
  float phase=0, inc=0, amp=0.12f; // gentler
};
struct Air {
  float env=1.0f; LP1 lp; HP1 hp; float amt=0.06f;
};

// -----------------------------
// Global engine params
// -----------------------------
struct Params {
  float mood=50, energy=40, space=35, tone=50;
  int   scaleIdx=SCALE_AEOLIAN;
  float tempo=104.0f;
  float volume=75.0f; // lower default
};
static Params P;
static Params Ptarget = P;
static Params Pscene0 = P;
static float  sceneAlpha=1.0f;
static uint32_t nextSceneMs=0;
static uint32_t sceneStartMs=0;
static uint32_t sceneCrossMs=0;

// Gestures
static bool gestureOn=false;
static uint32_t gestEndMs=0;
static uint32_t nextGestMs=0;

// Auto-wander timer
static uint32_t nextWanderMs=0;

// Stereo / FX
static float stereoWidth = 0.28f;
static float revMix = 0.14f, revDamp = 0.5f, delSend = 0.16f, delFB = 0.33f;
static float tilt = 0.0f;
static DCBlock dcL, dcR;

// Voices
static Pad   pad;
static Pluck plk[PLUCK_N];
static Perc  perc;
static Sub   sub;
static Air   air;

// Scheduler / timing
static uint64_t sampleCounter=0;
static uint32_t nextSubdivSample=0;
static int subdivDivisor = 8;
static float humanizeFrac = 0.25f;

// UI state
static int curPage = PAGE_MOOD;
static uint32_t lastUiMs=0;
static float outMeter=0.0f;

// Startup fade
static uint32_t bootMs=0;

// -----------------------------
// Quantizer
// -----------------------------
static inline int quantizeToScale(int midi, int rootMidi, const int8_t* steps) {
  int rel = midi - rootMidi;
  int oct = floorf(rel / 12.0f);
  int within = rel - 12*oct;
  int best = steps[0];
  for (int i=1;i<7;i++){ if (steps[i] <= within) best = steps[i]; }
  return rootMidi + 12*oct + best;
}
static inline const int8_t* currentScaleSteps() {
  switch (P.scaleIdx) {
    default:
    case SCALE_AEOLIAN: return AEOLIAN_STEPS;
    case SCALE_DORIAN:  return DORIAN_STEPS;
    case SCALE_LYDIAN:  return LYDIAN_STEPS;
    case SCALE_PHRYGIAN:return PHRYGIAN_STEPS;
  }
}
static inline float midi2hz(int m) {
  return 440.0f * powf(2.0f, (m-69)/12.0f);
}

// -----------------------------
// Mapping from pages to parameters
// -----------------------------
static void applyParamMapping() {
  float moodN = P.mood * 0.01f;
  float cutoff = expf(logf(400.0f) + (logf(8000.0f/400.0f))*moodN);
  pad.cutoff = cutoff;
  pad.sawMix = 0.12f + 0.35f * moodN; // clamp lower
  if (pad.sawMix > 0.48f) pad.sawMix = 0.48f; // avoid harsh
  revDamp    = 0.30f + 0.40f * moodN;

  float engN = P.energy * 0.01f;
  pad.lfoRate = 0.03f + 0.37f * engN;

  float spN = P.space * 0.01f;
  delSend   = 0.05f + 0.28f * spN;
  revMix    = 0.08f + 0.18f * spN;
  stereoWidth = 0.10f + 0.45f * spN;

  float toneN = (P.tone - 50.0f) / 50.0f; // -1..1
  tilt = 2.6f * toneN; // slightly gentler
}
static void updateSceneBlend() {
  auto lerp = [](float a, float b, float t){ return a + (b-a)*t; };
  P.mood   = lerp(Pscene0.mood,   Ptarget.mood,   sceneAlpha);
  P.energy = lerp(Pscene0.energy, Ptarget.energy, sceneAlpha);
  P.space  = lerp(Pscene0.space,  Ptarget.space,  sceneAlpha);
  P.tone   = lerp(Pscene0.tone,   Ptarget.tone,   sceneAlpha);
  P.tempo  = lerp(Pscene0.tempo,  Ptarget.tempo,  sceneAlpha);
  if (sceneAlpha >= 0.999f) P.scaleIdx = Ptarget.scaleIdx;
  applyParamMapping();
}

// -----------------------------
// Encoder ISRs
// -----------------------------
void IRAM_ATTR isr_encA() {
  uint32_t now = micros();
  if (now - encLastMicros < 300) return;
  encLastMicros = now;
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  uint8_t ab = (a<<1) | b;
  if ((lastAB==0b00 && ab==0b01) || (lastAB==0b01 && ab==0b11) ||
      (lastAB==0b11 && ab==0b10) || (lastAB==0b10 && ab==0b00)) {
    encDelta++;
  } else if ((lastAB==0b00 && ab==0b10) || (lastAB==0b10 && ab==0b11) ||
             (lastAB==0b11 && ab==0b01) || (lastAB==0b01 && ab==0b00)) {
    encDelta--;
  }
  lastAB = ab;
}
void IRAM_ATTR isr_encB() { isr_encA(); }

// -----------------------------
// UI helpers
// -----------------------------
static void uiDrawStatic() {
  tft.fillScreen(COL_BG);
  tft.fillRect(0,0, UI_W, 18, COL_BARBG);
  tft.setTextWrap(false); tft.setTextSize(1);
  tft.setCursor(2, 4); tft.setTextColor(COL_FG);
  tft.print("Ambient-House");
}

static void uiDrawStatus() {
  tft.fillRect(0, 18, UI_W, 14, COL_BG);
  tft.setCursor(2, 20); tft.setTextColor(COL_ACCENT);
  tft.print(pageNames[curPage]);
  tft.setCursor(70, 20); tft.setTextColor(COL_FG);
  int val = 0;
  switch (curPage) {
    case PAGE_MOOD:   val = (int)roundf(P.mood); break;
    case PAGE_ENERGY: val = (int)roundf(P.energy); break;
    case PAGE_SPACE:  val = (int)roundf(P.space); break;
    case PAGE_TONE:   val = (int)roundf(P.tone); break;
    case PAGE_SCALE:  val = SCALE_MODE; break;
    case PAGE_TEMPO:  val = (int)roundf(P.tempo); break;
    case PAGE_VOLUME: val = (int)roundf(P.volume); break;
    case PAGE_SCENE:  val = (int)roundf(sceneAlpha*100.0f); break;
  }
  if (curPage == PAGE_SCALE)      { tft.print(scaleNames[P.scaleIdx]); }
  else if (curPage == PAGE_SCENE) { tft.print("xfade "); tft.print(val); tft.print("%"); }
  else                            { tft.print(val); }

  int mw = (int)(outMeter * (UI_W-4)); if (mw < 0) mw = 0; if (mw > UI_W-4) mw = UI_W-4;
  tft.fillRect(2, 34, UI_W-4, 4, ST77XX_DARKGREY);
  tft.fillRect(2, 34, mw, 4, COL_METER);

  tft.fillRect(0, 42, UI_W, 10, COL_BG);
  tft.setCursor(2, 44); tft.setTextColor(ST77XX_YELLOW);
  tft.print(pageHints[curPage]);
}

// idle animation
static void uiIdleAnim(uint32_t nowMs) {
  static uint16_t lastY[16] = {0};
  static uint16_t lastX[16] = {0};
  static bool init=false;
  if (init) {
    for (int i=0;i<16;i++){
      tft.drawPixel(lastX[i], lastY[i], COL_BG);
    }
  }
  float beatHz = P.tempo / 60.0f;
  float t = nowMs * 0.001f;
  for (int i=0;i<16;i++){
    float x = 4 + i*7.7f;
    float ph = t*TAU_F*beatHz + i*0.2f;
    float y = 70 + 20.0f*fast_sin(ph);
    int xi = (int)x; int yi = (int)y;
    tft.drawPixel(xi, yi, COL_ACCENT);
    lastX[i]=xi; lastY[i]=yi;
  }
  init=true;
}

static void nextPage() {
  curPage = (curPage + 1) % PAGE_COUNT;
}
static void editPageByDelta(int d) {
  switch (curPage) {
    case PAGE_MOOD:   P.mood   = constrain(P.mood + d, 0.0f, 100.0f); break;
    case PAGE_ENERGY: P.energy = constrain(P.energy + d, 0.0f, 100.0f); break;
    case PAGE_SPACE:  P.space  = constrain(P.space + d, 0.0f, 100.0f); break;
    case PAGE_TONE:   P.tone   = constrain(P.tone + d, 0.0f, 100.0f); break;
    case PAGE_SCALE:  P.scaleIdx = (P.scaleIdx + (d>0?1:-1) + SCALE_COUNT) % SCALE_COUNT; break;
    case PAGE_TEMPO:  P.tempo  = constrain(P.tempo + d*0.5f, 93.6f, 114.4f); break; // +/-10%
    case PAGE_VOLUME: P.volume = constrain(P.volume + d, 0.0f, 100.0f); break;
    case PAGE_SCENE:  break;
  }
  applyParamMapping();
}

static void triggerManualScene() {
  Pscene0 = P; sceneAlpha = 0.0f;
  Ptarget = P;
  auto jitter = [](float x, float amt){ return constrain(x + (frand()*2.0f-1.0f)*amt, 0.0f, 100.0f); };
  Ptarget.mood   = jitter(P.mood,   25.0f);
  Ptarget.energy = jitter(P.energy, 25.0f);
  Ptarget.space  = jitter(P.space,  25.0f);
  Ptarget.tone   = jitter(P.tone,   25.0f);
  if (frand() < 0.33f) Ptarget.scaleIdx = (P.scaleIdx + 1 + (int)(frand()*3.0f)) % SCALE_COUNT;
  Ptarget.tempo  = constrain(P.tempo + (frand()*2.0f-1.0f)*3.0f, 93.6f, 114.4f);
  sceneCrossMs = 10000 + (uint32_t)(frand()*10000.0f);
  sceneStartMs = millis();
}

static void scheduleNextScene() {
  uint32_t ms = millis();
  nextSceneMs = ms + (120000u + (uint32_t)(frand()*120000.0f));
}

static void scheduleNextGesture() {
  nextGestMs = millis() + expMsRand(15000.0f * (1.0f + (100.0f-P.energy)/120.0f));
}
static void startGesture() {
  gestureOn = true;
  gestEndMs = millis() + (3000u + (uint32_t)(frand()*3000.0f));
}
static void endGesture() { gestureOn = false; }

// -----------------------------
// Pad/Sub/Pluck/Perc helpers
// -----------------------------
static void padSetFreq(float hz) {
  float cents = 3.0f + 7.0f*(P.energy*0.01f);
  float spread = powf(2.0f, cents/1200.0f);
  for (int i=0;i<PAD_UNISON;i++){
    float r = ( (i-(PAD_UNISON-1)*0.5f) / (PAD_UNISON*0.5f) );
    float f = hz * powf(spread, r);
    pad.v[i].inc = TAU_F * f * DT;
    pad.v[i].pan = 0.5f + 0.5f * r * stereoWidth;
    pad.v[i].drift = (frand()*2.0f-1.0f)*0.0002f;
    pad.v[i].driftRate = (frand()*0.00005f + 0.00002f);
  }
}
static void subSetFreq(float hz) { sub.inc = TAU_F * hz * DT; }

static void triggerPluck(float hz) {
  for (int i=0;i<PLUCK_N;i++){
    if (!plk[i].on) {
      plk[i].on = true;
      plk[i].env = 0.0f;
      plk[i].phase = 0.0f;
      plk[i].inc = TAU_F * hz * DT;
      plk[i].pan = 0.4f + 0.2f*frand();
      DSP::lp_set(plk[i].lp, min(6000.0f, pad.cutoff*1.2f), SR);
      DSP::hp_set(plk[i].hp, 140.0f, SR);
      return;
    }
  }
}
static void triggerPerc() {
  perc.on = true;
  perc.env = 0.0f;
  DSP::hp_set(perc.hp, 2000.0f + 2000.0f*(P.tone*0.01f), SR);
}
static void airUpdate() {
  DSP::hp_set(air.hp, 4200.0f, SR);
  DSP::lp_set(air.lp, 9500.0f, SR);
  air.amt = 0.05f + 0.09f*(P.space*0.01f);
}

static inline void tiltEQ(float &l, float &r) {
  static HP1 hpL, hpR; static bool init=false;
  if (!init){ DSP::hp_set(hpL, 6000.0f, SR); DSP::hp_set(hpR, 6000.0f, SR); init=true; }
  float hl = DSP::hp_process(hpL, l);
  float hr = DSP::hp_process(hpR, r);
  float g = tilt / 6.0f;
  l = l*(1.0f - g) + hl*g;
  r = r*(1.0f - g) + hr*g;
}

// -----------------------------
// Event scheduler
// -----------------------------
static void scheduleGridIfNeeded() {
  if (sampleCounter >= nextSubdivSample) {
    float spb = 60.0f / P.tempo;
    subdivDivisor = (P.energy > 55.0f && frand()<0.5f) ? 16 : 8;
    float sec = spb / (subdivDivisor/4.0f);
    float jitter = (frand()*2.0f-1.0f) * humanizeFrac * sec * 0.15f;
    uint32_t step = (uint32_t)((sec + jitter) * SR);
    nextSubdivSample = sampleCounter + std::max<uint32_t>(64u, step);

    float baseProb = 0.10f + 0.45f*(P.energy*0.01f);
    const int8_t* steps = currentScaleSteps();

    if (frand() < 0.20f) {
      int root = BASE_KEY;
      root = quantizeToScale(root, BASE_KEY, steps);
      float hz = midi2hz(root - 12 + (int)(frand()*5.0f));
      subSetFreq(hz);
    }
    if (frand() < 0.12f) {
      int deg = steps[(int)(frand()*7.0f)%7];
      int root = BASE_KEY + deg + (int)((frand()*2.0f-1.0f)*7.0f);
      root = quantizeToScale(root, BASE_KEY, steps);
      padSetFreq(midi2hz(root));
    }
    if (frand() < baseProb*1.0f) triggerPerc();
    if (frand() < baseProb*0.45f) {
      int deg = steps[(int)(frand()*7.0f)%7];
      int octave = (frand()<0.6f) ? 1 : 2;
      int note = BASE_KEY + deg + 12*octave + (frand()<0.2f?2:0);
      note = quantizeToScale(note, BASE_KEY, steps);
      triggerPluck(midi2hz(note));
    }
  }
}

// -----------------------------
// Audio synthesis per block
// -----------------------------
static int16_t outBuf[BLOCK_SAMP*NUM_OUT_CH];

static void audioBlock() {
  uint32_t nowMs = millis();

  // Scene morph
  if (sceneAlpha < 1.0f) {
    float t = (float)(nowMs - sceneStartMs) / (float)sceneCrossMs;
    sceneAlpha = constrain(t, 0.0f, 1.0f);
    updateSceneBlend();
  }

  // Gestures
  if (!gestureOn && nowMs >= nextGestMs) { startGesture(); }
  if (gestureOn && nowMs >= gestEndMs)  { endGesture(); scheduleNextGesture(); }

  // Auto-wander
  if (nowMs >= nextWanderMs) {
    nextWanderMs = nowMs + 1000 + (uint32_t)(frand()*1000.0f);
    auto walk = [](float &x){ x = constrain(x + (frand()*2.0f-1.0f)*1.2f, 0.0f, 100.0f); };
    walk(P.mood); walk(P.energy); walk(P.space); walk(P.tone);
    applyParamMapping();
  }

  // Scene schedule
  if (nowMs >= nextSceneMs) { triggerManualScene(); scheduleNextScene(); }

  // Button
  int sw = digitalRead(ENC_SW);
  if (!sw) { if (!btnDown) { btnDown=true; btnDownMs=nowMs; } }
  else if (btnDown) {
    btnDown=false;
    if (nowMs - btnDownMs > 20 && nowMs - btnDownMs < 800) {
      if (curPage == PAGE_SCALE) { SCALE_MODE = P.scaleIdx; }
      if (curPage == PAGE_SCENE) { triggerManualScene(); }
      nextPage();
    }
  }

  // Encoder delta
  int32_t d = encDelta;
  if (d != 0) {
    encDelta = 0;
    int step = (abs(d) >= 2) ? d : (d>0?1:-1);
    editPageByDelta(step);
  }

  // Gesture boosts
  float gestCut = gestureOn ? 1.20f : 1.0f;
  float gestSaw = gestureOn ? 1.10f : 1.0f;
  float gestSend= gestureOn ? 1.15f : 1.0f;

  // Coeff updates
  DSP::lp_set(pad.lp, pad.cutoff*gestCut, SR);
  airUpdate();
  DSP::lp_set(DSP::delayToneLP, 6000.0f*(0.6f+0.8f*(P.tone*0.01f)), SR);
  for (int i=0;i<4;i++){ DSP::revC[i].damp.a = 1.0f - expf(-TAU_F*(2000.0f + 6000.0f*revDamp)/SR); }

  // Delay from tempo (3/16) clamped by DELAY_MAX_SAMP
  float spb = 60.0f / P.tempo;
  int delaySamp = (int)((spb * 0.75f) * SR);
  if (delaySamp >= DSP::DELAY_MAX_SAMP) delaySamp = DSP::DELAY_MAX_SAMP-1;

  float vol = P.volume * 0.01f;
  float peak=0.0f;

  // Startup fade over ~3s to avoid “hurt ears”
  float startFade = min(1.0f, (nowMs - bootMs) / 3000.0f);

  for (int n=0;n<BLOCK_SAMP;n++){
    scheduleGridIfNeeded();

    float padL=0, padR=0;
    pad.lfoPh += TAU_F * pad.lfoRate * DT; if (pad.lfoPh >= TAU_F) pad.lfoPh -= TAU_F;
    float ampLfo = 0.88f + 0.12f * (0.5f*(fast_sin(pad.lfoPh)+1.0f));
    for (int i=0;i<PAD_UNISON;i++){
      pad.v[i].inc *= (1.0f + pad.v[i].drift * pad.v[i].driftRate);
      pad.v[i].phase += pad.v[i].inc;
      if (pad.v[i].phase >= TAU_F) pad.v[i].phase -= TAU_F;

      float s1 = fast_sin(pad.v[i].phase);
      // gentler saw with extra softening
      float saw = (fmodf(pad.v[i].phase/TAU_F,1.0f)*2.0f - 1.0f);
      float s  = (1.0f - pad.sawMix*gestSaw)*s1 + (pad.sawMix*gestSaw)*DSP::softclip(saw*0.65f);

      float x  = DSP::lp_process(pad.lp, s);
      float l = x * (1.0f - pad.v[i].pan);
      float r = x * (pad.v[i].pan);
      padL += l; padR += r;
    }
    padL *= pad.amp * ampLfo; padR *= pad.amp * ampLfo;

    sub.phase += sub.inc; if (sub.phase >= TAU_F) sub.phase -= TAU_F;
    float subS = fast_sin(sub.phase) * sub.amp;
    padL += 0.45f*subS; padR += 0.45f*subS;

    float plL=0, plR=0;
    for (int i=0;i<PLUCK_N;i++){
      if (!plk[i].on) continue;
      if (plk[i].env < 1.0f) plk[i].env += plk[i].a;
      else                   plk[i].env -= plk[i].d*DT*SR*0.001f;
      if (plk[i].env <= 0.0001f){ plk[i].on=false; continue; }

      plk[i].phase += plk[i].inc; if (plk[i].phase >= TAU_F) plk[i].phase -= TAU_F;
      float tone = fast_sin(plk[i].phase);
      float noi  = (float)((int)xorshift32() & 0xFFFF)/32768.0f - 1.0f;
      float s = (1.0f - plk[i].noiseMix)*tone + plk[i].noiseMix*noi;
      s = DSP::hp_process(plk[i].hp, s);
      s = DSP::lp_process(plk[i].lp, s);
      s *= plk[i].env * 0.32f;
      float l = s*(1.0f - plk[i].pan);
      float r = s*(plk[i].pan);
      plL += l; plR += r;
    }

    float pr = 0.0f;
    if (perc.on) {
      if (perc.env < 1.0f) perc.env += perc.a;
      else                 perc.env -= perc.d*DT*SR*0.001f;
      if (perc.env <= 0.0005f){ perc.on=false; perc.env=0.0f; }
      float noi = (float)((int)xorshift32() & 0xFFFF)/32768.0f - 1.0f;
      pr = DSP::hp_process(perc.hp, noi) * perc.env * 0.22f;
    }

    float airN = ((int)xorshift32() & 0xFFFF)/32768.0f - 1.0f;
    float airS = DSP::lp_process(air.lp, DSP::hp_process(air.hp, airN)) * air.amt;

    float L = padL + plL + pr + airS*0.35f;
    float R = padR + plR + pr + airS*0.35f;

    float dryL=L, dryR=R;
    float send = delSend*gestSend + revMix*0.28f;
    float monoSend = (L+R)*0.5f * send;
    float dOut = DSP::delay_process(monoSend, delaySamp, delFB, 0.33f);
    float rIn  = monoSend + dOut*0.22f;
    float rOut = DSP::reverb_process(rIn);

    L = dryL + dOut*0.28f + rOut*revMix;
    R = dryR + dOut*0.28f + rOut*revMix;

    tiltEQ(L, R);
    L = DSP::dcblock_process(dcL, L);
    R = DSP::dcblock_process(dcR, R);

    // Safer master headroom + startup fade
    float master = (P.volume * 0.01f) * 1.05f * startFade;
    L = DSP::softclip(L*master);
    R = DSP::softclip(R*master);

    float p = max(fabsf(L), fabsf(R));
    if (p > peak) peak = p;

    int16_t li = (int16_t)constrain(L*32767.0f, -32767.0f, 32767.0f);
    int16_t ri = (int16_t)constrain(R*32767.0f, -32767.0f, 32767.0f);
    outBuf[2*n+0] = li;
    outBuf[2*n+1] = ri;

    sampleCounter++;
  }
  outMeter = max(peak, outMeter*0.95f);

  size_t bytesWritten = 0;
  i2s_write((i2s_port_t)0, (const char*)outBuf, BLOCK_SAMP*NUM_OUT_CH*sizeof(int16_t), &bytesWritten, portMAX_DELAY);
}

// -----------------------------
// Audio / UI tasks
// -----------------------------
static void audioTask() { audioBlock(); }
static void uiTask() {
  uint32_t now = millis();
  if (now - lastUiMs < (1000/UI_FPS)) return;
  lastUiMs = now;
  uiDrawStatus();
  uiIdleAnim(now);
}

// -----------------------------
// I2S init — switch to I2S MSB format (MAX98357A-friendly)
// -----------------------------
static void audioInit() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SR;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S_MSB;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = BLOCK_SAMP;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_BCLK;
  pins.ws_io_num  = I2S_LRC;
  pins.data_out_num = I2S_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install((i2s_port_t)0, &cfg, 0, NULL);
  i2s_set_pin((i2s_port_t)0, &pins);
  i2s_set_clk((i2s_port_t)0, SR, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

// -----------------------------
// Reverb init tiny
// -----------------------------
static void reverbInit() {
  using namespace DSP;
  revC[0] = {revBuf1, (int)(sizeof(revBuf1)/sizeof(int16_t)), 0, 0.78f, {}, 0.0f};
  revC[1] = {revBuf2, (int)(sizeof(revBuf2)/sizeof(int16_t)), 0, 0.80f, {}, 0.0f};
  revC[2] = {revBuf3, (int)(sizeof(revBuf3)/sizeof(int16_t)), 0, 0.79f, {}, 0.0f};
  revC[3] = {revBuf4, (int)(sizeof(revBuf4)/sizeof(int16_t)), 0, 0.77f, {}, 0.0f};
  for (int i=0;i<4;i++){ lp_set(revC[i].damp, 6000.0f, SR); }

  revA[0] = {apBuf1, (int)(sizeof(apBuf1)/sizeof(int16_t)), 0, 0.7f};
  revA[1] = {apBuf2, (int)(sizeof(apBuf2)/sizeof(int16_t)), 0, 0.7f};
}

// -----------------------------
// TFT init
// -----------------------------
static void tftInit() {
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1); // 128x160
  tft.setTextWrap(false);
  uiDrawStatic();
  uiDrawStatus();
}

// -----------------------------
// Encoder init
// -----------------------------
static void encInit() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  lastAB = ((digitalRead(ENC_A))<<1) | (digitalRead(ENC_B));
  attachInterrupt(digitalPinToInterrupt(ENC_A), isr_encA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isr_encB, CHANGE);
}

// -----------------------------
// Setup / Loop
// -----------------------------
void setup() {
  for (int i=0;i<SINE_TBL_SZ;i++){
    sineTable[i] = sinf((TAU_F*i)/SINE_TBL_SZ);
  }
  rngState ^= (uint32_t)esp_random();

  audioInit();
  tftInit();
  encInit();
  reverbInit();
  DSP::lp_set(DSP::delayToneLP, 6000.0f, SR);
  dcL = {0.997f,0,0}; dcR = {0.997f,0,0};

  P.mood=50; P.energy=40; P.space=35; P.tone=50; P.scaleIdx=SCALE_AEOLIAN; P.tempo=104; P.volume=75;
  applyParamMapping();

  padSetFreq(midi2hz(BASE_KEY));
  subSetFreq(midi2hz(BASE_KEY-12));

  scheduleNextScene();
  scheduleNextGesture();
  nextWanderMs = millis() + 1200;

  bootMs = millis();

  uiDrawStatus();
}

void loop() {
  audioTask();
  uiTask();
}

/* Notes on fixes:
   - DRAM: main reductions are DELAY_MAX_SAMP (0.55s), smaller reverb buffers, sine table 1024.
   - Noise: use I2S_COMM_FORMAT_I2S_MSB, reduce saw level & master gain, add 3s fade-in, clamp sawMix.
   - If your MAX98357A is wired for left or right only, stereo interleaved still works; module sums internally.
*/

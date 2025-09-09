// === ESP32 + MAX98357A + AudioTools + Maximilian: Optimized for 44.1kHz ===
// Pins (your fixed wiring)
#define I2S_DOUT 22   // MAX98357A DIN
#define I2S_BCLK 26   // BCLK
#define I2S_LRC  25   // LRCLK/WS

#include <Arduino.h>
#include "AudioTools.h"
#include "AudioTools/AudioLibs/MaximilianDSP.h"
#include <maximilian.h>

// I2S sink
I2SStream i2s;

// ===== Synth blocks =====
static maxiOsc osc;     // main oscillator
static maxiEnv env;     // amplitude envelope
static maxiFilter lp;   // lowpass filter

// ===== Arp state =====
static const double chordFreqs[] = {
  261.63, 329.63, 392.00, 493.88  // C4 E4 G4 B4 (Cmaj7)
};
static const int chordLen = sizeof(chordFreqs) / sizeof(double);

volatile bool gate = false;
volatile double currentFreq = chordFreqs[0];

// OPTIMIZATION 1: Use faster timing calculations
const uint32_t noteMs = 360;
const uint32_t gateOnMs = 18;
const float outGain = 0.6f;  // Reduced gain to prevent clipping
const float lpCutMin = 1200.0f;
const float lpCutMax = 3000.0f;
float lpCut = lpCutMin;
int arpIndex = 0;

uint32_t lastNoteTick = 0;
uint32_t gateTick = 0;

// OPTIMIZATION 2: Simplified pad with control-rate updates
static maxiOsc padA, padB;
static maxiOsc lfoCut;  // Removed lfoAmp for CPU savings
static maxiEnv padEnv;
static maxiFilter padLP;

// Pre-calculated pad values (updated at control rate)
static double padF1 = 130.0;
static double padF2 = 131.0;
static double padCutoffCurrent = 900.0;
static double padLevel = 0.15;  // Reduced level
static double padRes = 0.6;     // Reduced resonance

// Control rate counter (update pad every N samples)
static int controlRateCounter = 0;
const int CONTROL_RATE_DIVIDER = 128;  // Update every 128 samples (~344Hz at 44.1kHz)

// OPTIMIZATION 3: Pre-calculated detune frequencies
static uint32_t nextPadUpdate = 0;

// ===== OPTIMIZED audio callback =====
void play(maxi_float_t* ch) {
  // OPTIMIZATION 4: Minimize calculations in audio callback
  
  // Main voice (simplified)
  double envOut = env.adsr(1.0, gate ? 1 : 0);
  double voice = osc.saw(currentFreq);
  double voiceFiltered = lp.lores(voice, lpCut, 1.0);
  double mainOut = voiceFiltered * envOut * outGain;
  
  // OPTIMIZATION 5: Control-rate pad updates
  double padOut = 0.0;
  if (controlRateCounter++ >= CONTROL_RATE_DIVIDER) {
    controlRateCounter = 0;
    
    // Update pad parameters at control rate only
    double cutMod = 1.0 + 0.08 * lfoCut.sinewave(0.015);
    padCutoffCurrent = 900.0 * cutMod;
    if (padCutoffCurrent < 200.0) padCutoffCurrent = 200.0;
    if (padCutoffCurrent > 4000.0) padCutoffCurrent = 4000.0;
  }
  
  // Generate pad (simplified - no per-sample modulation)
  double pad = 0.5 * (padA.sinewave(padF1) + padB.sinewave(padF2));
  double padFiltered = padLP.lores(pad, padCutoffCurrent, padRes);
  double padEnvOut = padEnv.adsr(1.0, 1);
  padOut = padFiltered * padEnvOut * padLevel;
  
  // OPTIMIZATION 6: Simplified mixing and clipping
  double mixOut = mainOut + padOut;
  
  // Fast soft clipping
  if (mixOut > 0.8f) mixOut = 0.8f;
  if (mixOut < -0.8f) mixOut = -0.8f;
  
  ch[0] = mixOut;
  ch[1] = mixOut;
}

audio_tools::Maximilian maxiEngine(i2s, 2048, play);  // OPTIMIZATION 7: Larger buffer

void setup() {
  Serial.begin(115200);
  
  // OPTIMIZATION 8: ESP32 performance settings
  setCpuFrequencyMhz(240);  // Max CPU frequency
  
  // I2S config for 44.1kHz
  auto cfg = i2s.defaultConfig(TX_MODE);
  cfg.sample_rate = 44100;      // Now using 44.1kHz
  cfg.bits_per_sample = 16;
  cfg.channels = 2;
  cfg.pin_data = I2S_DOUT;
  cfg.pin_bck = I2S_BCLK;
  cfg.pin_ws = I2S_LRC;
  i2s.begin(cfg);
  
  // OPTIMIZATION 9: Larger I2S buffer
  i2s.setWriteBufferSize(8192);  // Increased buffer size
  
  // Start Maximilian engine
  maxiEngine.begin(AudioInfo(cfg.sample_rate, cfg.channels, cfg.bits_per_sample));
  maxiEngine.setVolume(0.8f);
  
  // OPTIMIZATION 10: Faster envelope settings
  env.setAttack(2);    // Shorter attack
  env.setDecay(50);    // Shorter decay
  env.setSustain(0.12);
  env.setRelease(80);  // Shorter release
  
  // Pad envelope (longer but simpler)
  padEnv.setAttack(2000);
  padEnv.setDecay(3000);
  padEnv.setSustain(0.6);
  padEnv.setRelease(4000);
  
  // Pre-calculate pad frequencies
  double padBase = 130.81;
  double detune = 0.006;
  padF1 = padBase * (1.0 - detune);
  padF2 = padBase * (1.0 + detune);
  
  lpCut = lpCutMin;
  lastNoteTick = millis();
  gate = true;
  gateTick = lastNoteTick;
  
  Serial.println("Audio engine started at 44.1kHz");
}

void loop() {
  const uint32_t now = millis();
  
  // Arp sequencing (unchanged timing logic)
  if (now - lastNoteTick >= noteMs) {
    lastNoteTick = now;
    
    arpIndex = (arpIndex + 1) % chordLen;
    currentFreq = chordFreqs[arpIndex];
    
    gate = true;
    gateTick = now;
    
    // Smoother cutoff movement
    lpCut += 60.0f;
    if (lpCut > lpCutMax) lpCut = lpCutMin;
  }
  
  // OPTIMIZATION 11: Less frequent pad updates
  if (now >= nextPadUpdate) {
    // Smaller, less frequent changes
    double basePad = 130.81;
    double detuneAmount = 0.006 + (random(-1, 2) * 0.0002);
    if (detuneAmount < 0.004) detuneAmount = 0.004;
    if (detuneAmount > 0.010) detuneAmount = 0.010;
    
    padF1 = basePad * (1.0 - detuneAmount);
    padF2 = basePad * (1.0 + detuneAmount);
    
    nextPadUpdate = now + 8000 + random(0, 4000);  // 8-12 seconds
  }
  
  // Gate timing
  if (gate && (now - gateTick >= gateOnMs)) {
    gate = false;
  }
  
 
  
  // Pump audio
  maxiEngine.copy();
}
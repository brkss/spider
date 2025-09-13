// === ESP32 + MAX98357A + AudioTools + Maximilian: Enhanced Ambient Pad ===
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

// Enhanced ambient pad system
static maxiOsc padA, padB, padC;      // 3 oscillators for richer texture
static maxiOsc lfoCut, lfoAmp, lfoPitch;  // Multiple modulation sources
static maxiEnv padEnv;
static maxiFilter padLP, padHP;       // Lowpass + highpass for more shaping

// ===== Arp state =====
static const double chordFreqs[] = {
  261.63, 329.63, 392.00, 493.88  // C4 E4 G4 B4 (Cmaj7)
};
static const int chordLen = sizeof(chordFreqs) / sizeof(double);

volatile bool gate = false;
volatile double currentFreq = chordFreqs[0];

// Timing and control parameters
const uint32_t noteMs = 360;
const uint32_t gateOnMs = 18;
const float outGain = 0.6f;  // Reduced gain to prevent clipping
const float lpCutMin = 1200.0f;
const float lpCutMax = 3000.0f;
float lpCut = lpCutMin;
int arpIndex = 0;

uint32_t lastNoteTick = 0;
uint32_t gateTick = 0;

// Pre-calculated pad values (updated at control rate)
static double padF1 = 130.0;
static double padF2 = 131.0;
static double padF3 = 65.5;           // Sub-octave for depth
static double padCutoffCurrent = 1200.0;
static double padCutoffBase = 1200.0; // Base cutoff for evolution
static double padBaseDetune = 0.008;  // Base detune amount
static double padLevel = 0.35;        // Increased level for presence
static double padRes = 1.2;           // Higher resonance for character
static double padAmpMod = 1.0;        // Amplitude modulation
static double padPitchMod = 1.0;      // Pitch modulation

// Control rate counter (update pad every N samples)
static int controlRateCounter = 0;
const int CONTROL_RATE_DIVIDER = 64;  // More frequent updates for liveliness

// Pad evolution parameters
static uint32_t nextPadUpdate = 0;

// ===== OPTIMIZED audio callback =====
void play(maxi_float_t* ch) {
  // Main voice (simplified)
  double envOut = env.adsr(1.0, gate ? 1 : 0);
  double voice = osc.saw(currentFreq);
  double voiceFiltered = lp.lores(voice, lpCut, 1.0);
  double mainOut = voiceFiltered * envOut * outGain;
  
  // Enhanced control-rate pad updates with life
  double padOut = 0.0;
  if (controlRateCounter++ >= CONTROL_RATE_DIVIDER) {
    controlRateCounter = 0;
    
    // Multi-layered modulation for organic movement
    double cutMod = 1.0 + 0.25 * lfoCut.sinewave(0.012);      // Slower, deeper cuts
    double ampMod = 0.7 + 0.3 * lfoAmp.triangle(0.027);       // Breathing amplitude
    double pitchMod = 1.0 + 0.003 * lfoPitch.sinewave(0.041); // Subtle pitch drift
    
    padCutoffCurrent = padCutoffBase * cutMod;
    if (padCutoffCurrent < 400.0) padCutoffCurrent = 400.0;
    if (padCutoffCurrent > 6000.0) padCutoffCurrent = 6000.0;
    
    padAmpMod = ampMod;
    padPitchMod = pitchMod;
  }
  
  // Generate rich, layered pad texture
  double pad1 = padA.sinewave(padF1 * padPitchMod);           // Main tone
  double pad2 = padB.triangle(padF2 * padPitchMod);           // Slight harmonic variation
  double pad3 = padC.sinewave(padF3 * padPitchMod * 0.99);    // Sub-bass with slight detune
  
  // Mix the three layers with different characters
  double padMix = (0.4 * pad1) + (0.35 * pad2) + (0.25 * pad3);
  
  // Dual filtering for more sculpted sound
  double padFiltered = padLP.lores(padMix, padCutoffCurrent, padRes);
  padFiltered = padHP.hires(padFiltered, 80.0, 0.5);          // Remove mud
  
  // Apply envelope and amplitude modulation
  double padEnvOut = padEnv.adsr(1.0, 1);
  padOut = padFiltered * padEnvOut * padLevel * padAmpMod;
  
  // Simplified mixing and clipping
  double mixOut = mainOut + padOut;
  
  // Fast soft clipping
  if (mixOut > 0.8f) mixOut = 0.8f;
  if (mixOut < -0.8f) mixOut = -0.8f;
  
  ch[0] = mainOut;
  ch[1] = mainOut;
}

audio_tools::Maximilian maxiEngine(i2s, 2048, play);

void setup() {
  Serial.begin(115200);
  
  // ESP32 performance settings
  setCpuFrequencyMhz(240);  // Max CPU frequency
  
  // I2S config for 44.1kHz
  auto cfg = i2s.defaultConfig(TX_MODE);
  cfg.sample_rate = 44100;      // Now using 44.1kHz
  cfg.bits_per_sample = 16;
  cfg.channels = 1;
  cfg.pin_data = I2S_DOUT;
  cfg.pin_bck = I2S_BCLK;
  cfg.pin_ws = I2S_LRC;
  i2s.begin(cfg);
  
  // Larger I2S buffer
  i2s.setWriteBufferSize(8192);
  
  // Start Maximilian engine
  maxiEngine.begin(AudioInfo(cfg.sample_rate, cfg.channels, cfg.bits_per_sample));
  maxiEngine.setVolume(0.8f);
  
  // Faster envelope settings
  env.setAttack(2);    // Shorter attack
  env.setDecay(50);    // Shorter decay
  env.setSustain(0.12);
  env.setRelease(80);  // Shorter release
  
  // Pad envelope (longer, more atmospheric)
  padEnv.setAttack(3500);   // Slow, dreamy attack
  padEnv.setDecay(5000);    // Long decay
  padEnv.setSustain(0.75);  // Higher sustain for presence
  padEnv.setRelease(8000);  // Very long release for ambient tail
  
  // Initialize pad frequencies with complex relationships
  double padBase = 130.81;  // C3
  padF1 = padBase * (1.0 - padBaseDetune);           // Slightly flat
  padF2 = padBase * (1.0 + padBaseDetune * 1.3);     // More sharp
  padF3 = padBase * 0.5 * (1.0 + padBaseDetune * 0.7); // Sub-octave with detune
  
  lpCut = lpCutMin;
  lastNoteTick = millis();
  gate = true;
  gateTick = lastNoteTick;
  
  Serial.println("Enhanced ambient audio engine started at 44.1kHz");
}

void loop() {
  const uint32_t now = millis();
  
  // Arp sequencing
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
  
  // More dynamic and frequent pad evolution
  if (now >= nextPadUpdate) {
    // More dramatic changes for evolution
    padCutoffBase += random(-150, 151);  // ±150 Hz jumps
    if (padCutoffBase < 800) padCutoffBase = 800;
    if (padCutoffBase > 2800) padCutoffBase = 2800;
    
    // Evolving detune for organic drift
    padBaseDetune += (random(-3, 4) * 0.0005);  // ±0.15%
    if (padBaseDetune < 0.005) padBaseDetune = 0.005;
    if (padBaseDetune > 0.015) padBaseDetune = 0.015;
    
    // Recalculate frequencies
    double basePad = 130.81;
    padF1 = basePad * (1.0 - padBaseDetune);
    padF2 = basePad * (1.0 + padBaseDetune * 1.3);
    padF3 = basePad * 0.5 * (1.0 + padBaseDetune * 0.7);
    
    // More frequent updates for liveliness
    nextPadUpdate = now + 4000 + random(0, 3000);  // 4-7 seconds
    
    Serial.printf("Pad evolved: cutoff=%.1f, detune=%.4f\n", padCutoffBase, padBaseDetune);
  }
  
  // Gate timing
  if (gate && (now - gateTick >= gateOnMs)) {
    gate = false;
  }
  
  // Pump audio
  maxiEngine.copy();
}
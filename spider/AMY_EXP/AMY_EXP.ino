// AMY official example — adapted for ESP32-WROOM + MAX98357A
// Pins: DIN=22, BCLK=26, LRC/WS=25 (no MCLK needed for MAX98357)
// Python-style AMY implementation with volume control, KY-040 encoder, and soft note playback

#include <AMY-Arduino.h>

// KY-040 Encoder pins
#define ENC_A 27
#define ENC_B 14
#define ENC_SW 13

// --- Optional safety: force I2S path on some builds ---
// #define AMY_PLATFORM_ESP32 1
// #define AMY_USE_I2S 1

// Encoder variables
volatile int encoderValue = 0;
volatile int lastEncoderValue = 0;
volatile bool encoderChanged = false;

// Filter control variables
float currentFilterFreq = 800.0;
float currentResonance = 2.5;
const float MIN_FILTER_FREQ = 100.0;
const float MAX_FILTER_FREQ = 8000.0;

// Note playback variables
unsigned long lastNoteTime = 0;
const unsigned long NOTE_INTERVAL = 500; // 50ms between notes
bool notePlaying = false;
const int NOTE_MIDI = 60; // Middle C
const int SOFT_PATCH = 1; // Soft patch number

// Python amy.send() equivalent functions
void amy_send_patch(uint8_t patch_number, uint8_t synth, uint8_t num_voices) {
  amy_event e = amy_default_event();
  e.synth = synth;
  e.patch_number = patch_number;
  e.num_voices = num_voices;
  amy_add_event(&e);
}

void amy_send_note(uint8_t synth, float velocity, uint8_t midi_note) {
  amy_event e = amy_default_event();
  e.synth = synth;
  e.velocity = velocity;
  e.midi_note = midi_note;
  e.voices[0] = 0;  // Use first voice
  amy_add_event(&e);
}

void amy_send_filter(uint8_t synth, float filter_freq, float resonance) {
  amy_event e = amy_default_event();
  e.synth = synth;
  e.filter_freq_coefs[COEF_CONST] = filter_freq;  // Set constant filter frequency
  e.resonance = resonance;
  e.filter_type = FILTER_LPF;  // Set as low-pass filter
  amy_add_event(&e);
}

// NEW: Volume control functions
void amy_send_volume(uint8_t synth, float volume) {
  amy_event e = amy_default_event();
  e.synth = synth;
  e.volume = volume;  // 0.0 = silent, 1.0 = full volume
  amy_add_event(&e);
}

void amy_send_amp_coefs(uint8_t synth, float amp_constant, float amp_note = 0.0, float amp_velocity = 0.0) {
  amy_event e = amy_default_event();
  e.synth = synth;
  e.amp_coefs[COEF_CONST] = amp_constant;   // Base amplitude
  e.amp_coefs[COEF_NOTE] = amp_note;         // Amplitude change per note
  e.amp_coefs[COEF_VEL] = amp_velocity;      // Amplitude change per velocity
  amy_add_event(&e);
}

// Note playback function
void playNoteSequence() {
  if (millis() - lastNoteTime >= NOTE_INTERVAL) {
    if (notePlaying) {
      // Stop the note
      amy_send_note(1, 0, NOTE_MIDI);
      notePlaying = false;
    } else {
      // Play the note
      amy_send_note(1, 20, NOTE_MIDI); // Velocity 20 for soft sound
      notePlaying = true;
    }
    lastNoteTime = millis();
  }
}

// Encoder interrupt handler
void IRAM_ATTR encoderISR() {
  static uint8_t lastA = 0;
  static uint8_t lastB = 0;
  
  uint8_t currentA = digitalRead(ENC_A);
  uint8_t currentB = digitalRead(ENC_B);
  
  if (lastA != currentA || lastB != currentB) {
    if (lastA == 0 && currentA == 1) {
      if (currentB == 0) {
        encoderValue++;
      } else {
        encoderValue--;
      }
      encoderChanged = true;
    }
    
    lastA = currentA;
    lastB = currentB;
  }
}

// Update filter based on encoder value
void updateFilterFromEncoder() {
  if (encoderChanged) {
    // Map encoder value to filter frequency (logarithmic for better control)
    float normalizedValue = (encoderValue - lastEncoderValue) * 0.1; // Sensitivity
    currentFilterFreq *= exp(normalizedValue);
    
    // Clamp to valid range
    if (currentFilterFreq < MIN_FILTER_FREQ) currentFilterFreq = MIN_FILTER_FREQ;
    if (currentFilterFreq > MAX_FILTER_FREQ) currentFilterFreq = MAX_FILTER_FREQ;
    
    // Apply the new filter settings
    amy_send_filter(1, currentFilterFreq, currentResonance);
    
    // Debug output (optional)
    Serial.print("Filter Freq: ");
    Serial.print(currentFilterFreq);
    Serial.print(" Hz, Encoder: ");
    Serial.println(encoderValue);
    
    lastEncoderValue = encoderValue;
    encoderChanged = false;
  }
}

// Test functions with volume control
void test_python_equivalents() {
  // Python: amy.send(patch_number=1, synth=1, num_voices=4) - Soft patch
  amy_send_patch(SOFT_PATCH, 1, 4);
  
  // Python: amy.send(synth=1, vel=20, note=60) - Soft velocity, middle C
  amy_send_note(1, 20, NOTE_MIDI);
  
  // Python: amy.send(synth=1, filter_freq=[800], resonance=2.5)
  amy_send_filter(1, currentFilterFreq, currentResonance);
  
  // NEW: Set volume to 70% for soft sound
  amy_send_volume(1, 0.7);
  
  // NEW: Set amplitude coefficients for soft sound
  amy_send_amp_coefs(1, 0.5, 0.0, 0.8);  // 50% base, no note scaling, moderate velocity response
}

void setup() {
  // Initialize serial for debug output
  Serial.begin(115200);
  Serial.println("AMY with KY-040 Encoder Control and Soft Note Sequence");
  
  // Initialize encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  
  // Attach encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  
  amy_config_t amy_config = amy_default_config();

  // Nice to have features
  amy_config.features.startup_bleep = 1;
  amy_config.features.default_synths = 1;

  // No UART MIDI - keep none to avoid needing RX/TX wiring
  amy_config.midi = AMY_MIDI_IS_NONE;

  // ---- I2S pins for ESP32-WROOM + MAX98357A ----
  amy_config.i2s_mclk = -1;      // MAX98357 doesn't need MCLK
  amy_config.i2s_bclk = 26;      // BCLK
  amy_config.i2s_lrc  = 25;      // LRCLK/WS
  amy_config.i2s_dout = 22;      // DATA/DIN

  amy_start(amy_config);
  amy_live_start();

  // Set up soft sound
  amy_send_volume(1, 0.7);  // 70% volume for soft sound

  // Test the Python equivalents with volume control
  //test_python_equivalents();
  
  Serial.println("Setup complete. Twist the encoder to control filter cutoff!");
  Serial.println("Soft notes will play every 50ms on MIDI note 60 (middle C)");
}

void loop() {
  // Required: service AMY timing
  amy_update();
  
  // Update filter based on encoder changes
  updateFilterFromEncoder();
  
  // Play note sequence every 50ms
  playNoteSequence();
  
  // Optional: Add button functionality for encoder switch
  if (digitalRead(ENC_SW) == LOW) {
    // Button pressed - could reset filter, change resonance, etc.
    static unsigned long lastButtonPress = 0;
    if (millis() - lastButtonPress > 200) { // Debounce
      // Reset filter to default
      currentFilterFreq = 800.0;
      amy_send_filter(1, currentFilterFreq, currentResonance);
      Serial.println("Filter reset to 800 Hz");
      lastButtonPress = millis();
    }
  }
}
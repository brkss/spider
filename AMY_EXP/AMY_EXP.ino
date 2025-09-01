// AMY official example — adapted for ESP32-WROOM + MAX98357A
// Pins: DIN=22, BCLK=26, LRC/WS=25 (no MCLK needed for MAX98357)
// Python-style AMY implementation with volume control and KY-040 encoder

#include <AMY-Arduino.h>

// KY-040 Encoder pins
#define ENCODER_PIN_A 27
#define ENCODER_PIN_B 14
#define ENCODER_PIN_SWITCH 13

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

// Drone configuration constants
const int DRONE_PATCH = 0;
const int DRONE_NUM_VOICES = 6;
const int DRONE_MIDI_NOTE = 50;
const float DRONE_VELOCITY = 15.0;
const float DRONE_VOLUME = 0.5;
const float DRONE_AMP_BASE = 0.6;
const float DRONE_AMP_VELOCITY_RESPONSE = 1.0;

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

// Volume control functions
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

// Start main drone with configured parameters
void startMainDrone() {
  // Set patch with 6 voices
  amy_send_patch(DRONE_PATCH, 1, DRONE_NUM_VOICES);
  
  // Start drone note
  amy_send_note(1, DRONE_VELOCITY, DRONE_MIDI_NOTE);
  
  // Apply filter settings
  amy_send_filter(1, currentFilterFreq, currentResonance);
  
  // Set volume to 50%
  amy_send_volume(1, DRONE_VOLUME);
  
  // Set amplitude coefficients for more control
  amy_send_amp_coefs(1, DRONE_AMP_BASE, 0.0, DRONE_AMP_VELOCITY_RESPONSE);
}

// Encoder interrupt handler
void IRAM_ATTR encoderISR() {
  static uint8_t lastA = 0;
  static uint8_t lastB = 0;
  
  uint8_t currentA = digitalRead(ENCODER_PIN_A);
  uint8_t currentB = digitalRead(ENCODER_PIN_B);
  
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
    
    // Debug output
    Serial.print("Filter Freq: ");
    Serial.print(currentFilterFreq);
    Serial.print(" Hz, Encoder: ");
    Serial.println(encoderValue);
    
    lastEncoderValue = encoderValue;
    encoderChanged = false;
  }
}

void setup() {
  // Initialize serial for debug output
  Serial.begin(115200);
  Serial.println("AMY with KY-040 Encoder Control and Main Drone");
  
  // Initialize encoder pins
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_PIN_SWITCH, INPUT_PULLUP);
  
  // Attach encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);
  
  amy_config_t amy_config = amy_default_config();

  // Nice to have features
  amy_config.features.startup_bleep = 1;
  amy_config.features.default_synths = 1;

  // No UART MIDI - keep none to avoid needing RX/TX wiring
  amy_config.midi = AMY_MIDI_IS_NONE;

  // ---- I2S pins for ESP32-WROOM -> MAX98357A ----
  amy_config.i2s_mclk = -1;      // MAX98357 doesn't need MCLK
  amy_config.i2s_bclk = 26;      // BCLK
  amy_config.i2s_lrc  = 25;      // LRCLK/WS
  amy_config.i2s_dout = 22;      // DATA/DIN

  amy_start(amy_config);
  amy_live_start();

  // Set initial volume
  amy_send_volume(1, 1.0);  // 100% volume

  // Start the main drone
  startMainDrone();
  
  Serial.println("Setup complete. Twist the encoder to control filter cutoff!");
  Serial.println("Main drone playing on MIDI note 50 with 6 voices");
}

void loop() {
  // Required: service AMY timing
  amy_update();
  
  // Update filter based on encoder changes
  updateFilterFromEncoder();
  
  // Optional: Add button functionality for encoder switch
  if (digitalRead(ENCODER_PIN_SWITCH) == LOW) {
    // Button pressed - reset filter to default
    static unsigned long lastButtonPress = 0;
    if (millis() - lastButtonPress > 200) { // Debounce
      currentFilterFreq = 800.0;
      amy_send_filter(1, currentFilterFreq, currentResonance);
      Serial.println("Filter reset to 800 Hz");
      lastButtonPress = millis();
    }
  }
}
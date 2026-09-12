/*
  KEYFRAME MORPHER  --  four stored scenes, one knob that sweeps between them, for the HAGIWO MOD1
  Released under CC0.  MOD1 hardware by HAGIWO (https://note.com/solder_state/n/nc05d8e8fd311)

  Inspired by the interface of Mutable Instruments Frames.  Not affiliated with or endorsed by
  Mutable Instruments; this is an independent DIY firmware for different hardware.

  Four scenes are stored, each holding one value for each of the three outputs.  POT1 sweeps
  continuously from scene 1 to scene 4 and the outputs interpolate, so one gesture drags three
  destinations along three completely different paths.  That divergence is the whole point: with
  the factory scenes, F2 rises straight, F3 zig-zags through two full traversals and F4 falls
  through a plateau, all from the same knob.

  Frames has four channel knobs on top of its big one.  MOD1 has three pots and POT1 is spent on
  position, so the twelve stored values are reached by holding the button instead.

  --Pin assign---
  POT1  A0   position, scene 1 .. scene 4
  POT2  A1   nothing while playing.  Held button: picks the channel to edit
  POT3  A2   nothing while playing.  Held button: sets that channel's value
  F1    A3   position CV in, 0..5V.  Takes over from POT1 once something is patched
  F2    D9   channel 1 out, 0..5V   (OC1A)
  F3    D10  channel 2 out, 0..5V   (OC1B)
  F4    D11  channel 3 out, 0..5V   (OC2A)
  BUTTON D4  hold to edit the nearest scene.  A short tap does nothing
  LED   D3   channel 1 output, or the value being edited while the button is held
  EEPROM     twelve scene bytes at 0..11, a version stamp at 12

  Holding the button latches the scene nearest where POT1 is sitting and hands POT2 and POT3 to
  that scene.  Neither pot does anything until it has been deliberately turned, so holding the
  button on its own cannot overwrite a stored value and neither pot's resting position is ever
  adopted.  POT2 picks the channel by thirds of its travel; POT3 then writes the value.  Changing
  the channel disarms POT3 again, so moving through the channels does not drag one pot position
  across all three.

  Position is not frozen during an edit and there is no pot pickup on POT1: the scene being edited
  was latched when the hold began, so a brushed POT1 cannot change the target, and leaving the
  engine live means a CV driven sweep keeps showing the edit in motion.

  The engine runs at 2 kHz, 8 bit out through the 62.5 kHz PWM and the 1uF/1k reconstruction
  filter on the MOD1 board, as in the other firmwares.
*/

#include <EEPROM.h>
#include <avr/pgmspace.h>

// ---------------------------------------------------------------- pins
#define PIN_LED     3
#define PIN_BUTTON  4
#define PIN_OUT1    9    // OC1A, F2
#define PIN_OUT2    10   // OC1B, F3
#define PIN_OUT3    11   // OC2A, F4

// ---------------------------------------------------------------- config
#define TICK_US          500UL   // 2 kHz engine tick
#define NUM_SCENES       4
#define NUM_CH           3

#define DEBOUNCE_MS      30UL
#define HOLD_MS          300UL   // button held this long enters edit
#define EE_SAVE_DELAY_MS 2000UL

#define ARM_DELTA        24      // pot travel needed before it takes effect
#define ZONE_HYST        12      // travel past a zone boundary before the channel changes
#define EDIT_IDLE_LED    24      // steady dim while editing, before a channel has been picked

// F1 takeover.  The MOD1 has no switched jack, so "patched" has to be inferred from a rise.
#define CV_PRESENT_LO    20      // ~0.1 V on F1
#define CV_POT_TAKEOVER  8       // POT1 counts of travel that reclaim the position

#define EE_ADDR_SCENES   0       // 12 bytes, scene major
#define EE_ADDR_VER      12
// A distinctive stamp rather than 1: a chip carrying one of the other MOD1 firmwares can easily
// have a small integer sitting at address 12, and that would read as valid scene data.
#define EE_VERSION       0x4B

// ---------------------------------------------------------------- tables
// Factory scenes, chosen so the three paths are obviously different the moment the knob is turned.
//   ch0   0  85 170 255    straight rise
//   ch1   0 255   0 255    zig-zag, two full traversals
//   ch2 255 128 128   0    fall with a plateau in the middle
static const uint8_t FACTORY[NUM_SCENES][NUM_CH] PROGMEM = {
  {   0,   0, 255 },
  {  85, 255, 128 },
  { 170,   0, 128 },
  { 255, 255,   0 }
};

// ---------------------------------------------------------------- state
static uint8_t scenes[NUM_SCENES][NUM_CH];

// non blocking ADC round robin over A0..A3
static uint16_t adcVal[4];
static uint8_t  adcCh = 0;

// engine
static uint8_t  outVal[NUM_CH];

// F1 takeover
static bool     cvActive = false;
static uint16_t cvPotLast = 0;

// button and edit mode
static uint8_t  btnLastRead = HIGH, btnStable = HIGH;
static uint32_t btnChangedMs = 0, btnDownMs = 0;
static bool     editing = false;
static uint8_t  editScene = 0;
static int8_t   editCh = -1;
static uint16_t entryPot2 = 0, entryPot3 = 0;
static bool     armedCh = false, armedVal = false;

// LED
static uint8_t  patLeft = 0, patBright = 0;
static uint16_t patOnMs = 0, patOffMs = 0;
static bool     patOn = false;
static uint32_t patNextMs = 0;

// misc
static uint32_t lastTickUs = 0;
static uint32_t eeDirtyMs = 0;
static bool     eeDirty = false;

// ---------------------------------------------------------------- helpers
// OC2B at 0 still leaks a 1/256 sliver, so the LED would never look fully off
static inline void ledSet(uint8_t v) {
  if (v == 0) {
    TCCR2A &= (uint8_t)~(1 << COM2B1);
    PORTD  &= (uint8_t)~(1 << PD3);
  } else {
    TCCR2A |= (1 << COM2B1);
    OCR2B = v;
  }
}

static void startPattern(uint8_t pulses, uint8_t bright, uint16_t onMs, uint16_t offMs) {
  patLeft = pulses; patBright = bright; patOnMs = onMs; patOffMs = offMs;
  patOn = true; patNextMs = millis() + onMs;
}

static void markDirty() { eeDirty = true; eeDirtyMs = millis(); }

static inline uint8_t lerp8(uint8_t a, uint8_t b, uint16_t s) {
  if (b >= a) return (uint8_t)(a + (uint8_t)(((uint32_t)(b - a) * s) >> 16));
  return (uint8_t)(a - (uint8_t)(((uint32_t)(a - b) * s) >> 16));
}

// Whichever of POT1 and F1 owns the position, expanded 10 bit -> 16 bit so that fully clockwise
// really lands on the last scene rather than 63/65536 short of it.
static void positionSegment(uint8_t *seg, uint16_t *frac) {
  uint16_t pos = cvActive ? adcVal[3] : adcVal[0];
  if (pos > 1023) pos = 1023;
  uint16_t pos16 = (uint16_t)((pos << 6) | (pos >> 4));
  uint32_t t = (uint32_t)pos16 * (uint32_t)(NUM_SCENES - 1);   // < 3 << 16, so seg is 0..2
  *seg  = (uint8_t)(t >> 16);
  *frac = (uint16_t)t;
}

// ---------------------------------------------------------------- setup
static void configurePWM() {
  // Timer1, pins 9 and 10: fast PWM 8 bit, no prescaler -> 62.5 kHz
  TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM12) | (1 << CS10);

  // Timer2, pins 11 and 3: fast PWM, no prescaler -> 62.5 kHz
  TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1) | (1 << COM2B1);
  TCCR2B = (1 << CS20);
}

static void loadScenes() {
  if (EEPROM.read(EE_ADDR_VER) != EE_VERSION) {
    for (uint8_t s = 0; s < NUM_SCENES; s++)
      for (uint8_t c = 0; c < NUM_CH; c++)
        EEPROM.update((int)(EE_ADDR_SCENES + s * NUM_CH + c), pgm_read_byte(&FACTORY[s][c]));
    EEPROM.update(EE_ADDR_VER, EE_VERSION);
  }
  for (uint8_t s = 0; s < NUM_SCENES; s++)
    for (uint8_t c = 0; c < NUM_CH; c++)
      scenes[s][c] = EEPROM.read((int)(EE_ADDR_SCENES + s * NUM_CH + c));
}

void setup() {
  pinMode(PIN_OUT1, OUTPUT);
  pinMode(PIN_OUT2, OUTPUT);
  pinMode(PIN_OUT3, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  configurePWM();
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = 0;

  loadScenes();

  // Prime the filtered ADC values before taking the converter over, so the first tick already has
  // real pot positions instead of walking up from zero.
  adcVal[0] = analogRead(A0);
  adcVal[1] = analogRead(A1);
  adcVal[2] = analogRead(A2);
  adcVal[3] = analogRead(A3);
  cvPotLast = adcVal[0];

  DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D);
  adcCh  = 0;
  ADMUX  = (1 << REFS0) | adcCh;
  ADCSRA |= (1 << ADSC);

  lastTickUs = micros();
}

// ---------------------------------------------------------------- ADC
static inline void serviceADC() {
  if (ADCSRA & (1 << ADSC)) return;
  uint16_t v = ADC;
  // F1 is de-jittered along with the pots: it is a position CV, so raw converter noise would show
  // up on all three outputs at once.
  adcVal[adcCh] = (uint16_t)((adcVal[adcCh] * 3UL + v) >> 2);
  adcCh = (uint8_t)((adcCh + 1) & 3);
  ADMUX = (1 << REFS0) | adcCh;
  ADCSRA |= (1 << ADSC);
}

// ---------------------------------------------------------------- F1 takeover
// A rise past ~0.1 V latches the jack in, and it then keeps the position even when the CV falls
// back to 0 V.  Unlike the F3 takeover in clepz there is no idle timeout: 0 V here is a musical,
// sustained position -- scene 1 -- and a timeout would snatch the sweep back whenever a slow LFO
// dwelt at the bottom.  Turning POT1 is the only way back.
static void serviceCvTakeover() {
  if (adcVal[3] > CV_PRESENT_LO) {
    cvActive  = true;
    cvPotLast = adcVal[0];        // track POT1 while the CV drives, so only a later turn counts
    return;
  }
  if (!cvActive) return;

  int16_t d = (int16_t)adcVal[0] - (int16_t)cvPotLast;
  if (d > CV_POT_TAKEOVER || d < -CV_POT_TAKEOVER) cvActive = false;
}

// ---------------------------------------------------------------- edit mode
// POT2's travel splits into three zones, with hysteresis so a pot resting on a boundary does not
// dither between two channels.
static uint8_t chZone(uint16_t raw, int8_t cur) {
  uint8_t z = (uint8_t)(((uint32_t)raw * NUM_CH) >> 10);
  if (z >= NUM_CH) z = NUM_CH - 1;
  if (cur < 0 || z == (uint8_t)cur) return z;
  uint8_t  hi    = (z > (uint8_t)cur) ? z : (uint8_t)cur;
  uint16_t bound = (uint16_t)(((uint32_t)hi << 10) / NUM_CH);
  if (z > (uint8_t)cur) return (raw >= bound + ZONE_HYST) ? z : (uint8_t)cur;
  return (raw + ZONE_HYST <= bound) ? z : (uint8_t)cur;
}

static void beginEdit() {
  uint8_t  seg;
  uint16_t frac;
  positionSegment(&seg, &frac);

  editing   = true;
  editScene = (frac >= 32768) ? (uint8_t)(seg + 1) : seg;   // the nearest keyframe
  editCh    = -1;
  entryPot2 = adcVal[1];
  entryPot3 = adcVal[2];
  armedCh   = false;
  armedVal  = false;
}

static void serviceEdit() {
  // POT2 picks the channel, once it has actually been turned
  int16_t d = (int16_t)adcVal[1] - (int16_t)entryPot2;
  if (!armedCh && (d >= ARM_DELTA || d <= -ARM_DELTA)) armedCh = true;
  if (armedCh) {
    uint8_t z = chZone(adcVal[1], editCh);
    if ((int8_t)z != editCh) {
      editCh = (int8_t)z;
      startPattern((uint8_t)(editCh + 1), 255, 60, 120);
      // Re-arm the value pot, so moving on to another channel does not immediately stamp POT3's
      // current position onto it.
      entryPot3 = adcVal[2];
      armedVal  = false;
    }
  }

  // POT3 writes the value, once it has actually been turned since the channel was picked
  if (editCh < 0) return;
  int16_t v = (int16_t)adcVal[2] - (int16_t)entryPot3;
  if (!armedVal && (v >= ARM_DELTA || v <= -ARM_DELTA)) armedVal = true;
  if (!armedVal) return;

  uint8_t nv = (uint8_t)(adcVal[2] >> 2);
  if (nv != scenes[editScene][editCh]) {
    scenes[editScene][editCh] = nv;
    markDirty();
  }
}

// ---------------------------------------------------------------- button
static void serviceButton() {
  uint32_t now = millis();
  uint8_t  r = digitalRead(PIN_BUTTON);

  if (r != btnLastRead) { btnLastRead = r; btnChangedMs = now; }
  else if (r != btnStable && (now - btnChangedMs) >= DEBOUNCE_MS) {
    btnStable = r;
    if (r == LOW) btnDownMs = now;      // a release shorter than HOLD_MS does nothing
    else if (editing) editing = false;
  }

  if (btnStable == LOW && !editing && (now - btnDownMs) >= HOLD_MS) beginEdit();
}

// ---------------------------------------------------------------- LED
static void serviceLED() {
  uint32_t now = millis();

  if (patLeft) {                                   // channel confirmation
    if ((int32_t)(now - patNextMs) >= 0) {
      if (patOn) { patOn = false; patNextMs = now + patOffMs; patLeft--; }
      else       { patOn = true;  patNextMs = now + patOnMs; }
    }
    ledSet(patLeft ? (patOn ? patBright : 0) : 0);
    return;
  }

  if (editing) {
    // Dim and steady until a channel is picked, then the stored value itself, so POT3 has direct
    // feedback for what it is writing.
    ledSet(editCh < 0 ? (uint8_t)EDIT_IDLE_LED : scenes[editScene][editCh]);
    return;
  }

  ledSet(outVal[0]);
}

// ---------------------------------------------------------------- engine
static void outputTick() {
  uint8_t  seg;
  uint16_t frac;
  positionSegment(&seg, &frac);

  for (uint8_t c = 0; c < NUM_CH; c++)
    outVal[c] = lerp8(scenes[seg][c], scenes[seg + 1][c], frac);

  OCR1A = outVal[0];
  OCR1B = outVal[1];
  OCR2A = outVal[2];
}

// ---------------------------------------------------------------- loop
void loop() {
  serviceADC();
  serviceButton();
  serviceCvTakeover();
  if (editing) serviceEdit();
  serviceLED();

  if (eeDirty && (millis() - eeDirtyMs) > EE_SAVE_DELAY_MS) {
    eeDirty = false;
    for (uint8_t s = 0; s < NUM_SCENES; s++)
      for (uint8_t c = 0; c < NUM_CH; c++)
        EEPROM.update((int)(EE_ADDR_SCENES + s * NUM_CH + c), scenes[s][c]);
  }

  uint32_t now = micros();
  if ((uint32_t)(now - lastTickUs) < TICK_US) return;
  lastTickUs += TICK_US;
  if ((uint32_t)(now - lastTickUs) > TICK_US * 4) lastTickUs = now;
  outputTick();
}

/*
  KRELL  --  self generating random envelope for the HAGIWO MOD1
  Released under CC0.  MOD1 hardware by HAGIWO (https://note.com/solder_state/n/nc05d8e8fd311)

  The Krell patch is a technique, not a product: a Buchla 266 Source of Uncertainty wired into a
  Maths, where one random voltage sets the rise time, a second sets the fall time, a third sets how
  long to wait before firing again, and the end of the cycle re-rolls all three.  Set it going and
  leave it.  The other MOD1 firmwares all need something patched into them to move; this one is the
  thing that makes a patch move on its own.

  --Pin assign---
  POT1  A0   time range, about 5 ms to 10 s per stage
  POT2  A1   randomness amount: how far the times and the CV scatter from where the knobs sit
  POT3  A2   rest / density, 0 up to about 8x a stage time between cycles
  F1    A3   time CV in, 0..5V, adds up to about 5 octaves of speed
  F2    D9   cycle pulse out, 10 ms, at the start of every rise   (OC1A)
  F3    D10  this cycle's random CV, held for the whole cycle     (OC1B)
  F4    D11  envelope out, 0..5V                                  (OC2A)
  BUTTON D4  short press: envelope curve, EXP / LIN / LOG.  Hold 1.5 s: re-roll and restart now
  LED   D3   follows the envelope, plus the curve confirmation flashes
  EEPROM     the curve at address 0, a version stamp at 1

  One cycle is rise, fall, then rest.  Entering the rise rolls four numbers at once: a rise time, a
  fall time, a rest time and the CV that F3 holds for the cycle.  POT2 is the width of all four
  distributions, so it is one uncertainty knob rather than four: fully anticlockwise the module is
  a metronome playing the same envelope at the same pitch forever, fully clockwise nothing repeats.

  The three times are randomised in the *index* of an exponential table, not in milliseconds, so
  the scatter is log uniform: as likely to land half as long as twice as long.  That is what makes
  it sound like a Krell patch instead of like jitter.  The rest is derived from the current stage
  time rather than being an absolute delay, so sweeping POT1 keeps the density coherent.

  POT1 and F1 are read live inside a stage, so a knob turn or a CV move bends an envelope that is
  already in flight, the way the time knobs on a Maths do.  Only the random offsets are latched.

  The engine runs at 4 kHz, 8 bit out through the 62.5 kHz PWM and the 1uF/1k reconstruction filter
  on the MOD1 board.  Faster than the other firmwares on purpose: at 2 kHz the shortest attack
  would be ten steps.
*/

#include <EEPROM.h>
#include <avr/pgmspace.h>

// ---------------------------------------------------------------- pins
#define PIN_LED     3
#define PIN_BUTTON  4
#define PIN_OUT1    9    // OC1A, F2, cycle pulse
#define PIN_OUT2    10   // OC1B, F3, per cycle random CV
#define PIN_OUT3    11   // OC2A, F4, envelope

// ---------------------------------------------------------------- config
#define TICK_US          250UL   // 4 kHz engine tick
#define PULSE_TICKS      40      // 10 ms on F2, long enough to clear the 159 Hz output filter

#define DEBOUNCE_MS      30UL
#define PRESS_VLONG_MS   1500UL  // held this long re-rolls instead of changing the curve
#define EE_SAVE_DELAY_MS 2000UL

#define RAND_SPREAD      768     // POT2 fully CW -> +-384 index counts, about +-4 octaves
#define CV_DEPTH         512     // F1 at 5 V -> about 5 octaves faster

// The rest is an offset from the current stage index rather than a time of its own.  The table is
// 11 octaves over 1024 counts, so 290 counts is 3 octaves: POT3 walks from a rest one eighth of a
// stage up to one eight times a stage.
#define REST_FAST        290
#define REST_SPAN        580
#define REST_OFF         12      // POT3 below this: no rest at all, the cycle runs continuously

#define EE_ADDR_CURVE    0
#define EE_ADDR_VER      1
// A distinctive stamp rather than 1: a chip carrying one of the other MOD1 firmwares can easily
// have a small integer sitting at address 1, and that would read as a valid curve setting.
#define EE_VERSION       0x7A

enum { C_EXP = 0, C_LIN, C_LOG, NUM_CURVES };
enum { S_RISE = 0, S_FALL, S_REST };

// ---------------------------------------------------------------- tables
// Index 0..1023 -> 32 bit phase increment per tick.  Exponential, 107374 (10 s per stage at 4 kHz)
// up to 214748365 (5 ms), a ratio of 2000 spread over 32 segments: INC_TAB[k] = 107374 * 2000^(k/32).
static const uint32_t INC_TAB[33] PROGMEM = {
     107374,    136162,    172669,    218963,
     277669,    352116,    446522,    566239,
     718054,    910571,   1154706,   1464295,
    1856888,   2354739,   2986071,   3786669,
    4801915,   6089361,   7721984,   9792332,
   12417762,  15747099,  19969066,  25322989,
   32112357,  40722028,  51640044,  65485299,
   83042616, 105307241, 133541252, 169345108,
  214748365
};

// One saturating exponential, normalised to hit both ends exactly:
// EXP_TAB[i] = 65535 * (1 - exp(-5*i/64)) / (1 - exp(-5)).  Read forwards it is the EXP rise and,
// subtracted from full scale, the EXP fall; read backwards it is the LOG pair.  One table, six curves.
static const uint16_t EXP_TAB[65] PROGMEM = {
      0,  4958,  9544, 13785, 17708, 21336, 24691, 27794,
  30663, 33317, 35772, 38042, 40142, 42083, 43879, 45540,
  47076, 48497, 49811, 51026, 52149, 53189, 54150, 55039,
  55861, 56622, 57325, 57975, 58577, 59133, 59648, 60124,
  60564, 60971, 61347, 61695, 62017, 62315, 62590, 62845,
  63081, 63298, 63500, 63686, 63859, 64018, 64165, 64302,
  64428, 64544, 64652, 64752, 64844, 64930, 65009, 65082,
  65149, 65211, 65269, 65323, 65372, 65418, 65460, 65499,
  65535
};

// ---------------------------------------------------------------- state
static uint8_t  gCurve = C_EXP;

// non blocking ADC round robin over A0..A3
static uint16_t adcVal[4];
static uint8_t  adcCh = 0;

// engine
static uint32_t phase = 0;
static uint8_t  stage = S_RISE;
static int16_t  riseOff = 0, fallOff = 0, restOff = 0;   // this cycle's latched random offsets
static uint16_t cycleCv = 32768;
static uint16_t env = 0;
static uint8_t  pulseTicks = 0;

// button
static uint8_t  btnLastRead = HIGH, btnStable = HIGH;
static uint32_t btnChangedMs = 0, btnDownMs = 0;
static bool     vlongFired = false;

// LED
static uint8_t  patLeft = 0, patBright = 0;
static uint16_t patOnMs = 0, patOffMs = 0;
static bool     patOn = false;
static uint32_t patNextMs = 0;

// misc
static uint32_t rng = 0x9E3779B9UL;
static uint32_t lastTickUs = 0;
static uint32_t eeDirtyMs = 0;
static bool     eeDirty = false;

// ---------------------------------------------------------------- helpers
static inline uint32_t rnd32() {
  rng ^= rng << 13;
  rng ^= rng >> 17;
  rng ^= rng << 5;
  return rng;
}
static inline uint16_t rnd16() { return (uint16_t)(rnd32() >> 16); }

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

// index 0..1023 -> phase increment per tick, interpolated between table entries
static uint32_t incFromIndex(int16_t idx) {
  if (idx < 0) idx = 0;
  if (idx > 1023) idx = 1023;
  uint8_t  seg = (uint8_t)(idx >> 5);
  uint8_t  f   = (uint8_t)(idx & 31);
  uint32_t a = pgm_read_dword(&INC_TAB[seg]);
  uint32_t b = pgm_read_dword(&INC_TAB[seg + 1]);
  return a + (uint32_t)(((b - a) * (uint32_t)f) >> 5);
}

// A 16 bit phase into the curve: i = p >> 10 is 0..63, so [i + 1] never runs off the array
static inline uint16_t expCurve(uint16_t p) {
  uint8_t  i = (uint8_t)(p >> 10);
  uint16_t f = (uint16_t)(p & 1023);
  uint16_t a = pgm_read_word(&EXP_TAB[i]);
  uint16_t b = pgm_read_word(&EXP_TAB[i + 1]);
  return (uint16_t)(a + (uint16_t)(((uint32_t)(b - a) * f) >> 10));
}

static inline uint16_t riseShape(uint16_t p) {
  if (gCurve == C_LIN) return p;
  if (gCurve == C_LOG) return (uint16_t)(65535 - expCurve((uint16_t)(65535 - p)));
  return expCurve(p);
}

static inline uint16_t fallShape(uint16_t p) {
  if (gCurve == C_LIN) return (uint16_t)(65535 - p);
  if (gCurve == C_LOG) return expCurve((uint16_t)(65535 - p));
  return (uint16_t)(65535 - expCurve(p));
}

// POT1 plus the F1 CV, read live so a turn or a CV move bends a stage already in flight
static inline int16_t baseIndex() {
  int16_t v = (int16_t)adcVal[0] + (int16_t)(((uint32_t)adcVal[3] * CV_DEPTH) >> 10);
  return (v > 1023) ? (int16_t)1023 : v;
}

// Uniform in the table index, which is log uniform in time: as likely to land half as long as
// twice as long.  POT2 fully anticlockwise returns exactly 0, so the metronome is exact.
static int16_t randOffset() {
  uint16_t spread = (uint16_t)(((uint32_t)adcVal[1] * RAND_SPREAD) >> 10);
  if (!spread) return 0;
  return (int16_t)((int32_t)(((uint32_t)rnd16() * spread) >> 16) - (int32_t)(spread >> 1));
}

// The same spread, widened to the full output range, around mid scale
static uint16_t rollCv() {
  uint16_t s = (uint16_t)(((uint32_t)adcVal[1] << 6) | (adcVal[1] >> 4));
  if (!s) return 32768;
  return (uint16_t)((int32_t)32768 + (int32_t)(((uint32_t)rnd16() * s) >> 16) - (int32_t)(s >> 1));
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

static void loadSettings() {
  if (EEPROM.read(EE_ADDR_VER) != EE_VERSION) {
    EEPROM.update(EE_ADDR_CURVE, C_EXP);
    EEPROM.update(EE_ADDR_VER,   EE_VERSION);
  }
  uint8_t c = EEPROM.read(EE_ADDR_CURVE);
  gCurve = (c < NUM_CURVES) ? c : (uint8_t)C_EXP;
}

static void rollCycle();

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

  uint32_t s = 0;
  for (uint8_t i = 0; i < 16; i++) s = (s << 1) ^ (uint32_t)(analogRead(A6) & 1);
  rng = s ? (s ^ micros()) : 0x9E3779B9UL;

  loadSettings();

  // Prime the filtered ADC values before taking the converter over, so the first cycle is rolled
  // from real pot positions instead of from zero.
  adcVal[0] = analogRead(A0);
  adcVal[1] = analogRead(A1);
  adcVal[2] = analogRead(A2);
  adcVal[3] = analogRead(A3);

  DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D);
  adcCh  = 0;
  ADMUX  = (1 << REFS0) | adcCh;
  ADCSRA |= (1 << ADSC);

  rollCycle();
  lastTickUs = micros();
}

// ---------------------------------------------------------------- ADC
static inline void serviceADC() {
  if (ADCSRA & (1 << ADSC)) return;
  uint16_t v = ADC;
  // F1 is de-jittered along with the pots: it is a time CV read every tick, so converter noise
  // would put a flutter on the slope of every stage.
  adcVal[adcCh] = (uint16_t)((adcVal[adcCh] * 3UL + v) >> 2);
  adcCh = (uint8_t)((adcCh + 1) & 3);
  ADMUX = (1 << REFS0) | adcCh;
  ADCSRA |= (1 << ADSC);
}

// ---------------------------------------------------------------- button
static void serviceButton() {
  uint32_t now = millis();
  uint8_t  r = digitalRead(PIN_BUTTON);

  if (r != btnLastRead) { btnLastRead = r; btnChangedMs = now; }
  else if (r != btnStable && (now - btnChangedMs) >= DEBOUNCE_MS) {
    btnStable = r;
    if (r == LOW) {                          // press
      btnDownMs  = now;
      vlongFired = false;
    } else if (!vlongFired) {                // any release before the hold fired changes the curve
      gCurve = (uint8_t)((gCurve + 1) % NUM_CURVES);
      markDirty();
      startPattern((uint8_t)(gCurve + 1), 255, 60, 120);
    }
  }

  // A long hold re-rolls and restarts the cycle, so a set of numbers you do not like can be thrown
  // away without waiting out a ten second rest.
  if (btnStable == LOW && !vlongFired && (now - btnDownMs) >= PRESS_VLONG_MS) {
    vlongFired = true;
    rollCycle();
    startPattern(1, 255, 250, 100);
  }
}

// ---------------------------------------------------------------- LED
static void serviceLED() {
  uint32_t now = millis();

  if (patLeft) {                                   // curve confirmation
    if ((int32_t)(now - patNextMs) >= 0) {
      if (patOn) { patOn = false; patNextMs = now + patOffMs; patLeft--; }
      else       { patOn = true;  patNextMs = now + patOnMs; }
    }
    ledSet(patLeft ? (patOn ? patBright : 0) : 0);
    return;
  }

  ledSet((uint8_t)(env >> 8));
}

// ---------------------------------------------------------------- engine
// Everything for one cycle is decided here, at the start of the rise, so F2 and F3 change on the
// same edge: a sample and hold or a sequencer clocked from F2 reads the matching F3 value.
static void rollCycle() {
  riseOff = randOffset();
  fallOff = randOffset();
  restOff = randOffset();
  cycleCv = rollCv();
  OCR1B   = (uint8_t)(cycleCv >> 8);       // F3, held for the whole cycle
  pulseTicks = PULSE_TICKS;                // F2 goes high
  phase = 0;
  stage = S_RISE;
}

static inline uint32_t currentInc() {
  if (stage == S_REST) {
    int16_t r = (int16_t)(((uint32_t)adcVal[2] * REST_SPAN) >> 10);
    return incFromIndex((int16_t)(baseIndex() + REST_FAST - r + restOff));
  }
  return incFromIndex((int16_t)(baseIndex() + ((stage == S_RISE) ? riseOff : fallOff)));
}

static void outputTick() {
  uint32_t prev = phase;
  phase += currentInc();
  bool     wrapped = (phase < prev);        // the stage is over
  uint16_t p = (uint16_t)(phase >> 16);

  if (stage == S_RISE) {
    if (wrapped) { env = 65535; stage = S_FALL; phase = 0; }
    else         { env = riseShape(p); }
  } else if (stage == S_FALL) {
    if (wrapped) {
      env = 0;
      // POT3 at the very bottom is a hard "no rest": the fall runs straight into the next rise,
      // which is a Maths left cycling and is also where F2 lands on the true end of cycle.
      if (adcVal[2] < REST_OFF) rollCycle();
      else { stage = S_REST; phase = 0; }
    } else {
      env = fallShape(p);
    }
  } else {
    env = 0;
    if (wrapped) rollCycle();
  }

  if (pulseTicks) pulseTicks--;
  OCR2A = (uint8_t)(env >> 8);              // F4, envelope
  OCR1A = pulseTicks ? 255 : 0;             // F2, cycle pulse
}

// ---------------------------------------------------------------- loop
void loop() {
  serviceADC();
  serviceButton();
  serviceLED();

  if (eeDirty && (millis() - eeDirtyMs) > EE_SAVE_DELAY_MS) {
    eeDirty = false;
    EEPROM.update(EE_ADDR_CURVE, gCurve);
  }

  uint32_t now = micros();
  if ((uint32_t)(now - lastTickUs) < TICK_US) return;
  lastTickUs += TICK_US;
  if ((uint32_t)(now - lastTickUs) > TICK_US * 4) lastTickUs = now;
  outputTick();
}

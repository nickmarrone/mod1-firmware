/*
  KRELL  --  self generating random envelope for the HAGIWO MOD1
  Released under CC0.  MOD1 hardware by HAGIWO (https://note.com/solder_state/n/nc05d8e8fd311)

  The Krell patch is a technique, not a product: a Buchla 266 Source of Uncertainty wired into a
  Maths, where one random voltage sets the rise time, a second sets the fall time, a third sets how
  long to wait before firing again, and the end of the cycle re-rolls all three.  Set it going and
  leave it.  The other MOD1 firmwares all need something patched into them to move; this one is the
  thing that makes a patch move on its own.

  --Pin assign---
  POT1  A0   time range, about 5 ms to 10 s per stage     | hold button: rise / fall skew
  POT2  A1   randomness amount                            | hold button: timing vs pitch balance
  POT3  A2   rest / density, 0 up to about 8x a stage     | hold button: F3 scale quantiser
  F1    A3   time CV in, 0..5V, adds up to about 5 octaves of speed
  F2    D9   cycle pulse out, 10 ms, at the start of every rise   (OC1A)
  F3    D10  this cycle's random CV, held for the whole cycle     (OC1B)
  F4    D11  envelope out, 0..5V                                  (OC2A)
  BUTTON D4  short press: envelope curve, EXP / LIN / LOG.  Hold 1.5 s: re-roll and restart now.
             Hold and turn a pot: the shift layer, which cancels both of the above.
  LED   D3   follows the envelope; the shift layer and the confirmation flashes take priority
  EEPROM     curve at 0, version stamp at 1, skew at 2, balance at 3, scale at 4

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

  Three parameters with nowhere else to go live under the button, as in clepz.  Every default is
  the behaviour of the firmware before the shift layer existed: skew centred, balance centred, no
  quantising.  A pot does nothing until it has moved, so a hold and release cannot change anything,
  and on release a pot that was moved stays frozen until it is turned back through where it began.

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

// ---------------------------------------------------------------- shift layer
// The same names and values the other five firmwares use, so the feel carries across the set.
#define ARM_DELTA        24      // pot travel before it takes effect, so a hold cannot nudge
#define PICKUP_WINDOW    12      // how close counts as catching the old position again
#define ZONE_HYST        12      // travel past a zone boundary before it flips
#define MOVE_DELTA       3       // "this is the pot I am holding", for the LED
#define EDIT_IDLE_LED    24      // in the shift layer, nothing touched yet

#define SKEW_SPAN        290     // +-3 octaves of rise / fall asymmetry, the REST_FAST scale
#define SKEW_DEAD        24      // centre deadband, so exactly symmetric is reachable
#define BAL_DEAD         24      // ditto for the balance

#define EE_ADDR_CURVE    0
#define EE_ADDR_VER      1
#define EE_ADDR_SKEW     2
#define EE_ADDR_BAL      3
#define EE_ADDR_SCALE    4
// A distinctive stamp rather than 1: a chip carrying one of the other MOD1 firmwares can easily
// have a small integer sitting at address 1, and that would read as a valid curve setting.
// Bumped from 0x7A when the shift layer added addresses 2..4; the bump re-defaults them, and every
// default is the old behaviour, so an existing krell chip sounds identical after the update.
#define EE_VERSION       0x7B

enum { C_EXP = 0, C_LIN, C_LOG, NUM_CURVES };
enum { S_RISE = 0, S_FALL, S_REST };
enum { SC_OFF = 0, SC_OCT, SC_FIFTH, SC_PENT, SC_MINOR, SC_CHROM, NUM_SCALES };

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

// Which semitones of the octave a scale allows, bit n = n semitones above the root.
static const uint16_t SCALE_MASK[NUM_SCALES] PROGMEM = {
  0x000,   // off, never read: quantCode returns early
  0x001,   // octaves
  0x081,   // fifths          0 7
  0x4A9,   // minor pentatonic 0 3 5 7 10
  0x5AD,   // natural minor    0 2 3 5 7 8 10
  0xFFF    // chromatic
};

// The 8 bit output code for each semitone of the 5 octave range: NOTE_CODE[n] = (n*255 + 30) / 60.
// Snapping to a code from this table rather than computing a voltage means the quantiser adds no
// error of its own -- what comes out is the nearest note the 8 bit output can actually make.
static const uint8_t NOTE_CODE[61] PROGMEM = {
    0,   4,   9,  13,  17,  21,  26,  30,
   34,  38,  43,  47,  51,  55,  60,  64,
   68,  72,  77,  81,  85,  89,  94,  98,
  102, 106, 111, 115, 119, 123, 128, 132,
  136, 140, 145, 149, 153, 157, 162, 166,
  170, 174, 179, 183, 187, 191, 196, 200,
  204, 208, 213, 217, 221, 225, 230, 234,
  238, 242, 247, 251, 255
};

// ---------------------------------------------------------------- state
static uint8_t  gCurve = C_EXP;
static uint16_t gSkew  = 512;            // 512 = symmetric rise and fall
static uint16_t gBal   = 512;            // 512 = timing and pitch both get all of POT2
static uint8_t  gScale = SC_OFF;

// derived from gSkew / gBal, recomputed only when they change
static int16_t  skewIdx  = 0;
static uint16_t timeGain = 1023, cvGain = 1023;

// non blocking ADC round robin over A0..A3
static uint16_t adcVal[4];
static uint8_t  adcCh = 0;

// The pots the engine actually reads.  They track adcVal except while the shift layer has them.
static uint16_t potVal[3];
static uint16_t potEntry[3], potSeen[3];
static bool     frozen[3], armed[3];
static int8_t   entrySign[3];
static int8_t   lastTouched = -1;

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
static bool     shiftUsed = false;   // this hold was spent on a pot, so release does nothing

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
  int16_t v = (int16_t)potVal[0] + (int16_t)(((uint32_t)adcVal[3] * CV_DEPTH) >> 10);
  return (v > 1023) ? (int16_t)1023 : v;
}

// The index is exponential, so adding to the rise and subtracting the same from the fall is a
// symmetric ratio in time: POT1 stays the geometric mean of the two, and only the shape changes.
static void applySkew() {
  int16_t d = (int16_t)gSkew - 512;
  if (d > -SKEW_DEAD && d < SKEW_DEAD) { skewIdx = 0; return; }
  skewIdx = (int16_t)(((int32_t)d * SKEW_SPAN) / 512);
}

// POT2 stays the master amount of uncertainty; this splits it between the times and the pitch.
// Centre gives both of them all of it, which is what the module did before the shift layer.
static void applyBalance() {
  uint16_t b = gBal;
  if (b > (512 - BAL_DEAD) && b < (512 + BAL_DEAD)) b = 512;
  timeGain = (b <= 512) ? 1023 : (uint16_t)((1023 - b) << 1);
  cvGain   = (b >= 512) ? 1023 : (uint16_t)(b << 1);
}

// Uniform in the table index, which is log uniform in time: as likely to land half as long as
// twice as long.  POT2 fully anticlockwise returns exactly 0, so the metronome is exact.
static int16_t randOffset() {
  uint16_t spread = (uint16_t)(((uint32_t)potVal[1] * RAND_SPREAD) >> 10);
  spread = (uint16_t)(((uint32_t)spread * timeGain) >> 10);
  if (!spread) return 0;
  return (int16_t)((int32_t)(((uint32_t)rnd16() * spread) >> 16) - (int32_t)(spread >> 1));
}

// The same spread, widened to the full output range, around mid scale
static uint16_t rollCv() {
  uint16_t s = (uint16_t)(((uint32_t)potVal[1] << 6) | (potVal[1] >> 4));
  s = (uint16_t)(((uint32_t)s * cvGain) >> 10);
  if (!s) return 32768;
  return (uint16_t)((int32_t)32768 + (int32_t)(((uint32_t)rnd16() * s) >> 16) - (int32_t)(s >> 1));
}

// 0..255 -> the nearest code in the current scale.  n = code * 60 / 255 rounded, and 241/1024 is
// 0.23535 against a true 0.23529, under a twentieth of a semitone over the whole five octaves.
static uint8_t quantCode(uint8_t code) {
  if (gScale == SC_OFF) return code;
  uint16_t mask = pgm_read_word(&SCALE_MASK[gScale]);
  int16_t  n = (int16_t)(((uint16_t)code * 241 + 512) >> 10);
  for (int8_t d = 0; d <= 6; d++) {
    int16_t lo = n - d;
    int16_t hi = n + d;
    bool okLo = (lo >= 0)  && (mask & (1 << (lo % 12)));
    bool okHi = (hi <= 60) && (mask & (1 << (hi % 12)));
    if (!okLo && !okHi) continue;
    // Anything nearer than d semitones was rejected on an earlier pass, so the answer is one of
    // these two.  Rounding n to a whole semitone threw away the fraction that would have said
    // which, so when both are allowed the decision is made on the codes themselves.
    uint8_t cLo = okLo ? pgm_read_byte(&NOTE_CODE[lo]) : 0;
    uint8_t cHi = okHi ? pgm_read_byte(&NOTE_CODE[hi]) : 0;
    if (!okHi) return cLo;
    if (!okLo) return cHi;
    return ((uint8_t)(code - cLo) <= (uint8_t)(cHi - code)) ? cLo : cHi;
  }
  return code;   // unreachable: every mask has the root, so a match is always within six
}

// Pot travel splits into NUM_SCALES zones, with hysteresis so a pot resting on a boundary does
// not dither between two scales.
static uint8_t scaleZone(uint16_t raw) {
  uint8_t cur = gScale;
  uint8_t z = (uint8_t)(((uint32_t)raw * NUM_SCALES) >> 10);
  if (z >= NUM_SCALES) z = NUM_SCALES - 1;
  if (z == cur) return cur;
  uint16_t bound = (uint16_t)((((uint32_t)(z > cur ? z : cur)) << 10) / NUM_SCALES);
  if (z > cur) return (raw >= bound + ZONE_HYST) ? z : cur;
  return (raw + ZONE_HYST <= bound) ? z : cur;
}

// ---------------------------------------------------------------- pots and the shift layer
static void freezePots() {
  for (uint8_t ch = 0; ch < 3; ch++) {
    potEntry[ch] = adcVal[ch];
    potSeen[ch]  = adcVal[ch];
    armed[ch]  = false;
    frozen[ch] = true;          // potVal stops tracking and holds its current value
  }
  lastTouched = -1;
}

// Hand the pots back.  One that was moved stays frozen at its old value until it is turned back
// through where it started; one that was left alone goes live again straight away.
static void releasePots() {
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (!frozen[ch]) continue;
    if (armed[ch]) entrySign[ch] = (adcVal[ch] >= potEntry[ch]) ? 1 : -1;
    else           frozen[ch] = false;
  }
}

static void servicePots() {
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (!frozen[ch]) potVal[ch] = adcVal[ch];
  }
}

// A frozen pot goes live again once it reaches, or passes back through, its entry position.
static void servicePickup() {
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (!frozen[ch]) continue;
    int16_t d = (int16_t)adcVal[ch] - (int16_t)potEntry[ch];
    if (d <= PICKUP_WINDOW && d >= -PICKUP_WINDOW) frozen[ch] = false;
    else if ((entrySign[ch] > 0) != (d > 0))       frozen[ch] = false;
  }
}

// Runs only while the button is down.  A pot does nothing until it has travelled ARM_DELTA, so a
// hold and release cannot stamp a resting pot position onto a stored value.
static void serviceShift() {
  for (uint8_t ch = 0; ch < 3; ch++) {
    int16_t d = (int16_t)adcVal[ch] - (int16_t)potEntry[ch];
    if (!armed[ch]) {
      if (d < ARM_DELTA && d > -ARM_DELTA) continue;
      armed[ch]   = true;
      shiftUsed   = true;       // cancels the curve tap and the re-roll hold
      lastTouched = (int8_t)ch;
    }

    int16_t moved = (int16_t)adcVal[ch] - (int16_t)potSeen[ch];
    if (moved > MOVE_DELTA || moved < -MOVE_DELTA) {
      potSeen[ch] = adcVal[ch];
      lastTouched = (int8_t)ch;
    }

    if (ch == 0) {
      if (gSkew != adcVal[0]) { gSkew = adcVal[0]; applySkew();    markDirty(); }
    } else if (ch == 1) {
      if (gBal  != adcVal[1]) { gBal  = adcVal[1]; applyBalance(); markDirty(); }
    } else {
      uint8_t z = scaleZone(adcVal[2]);
      if (z != gScale) {
        gScale = z;
        markDirty();
        startPattern((uint8_t)(z + 1), 40, 60, 120);   // dim = the shift layer, as in clepz
      }
    }
  }
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
    EEPROM.update(EE_ADDR_SKEW,  128);      // 128 << 2 is 512: symmetric
    EEPROM.update(EE_ADDR_BAL,   128);      // and centred: both get all of POT2
    EEPROM.update(EE_ADDR_SCALE, SC_OFF);
    EEPROM.update(EE_ADDR_VER,   EE_VERSION);
  }
  uint8_t c = EEPROM.read(EE_ADDR_CURVE);
  gCurve = (c < NUM_CURVES) ? c : (uint8_t)C_EXP;
  gSkew  = (uint16_t)EEPROM.read(EE_ADDR_SKEW) << 2;
  gBal   = (uint16_t)EEPROM.read(EE_ADDR_BAL)  << 2;
  uint8_t s = EEPROM.read(EE_ADDR_SCALE);
  gScale = (s < NUM_SCALES) ? s : (uint8_t)SC_OFF;
  applySkew();
  applyBalance();
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
  potVal[0] = adcVal[0];
  potVal[1] = adcVal[1];
  potVal[2] = adcVal[2];

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
      shiftUsed  = false;
      freezePots();
    } else {
      releasePots();
      // A release that did not already fire, and did not spend the hold on a pot, is the curve tap
      if (!vlongFired && !shiftUsed) {
        gCurve = (uint8_t)((gCurve + 1) % NUM_CURVES);
        markDirty();
        startPattern((uint8_t)(gCurve + 1), 255, 60, 120);
      }
    }
  }

  if (btnStable == LOW) {
    serviceShift();
    // A long hold re-rolls and restarts the cycle, so a set of numbers you do not like can be
    // thrown away without waiting out a ten second rest.  Turning a pot cancels it.
    if (!vlongFired && !shiftUsed && (now - btnDownMs) >= PRESS_VLONG_MS) {
      vlongFired = true;
      rollCycle();
      startPattern(1, 255, 250, 100);
    }
  }
}

// ---------------------------------------------------------------- LED
static void serviceLED() {
  uint32_t now = millis();

  if (patLeft) {                                   // curve / scale confirmation
    if ((int32_t)(now - patNextMs) >= 0) {
      if (patOn) { patOn = false; patNextMs = now + patOffMs; patLeft--; }
      else       { patOn = true;  patNextMs = now + patOnMs; }
    }
    ledSet(patLeft ? (patOn ? patBright : 0) : 0);
    return;
  }

  // In the shift layer the LED leaves the envelope and shows the value being set.  Until a pot has
  // moved it sits at a steady dim level, which is how you know the layer is open.  The scale is
  // discrete, so its flashes carry it instead.
  if (btnStable == LOW) {
    if (lastTouched == 0)      ledSet((uint8_t)(gSkew >> 2));
    else if (lastTouched == 1) ledSet((uint8_t)(gBal >> 2));
    else                       ledSet(EDIT_IDLE_LED);
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
  uint8_t code = quantCode((uint8_t)(rollCv() >> 8));
  cycleCv = (uint16_t)code << 8;
  OCR1B   = code;                          // F3, held for the whole cycle
  pulseTicks = PULSE_TICKS;                // F2 goes high
  phase = 0;
  stage = S_RISE;
}

static inline uint32_t currentInc() {
  if (stage == S_REST) {
    int16_t r = (int16_t)(((uint32_t)potVal[2] * REST_SPAN) >> 10);
    return incFromIndex((int16_t)(baseIndex() + REST_FAST - r + restOff));
  }
  // The skew shortens one slope by as much as it lengthens the other, in the index domain
  return incFromIndex((int16_t)(baseIndex()
         + ((stage == S_RISE) ? (int16_t)(riseOff + skewIdx) : (int16_t)(fallOff - skewIdx))));
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
      if (potVal[2] < REST_OFF) rollCycle();
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
  servicePots();
  serviceButton();
  if (btnStable != LOW) servicePickup();    // entrySign is only meaningful once released
  serviceLED();

  if (eeDirty && (millis() - eeDirtyMs) > EE_SAVE_DELAY_MS) {
    eeDirty = false;
    EEPROM.update(EE_ADDR_CURVE, gCurve);
    EEPROM.update(EE_ADDR_SKEW,  (uint8_t)(gSkew >> 2));
    EEPROM.update(EE_ADDR_BAL,   (uint8_t)(gBal >> 2));
    EEPROM.update(EE_ADDR_SCALE, gScale);
  }

  uint32_t now = micros();
  if ((uint32_t)(now - lastTickUs) < TICK_US) return;
  lastTickUs += TICK_US;
  if ((uint32_t)(now - lastTickUs) > TICK_US * 4) lastTickUs = now;
  outputTick();
}

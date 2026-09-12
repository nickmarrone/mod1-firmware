/*
  SMOOTH RANDOM  --  3 channel smooth random voltage generator for the HAGIWO MOD1
  Released under CC0.  MOD1 hardware by HAGIWO (https://note.com/solder_state/n/nc05d8e8fd311)

  Three smooth random CVs, one per output jack, each with its own algorithm.  Nothing ever
  steps or jumps: every algorithm is continuous, so the outputs are safe on filter cutoffs,
  VCA levels, wavefolders and anything else that would click on a hard transition.

  --Pin assign---
  POT1  A0   channel 1 rate   (hold BUTTON: channel 1 algorithm.  LORENZ: master speed)
  POT2  A1   channel 2 rate   (hold BUTTON: channel 2 algorithm.  LORENZ: rho)
  POT3  A2   channel 3 rate   (hold BUTTON: channel 3 algorithm.  LORENZ: beta)
  F1    A3   global rate CV in, 0..5V, adds up to about +4 octaves to all channels
  F2    D9   channel 1 out, 0..5V
  F3    D10  channel 2 out, 0..5V
  F4    D11  channel 3 out, 0..5V
  BUTTON D4  hold to edit algorithms; hold longer with no pot moved to toggle LORENZ
  LED   D3   channel 1 output, or the algorithm indicator while editing
  EEPROM     one algorithm per channel plus the LORENZ flag

  Algorithms, in pot order from fully CCW to fully CW:

    0 DRIFT    1 WANDER    2 TURBULENCE    3 HOLD

  Holding the button freezes all three rates and hands the pots to the algorithms.  A pot does
  nothing until it has moved 24 counts, so holding the button alone changes nothing.  The LED
  shows the algorithm of the channel whose pot you last moved; until you move one it blinks dim
  to show you are in edit mode.  On release, a channel whose pot you moved keeps its old rate
  until that pot is turned back through where it started.

  LORENZ is not one of the four.  It is a single chaotic system whose x, y and z drive all three
  outputs at once, so it takes the whole module and all three pots.  Hold the button past 1.5 s
  without touching a pot to toggle it; the LED blinks twice going in and once coming out.  The
  per channel algorithms are remembered while it runs, and the rates the pots had before it are
  handed back with the same pickup rule an edit uses.

  Generator runs at 1 kHz, 16 bit internally, 8 bit out through the 62.5 kHz PWM and the
  1uF/1k reconstruction filter already on the MOD1 board.
*/

#include <EEPROM.h>
#include <avr/pgmspace.h>

// ---------------------------------------------------------------- pins
#define PIN_LED     3
#define PIN_BUTTON  4
#define PIN_OUT1    9    // OC1A
#define PIN_OUT2    10   // OC1B
#define PIN_OUT3    11   // OC2A

// ---------------------------------------------------------------- config
#define TICK_US        1000UL  // 1 kHz generator tick.  The MOD1's output filter sits
                               // at ~159 Hz, so 1 kHz is still six times the corner.
#define NUM_ALGOS      4
#define DEBOUNCE_MS    30UL
#define HOLD_MS        300UL   // button held this long enters algorithm edit
#define LORENZ_HOLD_MS 1500UL  // ... and this long, with no pot moved, toggles LORENZ
#define EE_SAVE_DELAY_MS 2000UL

#define ARM_DELTA      24      // pot travel needed before it takes an algorithm
#define PICKUP_WINDOW  12      // how close counts as catching the old position
#define ZONE_HYST      12      // travel past a zone boundary before the zone changes
#define MOVE_DELTA     3       // pot travel that counts as "this is the pot I am holding"

#define EDIT_IDLE_LED  24      // dim level, for the edit idle blink and for DRIFT
#define BLINK_MS       120UL   // one on or off phase of the LORENZ confirmation blink

// EEPROM map.  Byte 1 stays the power up seed counter it has always been.
#define EE_ADDR_ALGO1  0
#define EE_ADDR_SEED   1
#define EE_ADDR_ALGO2  2
#define EE_ADDR_ALGO3  3
#define EE_ADDR_LORENZ 4
#define EE_ADDR_VER    5
#define EE_VERSION     2

// Rate CV depth: full 5V adds this many table indices (1024 indices span ~11 octaves,
// so 373 indices is very close to +4 octaves).
#define CV_DEPTH       373L

enum { T_DRIFT = 0, T_WANDER, T_TURBULENCE, T_HOLD };

// ---------------------------------------------------------------- tables
// Phase increment per 1 ms tick, exponential 0.01 Hz .. 20 Hz (33 entries, interpolated)
static const uint32_t INC_TAB[33] PROGMEM = {
  42950UL, 54465UL, 69068UL, 87585UL,
  111068UL, 140846UL, 178609UL, 226496UL,
  287222UL, 364229UL, 461883UL, 585718UL,
  742756UL, 941897UL, 1194429UL, 1514669UL,
  1920768UL, 2435746UL, 3088796UL, 3916936UL,
  4967108UL, 6298843UL, 7987631UL, 10129201UL,
  12844948UL, 16288817UL, 20656024UL, 26194126UL,
  33217054UL, 42122903UL, 53416507UL, 67738047UL,
  85899346UL
};

// smoothstep(t) = t*t*(3-2t) scaled to 0..65535, 257 entries so we can interpolate
static const uint16_t SMOOTH_TAB[257] PROGMEM = {
  0, 3, 12, 27, 47, 74, 106, 144,
  188, 237, 292, 353, 418, 490, 567, 649,
  736, 829, 926, 1029, 1137, 1251, 1369, 1492,
  1620, 1753, 1891, 2033, 2180, 2332, 2489, 2650,
  2816, 2986, 3161, 3340, 3523, 3711, 3903, 4100,
  4300, 4504, 4713, 4926, 5142, 5363, 5587, 5816,
  6048, 6284, 6523, 6767, 7013, 7264, 7518, 7775,
  8036, 8300, 8568, 8838, 9112, 9390, 9670, 9953,
  10240, 10529, 10822, 11117, 11415, 11716, 12020, 12327,
  12636, 12948, 13262, 13579, 13898, 14220, 14544, 14871,
  15200, 15531, 15864, 16200, 16537, 16877, 17219, 17562,
  17908, 18255, 18604, 18955, 19308, 19663, 20019, 20376,
  20736, 21096, 21459, 21822, 22187, 22553, 22921, 23290,
  23660, 24031, 24403, 24776, 25150, 25525, 25901, 26278,
  26656, 27034, 27413, 27793, 28173, 28554, 28935, 29317,
  29700, 30082, 30465, 30849, 31232, 31616, 32000, 32384,
  32768, 33151, 33535, 33919, 34303, 34686, 35070, 35453,
  35835, 36218, 36600, 36981, 37362, 37742, 38122, 38501,
  38879, 39257, 39634, 40010, 40385, 40759, 41132, 41504,
  41875, 42245, 42614, 42982, 43348, 43713, 44076, 44439,
  44799, 45159, 45516, 45872, 46227, 46580, 46931, 47280,
  47627, 47973, 48316, 48658, 48998, 49335, 49671, 50004,
  50335, 50664, 50991, 51315, 51637, 51956, 52273, 52587,
  52899, 53208, 53515, 53819, 54120, 54418, 54713, 55006,
  55295, 55582, 55865, 56145, 56423, 56697, 56967, 57235,
  57499, 57760, 58017, 58271, 58522, 58768, 59012, 59251,
  59487, 59719, 59948, 60172, 60393, 60609, 60822, 61031,
  61235, 61435, 61632, 61824, 62012, 62195, 62374, 62549,
  62719, 62885, 63046, 63203, 63355, 63502, 63644, 63782,
  63915, 64043, 64166, 64284, 64398, 64506, 64609, 64706,
  64799, 64886, 64968, 65045, 65117, 65182, 65243, 65298,
  65347, 65391, 65429, 65461, 65488, 65508, 65523, 65532,
  65535
};
// ---------------------------------------------------------------- state
static uint8_t  gAlgo[3] = { T_DRIFT, T_DRIFT, T_DRIFT };
static bool     gLorenz  = false;

// non blocking ADC round robin over A0..A3
static uint16_t adcVal[4] = { 0, 0, 0, 0 };
static uint8_t  adcCh = 0;

// potVal is what the rate engine sees.  It tracks the pots except while a channel is frozen,
// which is how an edit and a LORENZ excursion both leave the old rates intact.  LORENZ itself
// reads adcVal live, because there the pots really are speed, rho and beta.
static uint16_t potVal[3];
static bool     frozen[3] = { false, false, false };
static uint16_t potEntry[3];
static uint16_t potSeen[3];
static int8_t   entrySign[3];
static bool     armed[3];

// DRIFT / HOLD / TURBULENCE share this segment generator.
// TURBULENCE uses three octaves per channel, DRIFT and HOLD use octave 0 only.
static uint32_t segPhase[3][3];
static uint16_t segFrom[3][3];
static uint16_t segTo[3][3];

// WANDER
static int32_t  wPos[3];
static int32_t  wVel[3];
#define WPOS_MAX 16777215L      // 16 bit output range shifted left by 8

// LORENZ
static float lx = 0.9f, ly = 0.0f, lz = 12.0f;
static float lRho = 28.0f, lBeta = 2.6667f;
static float lkX = 0.0f, lkY = 0.0f, lkZ = 0.0f;   // normalisation gains
static float lzOff = 0.0f;
static uint16_t lorPrev[3], lorCur[3];
static uint8_t  lorSub = 0;
static uint8_t  lorParamDiv = 0;

// button and edit mode
static uint8_t  btnLastRead = HIGH, btnStable = HIGH;
static uint32_t btnChangedMs = 0, btnDownMs = 0;
static bool     editing = false;
static bool     longFired = false;
static int8_t   lastTouched = -1;

// LED: channel 1's last output, and the LORENZ confirmation blink
static uint8_t  ledLevel = 0;
static uint8_t  blinkPhases = 0;
static uint32_t blinkMs = 0;

// misc
static uint32_t rng = 0x2545F491UL;
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

// smoothstep(x) with x as a full scale 16 bit phase, interpolated between table entries
static inline uint16_t smoothstep16(uint16_t x) {
  uint8_t  i = (uint8_t)(x >> 8);
  uint8_t  f = (uint8_t)(x & 0xFF);
  uint16_t a = pgm_read_word(&SMOOTH_TAB[i]);
  uint16_t b = pgm_read_word(&SMOOTH_TAB[i + 1]);
  return a + (uint16_t)(((uint32_t)(b - a) * f) >> 8);
}

// table index 0..1023 -> 32 bit phase increment per tick
static uint32_t incFromIndex(int16_t idx) {
  if (idx < 0) idx = 0;
  if (idx > 1023) idx = 1023;
  uint8_t  seg = (uint8_t)(idx >> 5);
  uint8_t  f   = (uint8_t)(idx & 31);
  uint32_t a = pgm_read_dword(&INC_TAB[seg]);
  uint32_t b = pgm_read_dword(&INC_TAB[seg + 1]);
  return a + (uint32_t)(((b - a) * (uint32_t)f) >> 5);
}

// pot value + global rate CV, clamped to the table range
static inline int16_t rateIndexFrom(uint16_t pot) {
  return (int16_t)min(1023L, (long)pot + (((long)adcVal[3] * CV_DEPTH) >> 10));
}
static inline int16_t rateIndex(uint8_t ch) { return rateIndexFrom(potVal[ch]); }

// advance one noise segment; returns the eased value between the two endpoints.
// easeFull = true  -> glide across the whole segment (DRIFT / TURBULENCE)
// easeFull = false -> glide across the first quarter then hold (HOLD)
static inline uint16_t segStep(uint8_t ch, uint8_t oct, uint32_t inc, bool easeFull) {
  uint32_t prev = segPhase[ch][oct];
  uint32_t now  = prev + inc;
  segPhase[ch][oct] = now;
  if (now < prev) {                       // wrapped: arrive, pick the next destination
    segFrom[ch][oct] = segTo[ch][oct];
    segTo[ch][oct]   = rnd16();
  }
  uint16_t ph = (uint16_t)(now >> 16);
  if (!easeFull) ph = (ph < 16384) ? (uint16_t)(ph << 2) : 65535;
  uint16_t s = smoothstep16(ph);
  uint16_t a = segFrom[ch][oct];
  uint16_t b = segTo[ch][oct];
  // unsigned all the way: (65535 * 65535) still fits a uint32, a signed version would not
  if (b >= a) return a + (uint16_t)(((uint32_t)(b - a) * s) >> 16);
  return a - (uint16_t)(((uint32_t)(a - b) * s) >> 16);
}

static inline void writeOut(uint8_t ch, uint16_t v) {
  uint8_t duty = (uint8_t)(v >> 8);
  if (ch == 0)      { OCR1A = duty; ledLevel = duty; }
  else if (ch == 1) OCR1B = duty;
  else              OCR2A = duty;
}

static void markDirty() { eeDirty = true; eeDirtyMs = millis(); }

// ---------------------------------------------------------------- setup
static void updateLorenzParams();

static void configurePWM() {
  // Timer1: 8 bit fast PWM, no prescaler -> 62.5 kHz on D9 (OC1A) and D10 (OC1B)
  TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM12) | (1 << CS10);
  // Timer2: fast PWM, no prescaler -> 62.5 kHz on D11 (OC2A) and D3 (OC2B, the LED)
  TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1) | (1 << COM2B1);
  TCCR2B = (1 << CS20);
}

static void seedRng() {
  uint32_t s = 0;
  for (uint8_t i = 0; i < 16; i++) {          // A6/A7 are unconnected on the MOD1: noise
    s = (s << 1) ^ (uint32_t)(analogRead((i & 1) ? A7 : A6) & 1) ^ (s >> 31);
  }
  uint8_t stored = EEPROM.read(EE_ADDR_SEED);
  EEPROM.write(EE_ADDR_SEED, (uint8_t)(stored + 1));
  s ^= ((uint32_t)stored << 24) ^ ((uint32_t)stored << 7) ^ micros();
  if (s == 0) s = 0x2545F491UL;
  rng = s;
}

// The old firmware kept one global type at byte 0 and nothing else, so there is no way to tell
// its byte 0 from a per channel one.  A version stamp settles it: an unrecognised layout is
// rewritten to defaults once, and everything after that is read back as written.
static void loadSettings() {
  if (EEPROM.read(EE_ADDR_VER) != EE_VERSION) {
    EEPROM.update(EE_ADDR_ALGO1,  T_DRIFT);
    EEPROM.update(EE_ADDR_ALGO2,  T_DRIFT);
    EEPROM.update(EE_ADDR_ALGO3,  T_DRIFT);
    EEPROM.update(EE_ADDR_LORENZ, 0);
    EEPROM.update(EE_ADDR_VER,    EE_VERSION);
  }
  gAlgo[0] = EEPROM.read(EE_ADDR_ALGO1);
  gAlgo[1] = EEPROM.read(EE_ADDR_ALGO2);
  gAlgo[2] = EEPROM.read(EE_ADDR_ALGO3);
  for (uint8_t ch = 0; ch < 3; ch++) if (gAlgo[ch] >= NUM_ALGOS) gAlgo[ch] = T_DRIFT;
  gLorenz = EEPROM.read(EE_ADDR_LORENZ) ? true : false;
}

static void initChannels() {
  for (uint8_t c = 0; c < 3; c++) {
    for (uint8_t o = 0; o < 3; o++) {
      segPhase[c][o] = 0;
      segFrom[c][o]  = rnd16();
      segTo[c][o]    = rnd16();
    }
    wPos[c] = (int32_t)rnd16() << 8;
    wVel[c] = 0;
    lorPrev[c] = 32768;
    lorCur[c]  = 32768;
    writeOut(c, segFrom[c][0]);      // start at a real value, not a 0 V blip
  }
}

void setup() {
  pinMode(PIN_OUT1, OUTPUT);
  pinMode(PIN_OUT2, OUTPUT);
  pinMode(PIN_OUT3, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  seedRng();                        // uses analogRead(), so do it before we take over the ADC
  configurePWM();
  loadSettings();
  initChannels();

  // Prime the filtered ADC values before taking the converter over, so the first tick already
  // has real pot positions instead of walking up from zero.
  adcVal[0] = analogRead(A0);
  adcVal[1] = analogRead(A1);
  adcVal[2] = analogRead(A2);
  adcVal[3] = analogRead(A3);
  for (uint8_t ch = 0; ch < 3; ch++) {
    potVal[ch]   = adcVal[ch];
    potEntry[ch] = adcVal[ch];
    potSeen[ch]  = adcVal[ch];
    armed[ch]    = false;
    // Booting straight into LORENZ: treat these positions as the rates to hand back on the way
    // out, otherwise the first exit would jump to wherever rho and beta left the pots.
    frozen[ch]   = gLorenz;
  }

  updateLorenzParams();            // so LORENZ has sane gains before its first refresh

  DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D);
  adcCh = 0;
  ADMUX  = (1 << REFS0) | adcCh;    // AVcc reference, start on A0
  ADCSRA |= (1 << ADSC);
  lastTickUs = micros();
}

// ---------------------------------------------------------------- ADC
static inline void serviceADC() {
  if (ADCSRA & (1 << ADSC)) return;         // still converting
  uint16_t v = ADC;
  adcVal[adcCh] = (uint16_t)((adcVal[adcCh] * 3UL + v) >> 2);   // de-jitter, so a pot resting
  adcCh = (uint8_t)((adcCh + 1) & 3);                           // on a zone boundary stays put
  ADMUX = (1 << REFS0) | adcCh;             // A0..A3 are mux channels 0..3
  ADCSRA |= (1 << ADSC);
}

// ---------------------------------------------------------------- pots and edit mode
static void freezePots() {
  for (uint8_t ch = 0; ch < 3; ch++) {
    potEntry[ch] = adcVal[ch];
    potSeen[ch]  = adcVal[ch];
    armed[ch] = false;
    frozen[ch] = true;          // potVal stops tracking and holds its current value
  }
}

// Hand the pots back.  One that was moved stays frozen at its old rate until it is turned back
// through where it started; one that was left alone goes live again straight away.
static void releasePots() {
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (!frozen[ch]) continue;
    int16_t d = (int16_t)adcVal[ch] - (int16_t)potEntry[ch];
    if (d > PICKUP_WINDOW || d < -PICKUP_WINDOW) entrySign[ch] = (d > 0) ? 1 : -1;
    else frozen[ch] = false;
  }
}

static void servicePots() {
  if (gLorenz) return;          // the pots are speed, rho and beta; the rates hold where they were
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

// Pot travel splits into NUM_ALGOS zones, with hysteresis so a pot resting on a boundary does
// not dither between two algorithms.
static uint8_t zoneFor(uint8_t ch, uint16_t raw) {
  uint8_t cur = gAlgo[ch];
  uint8_t z = (uint8_t)(((uint32_t)raw * NUM_ALGOS) >> 10);
  if (z == cur) return cur;
  uint16_t bound = (uint16_t)((((uint32_t)(z > cur ? z : cur)) << 10) / NUM_ALGOS);
  if (z > cur) return (raw >= bound + ZONE_HYST) ? z : cur;
  return (raw + ZONE_HYST <= bound) ? z : cur;
}

static void beginEdit() {
  editing = true;
  lastTouched = -1;
  if (gLorenz) {
    // Nothing to select, and the entry positions belong to the rates LORENZ took over from,
    // so leave them alone.  The long hold is the only thing this edit can do.
    armed[0] = armed[1] = armed[2] = false;
  } else {
    freezePots();
  }
}

static void endEdit() {
  editing = false;
  if (!gLorenz) releasePots();
}

static void serviceEdit() {
  for (uint8_t ch = 0; ch < 3; ch++) {
    int16_t d = (int16_t)adcVal[ch] - (int16_t)potEntry[ch];
    if (!armed[ch]) {
      if (d < ARM_DELTA && d > -ARM_DELTA) continue;
      armed[ch] = true;
      lastTouched = (int8_t)ch;
    }

    int16_t moved = (int16_t)adcVal[ch] - (int16_t)potSeen[ch];
    if (moved > MOVE_DELTA || moved < -MOVE_DELTA) {
      potSeen[ch] = adcVal[ch];
      lastTouched = (int8_t)ch;
    }

    uint8_t z = zoneFor(ch, adcVal[ch]);
    if (z != gAlgo[ch]) {
      gAlgo[ch] = z;
      markDirty();
    }
  }
}

static inline bool anyArmed() { return armed[0] || armed[1] || armed[2]; }

// ---------------------------------------------------------------- LED
static inline void ledSet(uint8_t v) {
  if (v == 0) {
    TCCR2A &= (uint8_t)~(1 << COM2B1);    // detach OC2B, otherwise a 1/256 sliver leaks through
    PORTD  &= (uint8_t)~(1 << PD3);
  } else {
    TCCR2A |= (1 << COM2B1);
    OCR2B = v;
  }
}

static void ledBlink(uint8_t times) {
  blinkPhases = (uint8_t)(times * 2);     // on, off, on, off ...
  blinkMs = millis();
}

// Triangle fade, one full up/down sweep every (1 << periodShift) ms.  The ramp is squared
// on the way out: the eye is roughly square law, so a linear duty ramp would shoot up and
// then sit near full for most of the sweep instead of reading as a steady fade.
static inline uint8_t triangleBrightness(uint16_t ms, uint8_t periodShift) {
  uint16_t half = (uint16_t)(1u << (periodShift - 1));
  uint16_t t    = (uint16_t)(ms & ((1u << periodShift) - 1));
  uint16_t up   = (t < half) ? t : (uint16_t)(2 * half - 1 - t);     // 0 .. half-1 and back
  uint8_t  lin  = (uint8_t)(((uint32_t)up << 8) >> (periodShift - 1));
  return (uint8_t)(((uint16_t)lin * lin) >> 8);
}

static inline void serviceLED() {
  uint32_t nowMs = millis();

  if (blinkPhases) {                                  // LORENZ confirmation overrides everything
    if ((uint32_t)(nowMs - blinkMs) >= BLINK_MS) { blinkMs += BLINK_MS; blinkPhases--; }
    if (blinkPhases) { ledSet((blinkPhases & 1) ? 0 : 255); return; }
  }

  if (!editing) { ledSet(ledLevel); return; }         // channel 1's output, LORENZ x included

  uint16_t ms = (uint16_t)nowMs;
  if (lastTouched < 0) {                              // in edit mode, no pot moved yet
    ledSet((ms & 256) ? EDIT_IDLE_LED : 0);           // ~2 Hz dim blink
    return;
  }
  switch (gAlgo[lastTouched]) {                       // nothing here is ever fully dark, so a
    case T_DRIFT:      ledSet(EDIT_IDLE_LED); break;  // dim LED never reads as a dead one
    case T_WANDER:     ledSet(triangleBrightness(ms, 10)); break;    // ~1 Hz
    case T_TURBULENCE: ledSet(triangleBrightness(ms, 8)); break;     // ~4 Hz
    default:           ledSet(255); break;                           // T_HOLD, steady on
  }
}

// ---------------------------------------------------------------- button
static void serviceButton() {
  uint32_t now = millis();
  uint8_t  r = digitalRead(PIN_BUTTON);

  if (r != btnLastRead) { btnLastRead = r; btnChangedMs = now; }
  else if (r != btnStable && (now - btnChangedMs) >= DEBOUNCE_MS) {
    btnStable = r;
    if (r == LOW) { btnDownMs = now; longFired = false; }   // a short tap does nothing
    else if (editing) endEdit();
  }

  if (btnStable != LOW) return;

  // longFired also marks the press spent, so the edit cannot restart after the gesture fires
  if (!editing && !longFired && (now - btnDownMs) >= HOLD_MS) beginEdit();

  // A long hold with every pot left alone is the whole module gesture.  In LORENZ no pot can
  // arm, because serviceEdit does not run there, so a stray nudge can never block the way out.
  if (editing && !longFired && (now - btnDownMs) >= LORENZ_HOLD_MS && !anyArmed()) {
    longFired = true;
    if (gLorenz) {
      // Leaving.  End the edit here and now: potEntry still points at where the pots were before
      // LORENZ, and letting serviceEdit loose on that stale reference would read the rho and beta
      // positions as algorithm choices.
      gLorenz = false;
      releasePots();
      editing = false;
    } else {
      gLorenz = true;          // entering; beginEdit already froze the pots, and endEdit will
    }                          // leave them that way, holding the rates for the trip back
    ledBlink(gLorenz ? 2 : 1);
    markDirty();
  }
}

// ---------------------------------------------------------------- generators
static inline void tickSegment(uint8_t ch, bool easeFull) {
  writeOut(ch, segStep(ch, 0, incFromIndex(rateIndex(ch)), easeFull));
}

static void tickWander(uint8_t ch) {
  uint32_t inc = incFromIndex(rateIndex(ch));
  int32_t  amp = (int32_t)(inc >> 9);            // kick size tracks the rate
  int8_t   r   = (int8_t)(rnd16() >> 8);
  wVel[ch] += (r * amp) >> 7;                    // random impulse, +/- amp
  wVel[ch] -= (wVel[ch] >> 5);                   // inertia / damping
  wPos[ch] += wVel[ch];
  if (wPos[ch] < 0) {                            // reflect off the rails
    wPos[ch] = -wPos[ch];
    wVel[ch] = -wVel[ch];
  } else if (wPos[ch] > WPOS_MAX) {
    wPos[ch] = 2L * WPOS_MAX - wPos[ch];
    wVel[ch] = -wVel[ch];
  }
  if (wPos[ch] < 0) wPos[ch] = 0;
  else if (wPos[ch] > WPOS_MAX) wPos[ch] = WPOS_MAX;
  writeOut(ch, (uint16_t)(wPos[ch] >> 8));
}

static void tickTurbulence(uint8_t ch) {
  uint32_t inc = incFromIndex(rateIndex(ch));
  uint32_t q   = inc >> 2;
  uint16_t v0 = segStep(ch, 0, inc, true);        // 1x
  uint16_t v1 = segStep(ch, 1, q * 11, true);     // 2.75x
  uint16_t v2 = segStep(ch, 2, q * 29, true);     // 7.25x
  int32_t acc = 4L * ((int32_t)v0 - 32768) + 2L * ((int32_t)v1 - 32768) + ((int32_t)v2 - 32768);
  int32_t val = 32768L + ((acc * 1609L) >> 13);   // x11/56: /7 to normalise, x11/8 for range
  if (val < 0) val = 0;
  else if (val > 65535L) val = 65535L;
  writeOut(ch, (uint16_t)val);
}

static void tickChannel(uint8_t ch) {
  switch (gAlgo[ch]) {
    case T_DRIFT:      tickSegment(ch, true);  break;
    case T_WANDER:     tickWander(ch);         break;
    case T_TURBULENCE: tickTurbulence(ch);     break;
    default:           tickSegment(ch, false); break;   // T_HOLD
  }
}

static void updateLorenzParams() {
  lRho  = 20.0f + (float)adcVal[1] * (40.0f / 1023.0f);    // 20 .. 60
  lBeta = 1.0f  + (float)adcVal[2] * (3.0f / 1023.0f);     // 1 .. 4
  float c = sqrt(lBeta * (lRho - 1.0f));                   // distance to the fixed points
  if (c < 0.5f) c = 0.5f;
  lkX = 1.0f / (5.0f * c);
  lkY = 1.0f / (7.0f * c);
  // z never visits the bottom of its own scale, so shift it up before normalising
  lzOff = 0.08f * (lRho - 1.0f);
  lkZ   = 1.0f / (1.70f * (lRho - 1.0f));
}

static inline uint16_t lorNorm(float v) {
  if (v <= 0.0f) return 0;
  if (v >= 1.0f) return 65535;
  return (uint16_t)(v * 65535.0f);
}

// LORENZ reads the pots live: here they are the attractor's own controls, not the rates that
// potVal is holding on to for when the module comes back out of this mode.
static void tickLorenz() {
  if (lorSub == 1 && ++lorParamDiv >= 16) {   // ~32 ms, on a tick with no integration to do
    lorParamDiv = 0;
    updateLorenzParams();
  }
  if (lorSub == 0) {
    // Lorenz time step from POT1 + rate CV.  The constant folds in a 0.15 factor that maps
    // the pot's 0.01..20 Hz onto 0.0015..3 Hz, which keeps the whole sweep inside the region
    // where forward Euler on this system stays stable (dt tops out at 0.0042 after the split).
    float dt = (float)incFromIndex(rateIndexFrom(adcVal[0])) * 1.956e-10f;
    uint8_t sub = 1;
    if (dt > 0.0021f) { sub = 2; dt *= 0.5f; }

    for (uint8_t i = 0; i < sub; i++) {
      float dx = 10.0f * (ly - lx);
      float dy = lx * (lRho - lz) - ly;
      float dz = lx * ly - lBeta * lz;
      lx += dx * dt;
      ly += dy * dt;
      lz += dz * dt;
    }
    if (!(lx > -1e6f && lx < 1e6f)) { lx = 0.9f; ly = 0.0f; lz = 12.0f; }   // NaN / blow up guard

    lorPrev[0] = lorCur[0]; lorPrev[1] = lorCur[1]; lorPrev[2] = lorCur[2];
    lorCur[0] = lorNorm(0.5f + lx * lkX);
    lorCur[1] = lorNorm(0.5f + ly * lkY);
    lorCur[2] = lorNorm((lz - lzOff) * lkZ);
  }

  for (uint8_t c = 0; c < 3; c++) {                 // interpolate across the 2 ticks
    int32_t p = lorPrev[c];
    int32_t d = (int32_t)lorCur[c] - p;
    writeOut(c, (uint16_t)(p + ((d * (int32_t)lorSub) >> 1)));
  }
  lorSub = (uint8_t)((lorSub + 1) & 1);
}

// ---------------------------------------------------------------- loop
void loop() {
  serviceADC();
  serviceButton();
  if (!gLorenz) {                 // in LORENZ the pots belong to the attractor, so there is
    if (editing) serviceEdit();   // nothing to arm and nothing to catch
    else         servicePickup();
  }
  servicePots();
  serviceLED();

  if (eeDirty && (millis() - eeDirtyMs) > EE_SAVE_DELAY_MS) {
    eeDirty = false;
    EEPROM.update(EE_ADDR_ALGO1,  gAlgo[0]);
    EEPROM.update(EE_ADDR_ALGO2,  gAlgo[1]);
    EEPROM.update(EE_ADDR_ALGO3,  gAlgo[2]);
    EEPROM.update(EE_ADDR_LORENZ, gLorenz ? 1 : 0);
  }

  uint32_t now = micros();
  if ((uint32_t)(now - lastTickUs) < TICK_US) return;
  lastTickUs += TICK_US;
  if ((uint32_t)(now - lastTickUs) > TICK_US * 4) lastTickUs = now;   // resync if we fell behind

  if (gLorenz) tickLorenz();
  else for (uint8_t c = 0; c < 3; c++) tickChannel(c);
}

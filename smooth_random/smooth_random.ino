/*
  SMOOTH RANDOM  --  3 channel smooth random voltage generator for the HAGIWO MOD1
  Released under CC0.  MOD1 hardware by HAGIWO (https://note.com/solder_state/n/nc05d8e8fd311)

  Three independent smooth random CVs, one per output jack.  Nothing ever steps or jumps:
  every algorithm is continuous, so the outputs are safe on filter cutoffs, VCA levels,
  wavefolders and anything else that would click on a hard transition.

  --Pin assign---
  POT1  A0   channel 1 rate   (LORENZ mode: master speed)
  POT2  A1   channel 2 rate   (LORENZ mode: rho  / chaos shape)
  POT3  A2   channel 3 rate   (LORENZ mode: beta / chaos shape)
  F1    A3   global rate CV in, 0..5V, adds up to about +4 octaves to all channels
  F2    D9   channel 1 out, 0..5V
  F3    D10  channel 2 out, 0..5V
  F4    D11  channel 3 out, 0..5V
  BUTTON D4  short press: next random type (saved to EEPROM)
  LED   D3   which type is selected

  Type                LED
  0 DRIFT             off
  1 WANDER            slow triangle fade (1 Hz)
  2 TURBULENCE        fast triangle fade (4 Hz)
  3 LORENZ            steady dim
  4 HOLD              steady on

  Generator runs at 2 kHz, 16 bit internally, 8 bit out through the 62.5 kHz PWM and the
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
#define NUM_TYPES      5
#define EE_ADDR_TYPE   0
#define EE_ADDR_SEED   1
#define DEBOUNCE_MS    40UL

// Rate CV depth: full 5V adds this many table indices (1024 indices span ~11 octaves,
// so 373 indices is very close to +4 octaves).
#define CV_DEPTH       373L

enum { T_DRIFT = 0, T_WANDER, T_TURBULENCE, T_LORENZ, T_HOLD };

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
static uint8_t  gType = T_DRIFT;

// non blocking ADC round robin over A0..A3
static uint16_t adcVal[4] = { 0, 0, 0, 0 };
static uint8_t  adcCh = 0;

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

// misc
static uint32_t rng = 0x2545F491UL;
static uint32_t lastTickUs = 0;
static uint32_t btnChangedMs = 0;
static uint8_t  btnLastRead = HIGH;
static uint8_t  btnStable = HIGH;

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

// pot + global rate CV, clamped to the table range
static inline int16_t rateIndex(uint8_t ch) {
  return (int16_t)min(1023L, (long)adcVal[ch] + (((long)adcVal[3] * CV_DEPTH) >> 10));
}

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
  if (ch == 0)      OCR1A = duty;
  else if (ch == 1) OCR1B = duty;
  else              OCR2A = duty;
}

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

  uint8_t t = EEPROM.read(EE_ADDR_TYPE);
  gType = (t < NUM_TYPES) ? t : (uint8_t)T_DRIFT;

  initChannels();
  updateLorenzParams();            // so LORENZ has sane gains before its first refresh

  ADMUX  = (1 << REFS0) | 0;        // AVcc reference, start on A0
  ADCSRA |= (1 << ADSC);
  lastTickUs = micros();
}

// ---------------------------------------------------------------- ADC
static inline void serviceADC() {
  if (ADCSRA & (1 << ADSC)) return;         // still converting
  adcVal[adcCh] = ADC;
  adcCh = (uint8_t)((adcCh + 1) & 3);
  ADMUX = (1 << REFS0) | adcCh;             // A0..A3 are mux channels 0..3
  ADCSRA |= (1 << ADSC);
}

// ---------------------------------------------------------------- button
static inline void serviceButton() {
  uint32_t now = millis();
  uint8_t  r = digitalRead(PIN_BUTTON);
  if (r != btnLastRead) {                 // reading moved: restart the settle timer
    btnLastRead = r;
    btnChangedMs = now;
    return;
  }
  if (r != btnStable && (now - btnChangedMs) >= DEBOUNCE_MS) {
    btnStable = r;
    if (r == LOW) {                       // confirmed press
      gType = (uint8_t)((gType + 1) % NUM_TYPES);
      EEPROM.update(EE_ADDR_TYPE, gType);
    }
  }
}

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
  uint16_t ms = (uint16_t)millis();
  switch (gType) {
    case T_DRIFT:      ledSet(0); break;
    case T_WANDER:     ledSet(triangleBrightness(ms, 10)); break;    // ~1 Hz
    case T_TURBULENCE: ledSet(triangleBrightness(ms, 8)); break;     // ~4 Hz
    case T_LORENZ:     ledSet(24); break;                            // steady dim
    default:           ledSet(255); break;                           // steady on
  }
}

// ---------------------------------------------------------------- generators
static void tickDriftOrHold(bool easeFull) {
  for (uint8_t c = 0; c < 3; c++) {
    writeOut(c, segStep(c, 0, incFromIndex(rateIndex(c)), easeFull));
  }
}

static void tickWander() {
  for (uint8_t c = 0; c < 3; c++) {
    uint32_t inc = incFromIndex(rateIndex(c));
    int32_t  amp = (int32_t)(inc >> 9);            // kick size tracks the rate
    int8_t   r   = (int8_t)(rnd16() >> 8);
    wVel[c] += (r * amp) >> 7;                     // random impulse, +/- amp
    wVel[c] -= (wVel[c] >> 5);                     // inertia / damping
    wPos[c] += wVel[c];
    if (wPos[c] < 0) {                             // reflect off the rails
      wPos[c] = -wPos[c];
      wVel[c] = -wVel[c];
    } else if (wPos[c] > WPOS_MAX) {
      wPos[c] = 2L * WPOS_MAX - wPos[c];
      wVel[c] = -wVel[c];
    }
    if (wPos[c] < 0) wPos[c] = 0;
    else if (wPos[c] > WPOS_MAX) wPos[c] = WPOS_MAX;
    writeOut(c, (uint16_t)(wPos[c] >> 8));
  }
}

static void tickTurbulence() {
  for (uint8_t c = 0; c < 3; c++) {
    uint32_t inc = incFromIndex(rateIndex(c));
    uint32_t q   = inc >> 2;
    uint16_t v0 = segStep(c, 0, inc, true);        // 1x
    uint16_t v1 = segStep(c, 1, q * 11, true);     // 2.75x
    uint16_t v2 = segStep(c, 2, q * 29, true);     // 7.25x
    int32_t acc = 4L * ((int32_t)v0 - 32768) + 2L * ((int32_t)v1 - 32768) + ((int32_t)v2 - 32768);
    int32_t val = 32768L + ((acc * 1609L) >> 13);  // x11/56: /7 to normalise, x11/8 for range
    if (val < 0) val = 0;
    else if (val > 65535L) val = 65535L;
    writeOut(c, (uint16_t)val);
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

static void tickLorenz() {
  if (lorSub == 1 && ++lorParamDiv >= 16) {   // ~32 ms, on a tick with no integration to do
    lorParamDiv = 0;
    updateLorenzParams();
  }
  if (lorSub == 0) {
    // Lorenz time step from POT1 + rate CV.  The constant folds in a 0.15 factor that maps
    // the pot's 0.01..20 Hz onto 0.0015..3 Hz, which keeps the whole sweep inside the region
    // where forward Euler on this system stays stable (dt tops out at 0.0042 after the split).
    float dt = (float)incFromIndex(rateIndex(0)) * 1.956e-10f;
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
  serviceLED();

  uint32_t now = micros();
  if ((uint32_t)(now - lastTickUs) < TICK_US) return;
  lastTickUs += TICK_US;
  if ((uint32_t)(now - lastTickUs) > TICK_US * 4) lastTickUs = now;   // resync if we fell behind

  switch (gType) {
    case T_DRIFT:      tickDriftOrHold(true);  break;
    case T_WANDER:     tickWander();           break;
    case T_TURBULENCE: tickTurbulence();       break;
    case T_LORENZ:     tickLorenz();           break;
    default:           tickDriftOrHold(false); break;   // T_HOLD
  }
}

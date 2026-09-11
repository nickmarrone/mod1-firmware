/*
  CLEPZ  --  step CV / random / smooth-noise LFO generator for the HAGIWO MOD1
  Released under CC0.  MOD1 hardware by HAGIWO (https://note.com/solder_state/n/nc05d8e8fd311)

  Inspired by the interface of Noise Engineering's Clep Diaz.  Not affiliated with or endorsed
  by Noise Engineering; this is an independent DIY firmware for different hardware.

  Clep Diaz has two 3-position switches, an encoder with a push, and six jacks.  MOD1 has one
  button, three pots and four jacks, so:
    - the two switches become button gestures (short press = mode, long press = direction)
    - the encoder becomes POT1 (count / amplitude); fully CCW is the encoder's mute
    - the two spare pots pick up clock tempo/division and slew
    - BOC and the bipolar output are dropped (MOD1 has no negative rail and no spare jack);
      beginning-of-cycle is shown on the LED instead

  --Pin assign---
  POT1  A0   step count (Step/Random) or amplitude (LFO).  Fully CCW = muted
  POT2  A1   internal tempo when nothing is patched to F1, clock divider when it is
  POT3  A2   slew / glide
  F1    A3   clock in  (rising edge, the only fast DC-coupled input on the MOD1)
  F2    A4   reset in  (read as analog with hysteresis: F2 has a 1uF cap to ground)
  F3    A5   CV in: adds to the step count (adds to amplitude in LFO mode)
  F4    D11  unipolar CV out, 0..5V
  BUTTON D4  short press = mode, long press = direction, very long press = reset / re-roll
  LED   D3   output level, beginning-of-cycle flash, and mode/direction confirmations

  D9 and D10 are deliberately left as inputs: on the MOD1 they sit on the same nets as A4/A5,
  so driving them would fight the reset and CV inputs.  Only Timer2 is reconfigured.
*/

#include <EEPROM.h>
#include <avr/pgmspace.h>

// ---------------------------------------------------------------- pins
#define PIN_LED      3
#define PIN_BUTTON   4
#define PIN_F2_ALT   9     // shares the F2 net with A4 - must stay an input
#define PIN_F3_ALT   10    // shares the F3 net with A5 - must stay an input
#define PIN_OUT      11    // F4, OC2A
#define PIN_CLOCK    17    // F1 = A3 = D17

// ---------------------------------------------------------------- config
#define TICK_US        500UL     // 2 kHz output tick
#define CLK_DEBOUNCE_MS  2UL
#define CLK_TIMEOUT_MS   3000UL  // no edge for this long -> fall back to the internal clock
#define PRESS_SHORT_MS   400UL
#define PRESS_VLONG_MS   1500UL
#define DEBOUNCE_MS      30UL
#define EE_SAVE_DELAY_MS 2000UL
#define HINT_MS          800UL
#define BOC_FLASH_MS     20UL

#define EE_ADDR_MODE 0
#define EE_ADDR_DIR  1

#define MAX_STEPS      32        // Up / Down; Up-Down ping-pongs these into a 62 step cycle
#define RESET_HI       410       // ~2.0 V on a 0..1023 ADC
#define RESET_LO       205       // ~1.0 V

enum { M_STEP = 0, M_RAND, M_LFO };
enum { D_UP = 0, D_UPDN, D_DN };

// Clock dividers selected by POT2 when an external clock is present
static const uint8_t DIVIDERS[8] PROGMEM = { 1, 2, 3, 4, 6, 8, 12, 16 };

// Internal tempo, ms per step: 2000 ms (30 BPM) down to 62 ms (~960 BPM), exponential
static const uint16_t TEMPO_TAB[17] PROGMEM = {
  2000, 1610, 1296, 1043, 839, 675,
  544, 438, 352, 283, 228, 184,
  148, 119, 96, 77, 62
};

// ---------------------------------------------------------------- state
static uint8_t gMode = M_STEP;
static uint8_t gDir  = D_UP;

// ADC round robin.  Reset (channel 4) is sampled every other slot so it stays responsive.
static const uint8_t ADC_SEQ[8] PROGMEM = { 0, 4, 1, 4, 2, 4, 5, 4 };
static uint8_t  adcSlot = 0;
static uint16_t adcVal[6];        // indexed by mux channel; 3 is unused (F1 is digital)

// clock
static bool     extClock = false;
static uint32_t lastEdgeMs = 0;
static uint32_t lastAdvanceMs = 0;
static uint16_t extPeriodMs = 500;
static uint16_t effPeriodMs = 500;   // period between step advances, after division
static uint8_t  divCounter = 0;
static uint8_t  clkPrev = LOW;
static uint32_t clkEdgeGuardMs = 0;

// sequence
static uint8_t  stepVals[MAX_STEPS];
static uint8_t  gCount = 8;
static uint8_t  stepInCycle = 0;
static uint16_t engineTarget = 0;    // 16 bit target the slew chases

// LFO
static uint16_t lfoFrom = 32768, lfoTo = 32768;
static uint32_t lfoStartMs = 0;

// slew
static uint16_t slewOut = 0;
static uint16_t slewK = 65535;
static uint16_t slewKPot = 0x7FFF;
static uint16_t slewKPeriod = 0;

// button
static uint32_t btnChangedMs = 0, btnDownMs = 0;
static uint8_t  btnLastRead = HIGH, btnStable = HIGH;
static bool     vlongFired = false;

// LED
static uint8_t  patLeft = 0, patBright = 0;
static uint16_t patOnMs = 0, patOffMs = 0;
static bool     patOn = false;
static uint32_t patNextMs = 0;
static uint32_t hintUntilMs = 0;
static uint32_t bocUntilMs = 0;

// misc
static uint32_t rng = 0x9E3779B9UL;
static uint32_t lastTickUs = 0;
static uint32_t eeDirtyMs = 0;
static bool     eeDirty = false;
static uint8_t  resetHigh = 0;

// ---------------------------------------------------------------- helpers
static inline uint32_t rnd32() {
  rng ^= rng << 13;
  rng ^= rng >> 17;
  rng ^= rng << 5;
  return rng;
}
static inline uint8_t rnd8() { return (uint8_t)(rnd32() >> 24); }

static inline void ledSet(uint8_t v) {
  if (v == 0) {
    TCCR2A &= (uint8_t)~(1 << COM2B1);     // OC2B at 0 still leaks a 1/256 sliver
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

// symmetric S curve, peak slope 1.5x the average
static inline uint16_t easeSym(uint16_t p) {
  uint32_t u2 = ((uint32_t)p * p) >> 16;          // p^2 in Q16
  uint32_t u3 = (u2 * p) >> 16;                   // p^3 in Q16
  uint32_t v  = 3UL * u2 - 2UL * u3;              // always >= 0 because u3 <= u2
  return (v > 65535UL) ? (uint16_t)65535 : (uint16_t)v;
}

// gentler: halfway to linear, peak slope ~1.25x -- the segment takes its time
static inline uint16_t easeMild(uint16_t p)  { return (uint16_t)(((uint32_t)p + easeSym(p)) >> 1); }
// steeper: S curve applied twice, peak slope ~2.25x -- the segment snaps across
static inline uint16_t easeSteep(uint16_t p) { return easeSym(easeSym(p)); }

static inline uint16_t lerp16(uint16_t a, uint16_t b, uint16_t s) {
  if (b >= a) return a + (uint16_t)(((uint32_t)(b - a) * s) >> 16);
  return a - (uint16_t)(((uint32_t)(a - b) * s) >> 16);
}

// ---------------------------------------------------------------- setup
void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_CLOCK, INPUT);
  pinMode(PIN_F2_ALT, INPUT);        // never drive these: same nets as A4 / A5
  pinMode(PIN_F3_ALT, INPUT);

  uint32_t s = 0;
  for (uint8_t i = 0; i < 16; i++) s = (s << 1) ^ (uint32_t)(analogRead(A6) & 1);
  rng = s ? (s ^ micros()) : 0x9E3779B9UL;

  // Timer2 only: fast PWM, no prescaler -> 62.5 kHz on D11 (OC2A) and D3 (OC2B).
  // Timer1 is left alone so D9/D10 stay high impedance.
  TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1) | (1 << COM2B1);
  TCCR2B = (1 << CS20);
  OCR2A = 0;

  uint8_t m = EEPROM.read(EE_ADDR_MODE);
  uint8_t d = EEPROM.read(EE_ADDR_DIR);
  gMode = (m <= M_LFO) ? m : (uint8_t)M_STEP;
  gDir  = (d <= D_DN)  ? d : (uint8_t)D_UP;

  for (uint8_t i = 0; i < MAX_STEPS; i++) stepVals[i] = rnd8();

  DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC4D) | (1 << ADC5D);
  ADMUX  = (1 << REFS0) | 0;
  ADCSRA |= (1 << ADSC);

  lastEdgeMs = 0;
  lastAdvanceMs = millis();
  lastTickUs = micros();
}

// ---------------------------------------------------------------- ADC
static inline void serviceADC() {
  if (ADCSRA & (1 << ADSC)) return;
  uint8_t  ch = pgm_read_byte(&ADC_SEQ[adcSlot]);
  uint16_t v  = ADC;
  if (ch == 4) adcVal[4] = v;                          // reset input: raw, no lag
  else         adcVal[ch] = (uint16_t)((adcVal[ch] * 3UL + v) >> 2);   // pots / CV: de-jitter
  adcSlot = (uint8_t)((adcSlot + 1) & 7);
  ADMUX = (1 << REFS0) | pgm_read_byte(&ADC_SEQ[adcSlot]);
  ADCSRA |= (1 << ADSC);
}

// ---------------------------------------------------------------- sequence
static inline uint8_t cycleLength() {
  if (gCount <= 1) return 1;
  return (gDir == D_UPDN) ? (uint8_t)(2 * gCount - 2) : gCount;
}

static inline uint8_t indexForPosition(uint8_t pos) {
  if (gCount <= 1) return 0;
  switch (gDir) {
    case D_DN:   return (uint8_t)(gCount - 1 - pos);
    case D_UPDN: return (pos < gCount) ? pos : (uint8_t)(2 * gCount - 2 - pos);
    default:     return pos;
  }
}

static void applyStepValue() {
  uint8_t idx = indexForPosition(stepInCycle);
  if (gMode == M_RAND) {
    engineTarget = (uint16_t)stepVals[idx] << 8;
  } else if (gCount <= 1) {
    engineTarget = 0;
  } else {
    engineTarget = (uint16_t)(((uint32_t)idx * 65535UL) / (gCount - 1));
  }
}

// POT1 + F3 CV set the LFO amplitude, which scales the target away from 0 V
static void newLfoTarget() {
  lfoFrom = slewOut;
  uint16_t amp = (uint16_t)min(1023L, (long)adcVal[0] + (long)adcVal[5]);
  lfoTo = (uint16_t)(((uint32_t)rnd8() * 257UL * amp) / 1023UL);
}

static void fireBOC() {
  bocUntilMs = millis() + BOC_FLASH_MS;
  if (gMode == M_RAND) {
    for (uint8_t i = 0; i < MAX_STEPS; i++) stepVals[i] = rnd8();  // vary every pass
  }
}

static void advanceStep() {
  lastAdvanceMs = millis();
  if (gMode == M_LFO) {
    newLfoTarget();
    lfoStartMs = lastAdvanceMs;
    if ((rnd8() & 3) == 0) bocUntilMs = lastAdvanceMs + BOC_FLASH_MS;  // random, as on the original
    return;
  }
  if (gCount == 0) return;
  uint8_t len = cycleLength();
  stepInCycle = (uint8_t)(stepInCycle + 1);
  if (stepInCycle >= len) {
    stepInCycle = 0;
    fireBOC();
  }
  applyStepValue();
}

static void resetSequence(bool reroll) {
  stepInCycle = 0;
  divCounter = 0;
  if (reroll) for (uint8_t i = 0; i < MAX_STEPS; i++) stepVals[i] = rnd8();
  if (gMode == M_LFO) {
    newLfoTarget();
    lfoStartMs = millis();
  } else {
    applyStepValue();
  }
  bocUntilMs = millis() + BOC_FLASH_MS;
}

// ---------------------------------------------------------------- clock
static void serviceClock() {
  uint32_t now = millis();

  uint8_t c = digitalRead(PIN_CLOCK);
  if (c == HIGH && clkPrev == LOW && (now - clkEdgeGuardMs) >= CLK_DEBOUNCE_MS) {
    clkEdgeGuardMs = now;
    if (extClock && lastEdgeMs != 0) {
      uint32_t p = now - lastEdgeMs;
      if (p >= 2 && p <= 10000) extPeriodMs = (uint16_t)p;
    }
    lastEdgeMs = now;
    extClock = true;

    uint8_t divN = pgm_read_byte(&DIVIDERS[adcVal[1] >> 7]);
    effPeriodMs = (uint16_t)min(60000UL, (uint32_t)extPeriodMs * divN);
    if (++divCounter >= divN) {
      divCounter = 0;
      advanceStep();
    }
  }
  clkPrev = c;

  if (extClock && (now - lastEdgeMs) > CLK_TIMEOUT_MS) {
    extClock = false;
    divCounter = 0;
  }

  if (!extClock) {
    // POT2 sets an internal tempo, 30..960 BPM, interpolated between table entries
    uint16_t pot = adcVal[1];
    uint8_t  seg = (uint8_t)(pot >> 6);            // 0..15
    uint8_t  f   = (uint8_t)(pot & 63);
    uint16_t a = pgm_read_word(&TEMPO_TAB[seg]);
    uint16_t b = pgm_read_word(&TEMPO_TAB[seg + 1]);
    effPeriodMs = (uint16_t)(a - (((uint32_t)(a - b) * f) >> 6));
    if ((now - lastAdvanceMs) >= effPeriodMs) advanceStep();
  }
}

// ---------------------------------------------------------------- reset input
static void serviceReset() {
  uint16_t v = adcVal[4];
  if (!resetHigh && v > RESET_HI) {
    resetHigh = 1;
    resetSequence(false);
  } else if (resetHigh && v < RESET_LO) {
    resetHigh = 0;
  }
}

// ---------------------------------------------------------------- button
static void markDirty() { eeDirty = true; eeDirtyMs = millis(); }

static void serviceButton() {
  uint32_t now = millis();
  uint8_t  r = digitalRead(PIN_BUTTON);

  if (r != btnLastRead) { btnLastRead = r; btnChangedMs = now; }
  else if (r != btnStable && (now - btnChangedMs) >= DEBOUNCE_MS) {
    btnStable = r;
    if (r == LOW) {                       // press
      btnDownMs = now;
      vlongFired = false;
    } else if (!vlongFired) {             // release without having already fired
      uint32_t held = now - btnDownMs;
      if (held < PRESS_SHORT_MS) {
        gMode = (uint8_t)((gMode + 1) % 3);
        stepInCycle = 0;
        applyStepValue();
        startPattern((uint8_t)(gMode + 1), 255, 60, 120);      // bright = mode
      } else {
        gDir = (uint8_t)((gDir + 1) % 3);
        stepInCycle = 0;
        applyStepValue();
        startPattern((uint8_t)(gDir + 1), 40, 200, 160);       // dim = direction
      }
      markDirty();
    }
  }

  if (btnStable == LOW && !vlongFired && (now - btnDownMs) >= PRESS_VLONG_MS) {
    vlongFired = true;
    resetSequence(true);
    startPattern(1, 255, 250, 100);
  }
}

// ---------------------------------------------------------------- LED
static void serviceLED() {
  uint32_t now = millis();

  if (patLeft) {                                   // mode / direction confirmation
    if ((int32_t)(now - patNextMs) >= 0) {
      if (patOn) { patOn = false; patNextMs = now + patOffMs; patLeft--; }
      else       { patOn = true;  patNextMs = now + patOnMs; }
    }
    ledSet(patLeft ? (patOn ? patBright : 0) : 0);
    return;
  }

  if ((int32_t)(now - hintUntilMs) < 0) {          // Clep Diaz's divisibility hint
    if (gCount && (gCount % 4) == 0)      ledSet(255);
    else if (gCount && (gCount % 3) == 0) ledSet(40);
    else                                  ledSet(0);
    return;
  }

  if ((int32_t)(now - bocUntilMs) < 0) { ledSet(255); return; }

  if (gCount == 0 && gMode != M_LFO) { ledSet(0); return; }
  ledSet((uint8_t)(slewOut >> 8));                 // otherwise follow the output
}

// ---------------------------------------------------------------- controls
static void serviceCount() {
  if (gMode == M_LFO) return;                       // POT1 is amplitude in LFO mode

  long fromPot = ((long)adcVal[0] * (MAX_STEPS + 1)) >> 10;   // 0..MAX_STEPS
  long fromCv  = ((long)adcVal[5] * 31L + 512L) >> 10;        // CV adds up to 31 steps
  long n = fromPot + fromCv;
  if (n > MAX_STEPS) n = MAX_STEPS;

  if ((uint8_t)n == gCount) return;
  gCount = (uint8_t)n;
  if (stepInCycle >= cycleLength()) stepInCycle = 0;
  applyStepValue();
  hintUntilMs = millis() + HINT_MS;
}

static void serviceSlew() {
  uint16_t pot = adcVal[2];
  int16_t  delta = (int16_t)pot - (int16_t)slewKPot;
  if (delta > -4 && delta < 4 && effPeriodMs == slewKPeriod) return;
  slewKPot = pot;
  slewKPeriod = effPeriodMs;
  if (pot < 8) { slewK = 65535; return; }
  // ticks in one step, scaled by the pot: tau goes from ~0 up to one whole step
  uint32_t ticks = ((uint32_t)effPeriodMs * 2UL * pot) / 1023UL;
  if (ticks < 1) ticks = 1;
  uint32_t k = 65535UL / ticks;
  slewK = (k < 1) ? 1 : (uint16_t)k;
}

// ---------------------------------------------------------------- output tick
static void outputTick() {
  uint16_t target;

  if (gMode == M_LFO) {
    uint32_t el = millis() - lfoStartMs;
    uint16_t per = effPeriodMs ? effPeriodMs : 1;
    uint16_t p = (el >= per) ? 65535 : (uint16_t)((el * 65535UL) / per);
    bool rising = (lfoTo >= lfoFrom);
    uint16_t s;
    if (gDir == D_UPDN)      s = easeSym(p);                             // symmetrical
    else if (gDir == D_UP)   s = rising ? easeMild(p) : easeSteep(p);    // gentler rise
    else                     s = rising ? easeSteep(p) : easeMild(p);    // gentler fall
    target = lerp16(lfoFrom, lfoTo, s);
  } else if (gCount == 0) {
    target = 0;                                    // POT1 fully CCW = muted
  } else {
    target = engineTarget;
  }

  if (slewK >= 65535) {
    slewOut = target;
  } else {
    int32_t d = (int32_t)target - (int32_t)slewOut;
    int32_t step = (d * (int32_t)slewK) >> 16;
    if (step == 0 && d != 0) step = (d > 0) ? 1 : -1;
    slewOut = (uint16_t)((int32_t)slewOut + step);
  }
  OCR2A = (uint8_t)(slewOut >> 8);
}

// ---------------------------------------------------------------- loop
void loop() {
  serviceADC();
  serviceClock();
  serviceReset();
  serviceButton();
  serviceCount();
  serviceSlew();
  serviceLED();

  if (eeDirty && (millis() - eeDirtyMs) > EE_SAVE_DELAY_MS) {
    eeDirty = false;
    EEPROM.update(EE_ADDR_MODE, gMode);
    EEPROM.update(EE_ADDR_DIR, gDir);
  }

  uint32_t now = micros();
  if ((uint32_t)(now - lastTickUs) < TICK_US) return;
  lastTickUs += TICK_US;
  if ((uint32_t)(now - lastTickUs) > TICK_US * 4) lastTickUs = now;
  outputTick();
}

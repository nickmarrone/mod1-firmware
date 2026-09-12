/*
  ENVELOPE FOLLOWER  --  envelope follower and gate extractor for the HAGIWO MOD1
  MOD1 hardware and the original firmware family by HAGIWO
  (https://note.com/solder_state/n/nc05d8e8fd311).  Released under CC0.

  The other MOD1 firmwares all generate.  This one listens: it takes a signal on F1 and puts out
  its amplitude envelope, a gate while that envelope sits above a threshold, and the envelope
  inverted for ducking.  In the spirit of a Doepfer A-119 or an Intellijel Audio Interface II,
  with the important difference that F1 is DC coupled and 0..5V, so this is best understood as a
  CV follower that also works on hot, externally biased audio.

  --Pin assign---
  POT1  A0   sensitivity / gain, 1x .. 33x
  POT2  A1   release time, 5 ms .. 2 s.  Attack is fixed at 1.5 ms
  POT3  A2   gate threshold
  F1    A3   signal in, 0..5V, DC coupled.  The only fast input on the board
  F2    D9   envelope out, 0..5V           (OC1A)
  F3    D10  gate out, 0 or 5V             (OC1B)
  F4    D11  inverted envelope out, 0..5V  (OC2A)
  BUTTON D4  short press: AC / DC mode.  Hold 1 s: re-zero the bias estimate
  LED   D3   dim meter following the envelope, full bright while the gate is high
  EEPROM     the AC / DC mode at address 0, a version stamp at 1

  Two modes, because the input is DC coupled and there is no way to know what is patched:

    AC   the rectifier's zero point is a slowly tracked estimate of the input's own average, so
         audio biased anywhere reads correctly and an LFO reads as its swing.  High passed at
         about 0.8 Hz, so a signal slower than that is partly tracked out as bias.
    DC   the envelope is a slew limited follower of the absolute input level, so a standing CV
         reads as a level rather than as silence.

  The engine runs in two rates.  A 20 kHz fast tick samples F1 and peak detects it; a 1.25 kHz
  control tick does the attack / release filtering, the gate and the outputs.  Peak detecting fast
  and smoothing slow is what keeps the time constants honest without 20 kHz arithmetic, and
  1.25 kHz is still eight times the 159 Hz reconstruction filter on the MOD1 board.  Output is
  8 bit through the 62.5 kHz PWM, as in the other firmwares.
*/

#include <EEPROM.h>
#include <avr/pgmspace.h>

// ---------------------------------------------------------------- pins
#define PIN_LED     3
#define PIN_BUTTON  4
#define PIN_OUT1    9    // OC1A, F2, envelope
#define PIN_OUT2    10   // OC1B, F3, gate
#define PIN_OUT3    11   // OC2A, F4, inverted envelope

// ---------------------------------------------------------------- config
#define FAST_US          50UL    // 20 kHz sampling tick
#define CTRL_DIV         16      // control tick every 16 fast ticks -> 1.25 kHz, window of 16

#define DEBOUNCE_MS      30UL
#define PRESS_LONG_MS    1000UL  // held this long re-zeros the bias estimate instead
#define EE_SAVE_DELAY_MS 2000UL

#define ENV_FULL         0xFFFFFFUL   // the envelope is 24 bit, emitted as env >> 16
#define ATT_COEF         27090U       // one pole coefficient for a 1.5 ms attack at 1.25 kHz

// Gate threshold from POT3.  Fully anticlockwise is the lowest *usable* threshold, about 40 mV of
// envelope, rather than zero: a threshold of zero would just hold the gate permanently high.
#define THR_MIN          131072UL
#define THR_STEP         16100UL      // puts a full clockwise pot at ~99 % of full scale
#define GATE_MIN_TICKS   7            // ~5.6 ms minimum gate width, so a tail cannot chatter it

#define DC_SHIFT         8            // bias tracker time constant, ~205 ms at 1.25 kHz

#define EE_ADDR_MODE     0
#define EE_ADDR_VER      1
#define EE_VERSION       1

enum { M_AC = 0, M_DC, NUM_MODES };

// ---------------------------------------------------------------- tables
// One pole release coefficient in Q16: 65536 * (1 - exp(-0.0008 / tau)), tau exponential from
// 5 ms to 2 s across 33 entries, interpolated.  Descending, so a is always >= b in relCoefFrom.
static const uint16_t REL_COEF[33] PROGMEM = {
  9690, 8143, 6828, 5715,
  4775, 3985, 3323, 2767,
  2303, 1916, 1593, 1323,
  1099,  913,  758,  629,
   522,  433,  360,  298,
   247,  205,  170,  141,
   117,   97,   81,   67,
    55,   46,   38,   32,
    26
};

// ---------------------------------------------------------------- state
static uint8_t  gMode = M_AC;

// ADC.  F1 takes every slot but one in 64, where a single pot is refreshed in rotation, so each
// pot lands every 9.6 ms and one F1 sample in 64 is a held repeat.
static uint16_t adcVal[3];            // pots, de-jittered
static uint16_t sigRaw = 0;           // latest F1 sample, raw: the follower wants no lag
static uint8_t  adcPend = 3;          // channel whose conversion is in flight
static uint8_t  potNext = 0;

// engine
static uint16_t winSum  = 0;          // 16 samples of 0..1023 still fits a uint16
static uint16_t winPeak = 0;
static int32_t  dcQ8 = 0;             // tracked input bias, counts << 8
static int16_t  dcNow = 0;            // the rectifier's zero point, in counts
static uint32_t env = 0;              // 0 .. ENV_FULL
static bool     gateHigh = false;
static uint8_t  gateTicks = 0;

// button and LED
static uint8_t  btnLastRead = HIGH, btnStable = HIGH;
static uint32_t btnChangedMs = 0, btnDownMs = 0;
static bool     longFired = false;
static uint8_t  patLeft = 0, patBright = 0;
static uint16_t patOnMs = 0, patOffMs = 0;
static bool     patOn = false;
static uint32_t patNextMs = 0;

// misc
static uint32_t lastTickUs = 0;
static uint16_t fastTicks = 0;
static uint8_t  ctrlDiv = 0;
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

// Square law, so the useful part of the gain range is not all crowded into the last few degrees
// of travel.  256 is unity, full clockwise is about 33x.
static inline uint16_t gainFrom(uint16_t pot) {
  return (uint16_t)(256 + (((uint32_t)pot * pot) >> 7));
}

static inline uint16_t relCoefFrom(uint16_t pot) {
  uint8_t  i = (uint8_t)(pot >> 5);
  uint16_t a = pgm_read_word(&REL_COEF[i]);
  uint16_t b = pgm_read_word(&REL_COEF[i + 1]);
  uint8_t  f = (uint8_t)(pot & 31);
  return (uint16_t)(a - (uint16_t)(((uint32_t)(a - b) * f) >> 5));
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
    EEPROM.update(EE_ADDR_MODE, M_AC);
    EEPROM.update(EE_ADDR_VER,  EE_VERSION);
  }
  uint8_t m = EEPROM.read(EE_ADDR_MODE);
  gMode = (m < NUM_MODES) ? m : (uint8_t)M_AC;
}

void setup() {
  pinMode(PIN_OUT1, OUTPUT);
  pinMode(PIN_OUT2, OUTPUT);
  pinMode(PIN_OUT3, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  // A3 stays an input, and so do A4 and A5: they share the F2 and F3 nets with D9 and D10.

  configurePWM();
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = 255;            // F4 is the inverted envelope, so silence is full scale

  loadSettings();

  // Prime the filtered values before taking the converter over, so the first control tick has
  // real pot positions and a real bias estimate instead of walking up from zero.
  adcVal[0] = analogRead(A0);
  adcVal[1] = analogRead(A1);
  adcVal[2] = analogRead(A2);
  sigRaw    = analogRead(A3);
  dcQ8      = (int32_t)((uint32_t)sigRaw << 8);
  dcNow     = (gMode == M_AC) ? (int16_t)sigRaw : (int16_t)0;

  DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D);
  adcPend = 3;
  ADMUX  = (1 << REFS0) | adcPend;
  // Arduino's init() leaves the prescaler at /128, which is 104 us a conversion.  /16 gives a
  // 1 MHz ADC clock and ~13 us, which is what makes a 50 us sampling tick possible at all.
  ADCSRA = (uint8_t)((ADCSRA & (uint8_t)~0x07) | 0x04);
  ADCSRA |= (1 << ADSC);

  lastTickUs = micros();
}

// ---------------------------------------------------------------- ADC
static inline void serviceADC() {
  if (ADCSRA & (1 << ADSC)) return;     // still converting: hold the sample, keep the channel
  uint16_t v = ADC;
  if (adcPend == 3) sigRaw = v;                                          // F1: raw
  else adcVal[adcPend] = (uint16_t)((adcVal[adcPend] * 3UL + v) >> 2);   // pots: de-jitter

  if ((fastTicks & 63) == 0) {
    adcPend = potNext;
    potNext = (uint8_t)((potNext + 1) % 3);
  } else {
    adcPend = 3;
  }
  ADMUX = (1 << REFS0) | adcPend;
  ADCSRA |= (1 << ADSC);
}

// ---------------------------------------------------------------- button
static void serviceButton() {
  uint32_t now = millis();
  uint8_t  r = digitalRead(PIN_BUTTON);

  if (r != btnLastRead) { btnLastRead = r; btnChangedMs = now; }
  else if (r != btnStable && (now - btnChangedMs) >= DEBOUNCE_MS) {
    btnStable = r;
    if (r == LOW) {                       // press
      btnDownMs = now;
      longFired = false;
    } else if (!longFired) {              // any release before the hold fired toggles the mode
      gMode = (uint8_t)((gMode + 1) % NUM_MODES);
      markDirty();
      startPattern((uint8_t)(gMode == M_AC ? 2 : 1), 255, 60, 120);   // two pulses AC, one DC
    }
  }

  // A long hold snaps the bias estimate to the input instead of waiting ~200 ms for it to settle.
  // It runs in DC mode too, where it primes the estimate for the next switch to AC.
  if (btnStable == LOW && !longFired && (now - btnDownMs) >= PRESS_LONG_MS) {
    longFired = true;
    dcQ8  = (int32_t)((uint32_t)sigRaw << 8);
    dcNow = (gMode == M_AC) ? (int16_t)sigRaw : (int16_t)0;
    startPattern(3, 40, 60, 120);                                    // dim, so it reads as other
  }
}

// ---------------------------------------------------------------- LED
static void serviceLED() {
  uint32_t now = millis();

  if (patLeft) {                                   // mode confirmation
    if ((int32_t)(now - patNextMs) >= 0) {
      if (patOn) { patOn = false; patNextMs = now + patOffMs; patLeft--; }
      else       { patOn = true;  patNextMs = now + patOnMs; }
    }
    ledSet(patLeft ? (patOn ? patBright : 0) : 0);
    return;
  }

  // A dim meter following the envelope, snapping to full bright while the gate is high, so one
  // LED shows both the level and where the threshold sits in it.
  ledSet(gateHigh ? 255 : (uint8_t)(env >> 18));
}

// ---------------------------------------------------------------- engine
static void controlTick() {
  uint16_t mean = (uint16_t)(winSum >> 4);
  uint16_t peak = winPeak;
  winSum  = 0;
  winPeak = 0;

  // The bias estimate is tracked in both modes, so switching to AC does not have to re-settle
  dcQ8 += (((int32_t)((uint32_t)mean << 8) - dcQ8) >> DC_SHIFT);

  uint16_t gainQ8 = gainFrom(adcVal[0]);
  uint32_t t;
  if (gMode == M_AC) {
    dcNow = (int16_t)(dcQ8 >> 8);
    t = (uint32_t)peak * gainQ8;                   // full scale swing at unity gain is 512 counts
    t = (t >= 131072UL) ? ENV_FULL : (t << 7);
  } else {
    dcNow = 0;                                     // rectify about 0 V: the level itself
    t = (uint32_t)mean * gainQ8;
    t = (t >= 262144UL) ? ENV_FULL : (t << 6);
  }

  // One pole, 24 bit state.  The 16 bits below the output's LSB are what let a 2 s time constant
  // move at all.  The shifts keep the product inside an int32 and still leave 4096 counts of
  // error enough to move the envelope, so it never stalls short of its target.
  uint16_t coef = (t > env) ? ATT_COEF : relCoefFrom(adcVal[1]);
  int32_t  err  = (int32_t)t - (int32_t)env;
  int32_t  nxt  = (int32_t)env + ((((err >> 10) * (int32_t)coef) >> 6));
  if (nxt < 0) nxt = 0;
  else if (nxt > (int32_t)ENV_FULL) nxt = (int32_t)ENV_FULL;
  env = (uint32_t)nxt;

  uint32_t thr = THR_MIN + (uint32_t)adcVal[2] * THR_STEP;
  if (!gateHigh) {
    if (env > thr) { gateHigh = true; gateTicks = 0; }
  } else if (gateTicks < GATE_MIN_TICKS) {
    gateTicks++;
  } else if (env < thr - (thr >> 4)) {             // relative hysteresis, 1/16 of the threshold
    gateHigh = false;
  }

  uint8_t e8 = (uint8_t)(env >> 16);
  OCR1A = e8;                                      // F2, envelope
  OCR1B = gateHigh ? 255 : 0;                      // F3, gate
  OCR2A = (uint8_t)(255 - e8);                     // F4, inverted envelope

  serviceButton();
  serviceLED();

  if (eeDirty && (millis() - eeDirtyMs) > EE_SAVE_DELAY_MS) {
    eeDirty = false;
    EEPROM.update(EE_ADDR_MODE, gMode);
  }
}

// ---------------------------------------------------------------- loop
void loop() {
  uint32_t now = micros();
  if ((uint32_t)(now - lastTickUs) < FAST_US) return;
  lastTickUs += FAST_US;
  if ((uint32_t)(now - lastTickUs) > FAST_US * 4) lastTickUs = now;   // resync if we fell behind

  fastTicks++;
  serviceADC();

  // Exactly one accumulation per fast tick, so the window always holds 16 samples even on the
  // tick where the converter was busy with a pot and sigRaw is a held repeat.  Without the hold
  // the mean would dip 6 % once every four windows, which DC mode would put on the output.
  winSum += sigRaw;
  int16_t d = (int16_t)sigRaw - dcNow;
  if (d < 0) d = (int16_t)-d;
  if ((uint16_t)d > winPeak) winPeak = (uint16_t)d;

  if (++ctrlDiv >= CTRL_DIV) { ctrlDiv = 0; controlTick(); }
}

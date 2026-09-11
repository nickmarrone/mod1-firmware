/*
  TRIPLE LFO  --  3 channel LFO for the HAGIWO MOD1, one waveform per channel
  Based on HAGIWO's "MOD1 3ch LFO Ver1.0".  MOD1 hardware and the original firmware by HAGIWO
  (https://note.com/solder_state/n/nc05d8e8fd311).  Released under CC0.

  The original had three waveforms and all three channels had to share one of them, because a
  single 1024 byte wave table lived in RAM.  Here every shape is computed from the phase instead,
  so each channel carries its own waveform and only sine needs a table, in flash.

  --Pin assign---
  POT1  A0   LFO1 frequency   (hold BUTTON: LFO1 waveform)
  POT2  A1   LFO2 frequency   (hold BUTTON: LFO2 waveform)
  POT3  A2   LFO3 frequency   (hold BUTTON: LFO3 waveform)
  F1    A3   frequency CV in, adds to all three channels
  F2    D9   LFO1 out, 0..5V  (OC1A)
  F3    D10  LFO2 out, 0..5V  (OC1B)
  F4    D11  LFO3 out, 0..5V  (OC2A)
  BUTTON D4  hold to edit waveforms.  A short tap does nothing
  LED   D3   LFO1 output, or the waveform preview while editing
  EEPROM     one waveform per channel, addresses 0, 1, 2

  Waveforms, in pot order from fully CCW to fully CW:

    0 TRIANGLE    1 SQUARE    2 SINE    3 SAW UP    4 SAW DOWN    5 STEPPED RANDOM

  Holding the button freezes all three frequencies and hands the pots to the waveforms.  A pot
  does nothing until it has moved 24 counts, so holding the button alone changes nothing.  The
  LED previews the shape of the channel whose pot you last moved, at about 1.5 Hz; until you move
  one it sits at a steady dim level to show you are in edit mode.  On release, a channel whose pot
  you moved keeps its old frequency until that pot is turned back through where it started.

  STEPPED RANDOM draws 8 new levels per cycle, one per eighth of the phase, each held flat.

  Frequency is 0.02 to 5 Hz per pot, with the same range again added from F1, unchanged from the
  original.  The engine runs at 2.5 kHz, 8 bit out through the 62.5 kHz PWM and the 1uF/1k
  reconstruction filter on the MOD1 board.
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
#define TICK_US          400UL   // 2500 Hz engine tick
#define NUM_WAVES        6
#define DEBOUNCE_MS      30UL
#define HOLD_MS          300UL   // button held this long enters waveform edit
#define EE_SAVE_DELAY_MS 2000UL

#define ARM_DELTA        24      // pot travel needed before it takes a waveform
#define PICKUP_WINDOW    12      // how close counts as catching the old position
#define ZONE_HYST        12      // travel past a zone boundary before the zone changes
#define MOVE_DELTA       3       // pot travel that counts as "this is the pot I am holding"

#define EDIT_IDLE_LED    24      // steady dim while editing, before any pot has moved

// Phase increment per tick.  0.02 Hz is 34360, and the pot adds 8363 per ADC count, which puts
// a full clockwise pot at 5.0 Hz.  F1 contributes the same expression on top.
#define INC_MIN          34360UL
#define INC_PER_COUNT    8363UL
#define PREVIEW_INC      2576980UL   // ~1.5 Hz for the LED preview

#define RND_SHIFT        13      // phase >> 13 gives 8 slices per cycle

enum { W_TRI = 0, W_SQR, W_SIN, W_SAWUP, W_SAWDN, W_RND };

// ---------------------------------------------------------------- tables
// (sin(2*pi*i/256) + 1) * 127.5, rounded
static const uint8_t SINE_TAB[256] PROGMEM = {
  128, 131, 134, 137, 140, 143, 146, 149,
  152, 155, 158, 162, 165, 167, 170, 173,
  176, 179, 182, 185, 188, 190, 193, 196,
  198, 201, 203, 206, 208, 211, 213, 215,
  218, 220, 222, 224, 226, 228, 230, 232,
  234, 235, 237, 238, 240, 241, 243, 244,
  245, 246, 248, 249, 250, 250, 251, 252,
  253, 253, 254, 254, 254, 255, 255, 255,
  255, 255, 255, 255, 254, 254, 254, 253,
  253, 252, 251, 250, 250, 249, 248, 246,
  245, 244, 243, 241, 240, 238, 237, 235,
  234, 232, 230, 228, 226, 224, 222, 220,
  218, 215, 213, 211, 208, 206, 203, 201,
  198, 196, 193, 190, 188, 185, 182, 179,
  176, 173, 170, 167, 165, 162, 158, 155,
  152, 149, 146, 143, 140, 137, 134, 131,
  128, 124, 121, 118, 115, 112, 109, 106,
  103, 100,  97,  93,  90,  88,  85,  82,
   79,  76,  73,  70,  67,  65,  62,  59,
   57,  54,  52,  49,  47,  44,  42,  40,
   37,  35,  33,  31,  29,  27,  25,  23,
   21,  20,  18,  17,  15,  14,  12,  11,
   10,   9,   7,   6,   5,   5,   4,   3,
    2,   2,   1,   1,   1,   0,   0,   0,
    0,   0,   0,   0,   1,   1,   1,   2,
    2,   3,   4,   5,   5,   6,   7,   9,
   10,  11,  12,  14,  15,  17,  18,  20,
   21,  23,  25,  27,  29,  31,  33,  35,
   37,  40,  42,  44,  47,  49,  52,  54,
   57,  59,  62,  65,  67,  70,  73,  76,
   79,  82,  85,  88,  90,  93,  97, 100,
  103, 106, 109, 112, 115, 118, 121, 124
};

// ---------------------------------------------------------------- state
static uint8_t  gWave[3] = { W_TRI, W_TRI, W_TRI };

// non blocking ADC round robin over A0..A3
static uint16_t adcVal[4];
static uint8_t  adcCh = 0;

// engine.  potInc holds the pot's contribution and stops tracking while a channel is frozen;
// the F1 CV is added live on every tick, so patched modulation keeps working during an edit.
static uint32_t phase[3];
static uint32_t potInc[3];
static bool     frozen[3]  = { false, false, false };
static uint16_t potEntry[3];
static uint16_t potSeen[3];
static int8_t   entrySign[3];
static bool     armed[3];

// stepped random.  Slot 3 belongs to the LED preview.
static uint8_t  rndVal[4];
static uint8_t  rndSlice[4];
static uint32_t rng = 0x9E3779B9UL;

// button and edit mode
static uint8_t  btnLastRead = HIGH, btnStable = HIGH;
static uint32_t btnChangedMs = 0, btnDownMs = 0;
static bool     editing = false;
static int8_t   lastTouched = -1;
static uint32_t previewPhase = 0;

// misc
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
static inline uint8_t rnd8() { return (uint8_t)(rnd32() >> 24); }

static inline uint32_t incFromRaw(uint16_t raw) {
  return INC_MIN + (uint32_t)raw * INC_PER_COUNT;
}

// OC2B at 0 still leaks a 1/256 sliver, so a square wave preview would never look fully off
static inline void ledSet(uint8_t v) {
  if (v == 0) {
    TCCR2A &= (uint8_t)~(1 << COM2B1);
    PORTD  &= (uint8_t)~(1 << PD3);
  } else {
    TCCR2A |= (1 << COM2B1);
    OCR2B = v;
  }
}

static void markDirty() { eeDirty = true; eeDirtyMs = millis(); }

// ---------------------------------------------------------------- waveforms
// slot picks the stepped random state: 0..2 are the channels, 3 is the LED preview
static uint8_t renderWave(uint8_t w, uint16_t p16, uint8_t slot) {
  switch (w) {
    case W_TRI:
      return (p16 < 32768) ? (uint8_t)(p16 >> 7)
                           : (uint8_t)(255 - ((p16 - 32768) >> 7));
    case W_SQR:
      return (p16 < 32768) ? 0 : 255;
    case W_SIN:
      return pgm_read_byte(&SINE_TAB[p16 >> 8]);
    case W_SAWUP:
      return (uint8_t)(p16 >> 8);
    case W_SAWDN:
      return (uint8_t)(255 - (p16 >> 8));
    default: {                                  // W_RND
      uint8_t s = (uint8_t)(p16 >> RND_SHIFT);
      if (s != rndSlice[slot]) {
        rndSlice[slot] = s;
        rndVal[slot] = rnd8();
      }
      return rndVal[slot];
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

void setup() {
  pinMode(PIN_OUT1, OUTPUT);
  pinMode(PIN_OUT2, OUTPUT);
  pinMode(PIN_OUT3, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  configurePWM();

  uint32_t s = 0;
  for (uint8_t i = 0; i < 16; i++) s = (s << 1) ^ (uint32_t)(analogRead(A6) & 1);
  rng = s ? (s ^ micros()) : 0x9E3779B9UL;

  // One waveform per channel.  A chip carrying the original firmware has a valid byte at 0 and
  // junk at 1 and 2, so anything out of range falls back to triangle.
  for (uint8_t ch = 0; ch < 3; ch++) {
    uint8_t w = EEPROM.read(ch);
    gWave[ch] = (w < NUM_WAVES) ? w : (uint8_t)W_TRI;
  }

  // Prime the filtered ADC values before taking the converter over, so the first tick already
  // has real pot positions instead of walking up from zero.
  adcVal[0] = analogRead(A0);
  adcVal[1] = analogRead(A1);
  adcVal[2] = analogRead(A2);
  adcVal[3] = analogRead(A3);
  for (uint8_t ch = 0; ch < 3; ch++) potInc[ch] = incFromRaw(adcVal[ch]);

  DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D);
  adcCh = 0;
  ADMUX  = (1 << REFS0) | adcCh;
  ADCSRA |= (1 << ADSC);

  lastTickUs = micros();
}

// ---------------------------------------------------------------- ADC
static inline void serviceADC() {
  if (ADCSRA & (1 << ADSC)) return;
  uint16_t v = ADC;
  adcVal[adcCh] = (uint16_t)((adcVal[adcCh] * 3UL + v) >> 2);   // de-jitter
  adcCh = (uint8_t)((adcCh + 1) & 3);
  ADMUX = (1 << REFS0) | adcCh;
  ADCSRA |= (1 << ADSC);
}

// ---------------------------------------------------------------- edit mode
// Pot travel splits into NUM_WAVES zones, with hysteresis so a pot resting on a boundary does
// not dither between two shapes.
static uint8_t zoneFor(uint8_t ch, uint16_t raw) {
  uint8_t cur = gWave[ch];
  uint8_t z = (uint8_t)(((uint32_t)raw * NUM_WAVES) >> 10);
  if (z == cur) return cur;
  uint16_t bound = (uint16_t)((((uint32_t)(z > cur ? z : cur)) << 10) / NUM_WAVES);
  if (z > cur) return (raw >= bound + ZONE_HYST) ? z : cur;
  return (raw + ZONE_HYST <= bound) ? z : cur;
}

static void beginEdit() {
  editing = true;
  lastTouched = -1;
  previewPhase = 0;
  for (uint8_t ch = 0; ch < 3; ch++) {
    potEntry[ch] = adcVal[ch];
    potSeen[ch]  = adcVal[ch];
    armed[ch] = false;
    frozen[ch] = true;          // potInc stops tracking and holds its current value
  }
}

static void endEdit() {
  editing = false;
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (armed[ch]) {
      // stay frozen until the pot is turned back through where the edit started
      entrySign[ch] = (adcVal[ch] >= potEntry[ch]) ? 1 : -1;
    } else {
      frozen[ch] = false;
    }
  }
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
    if (z != gWave[ch]) {
      gWave[ch] = z;
      markDirty();
    }
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

// ---------------------------------------------------------------- button
static void serviceButton() {
  uint32_t now = millis();
  uint8_t  r = digitalRead(PIN_BUTTON);

  if (r != btnLastRead) { btnLastRead = r; btnChangedMs = now; }
  else if (r != btnStable && (now - btnChangedMs) >= DEBOUNCE_MS) {
    btnStable = r;
    if (r == LOW) btnDownMs = now;      // a release shorter than HOLD_MS does nothing
    else if (editing) endEdit();
  }

  if (btnStable == LOW && !editing && (now - btnDownMs) >= HOLD_MS) beginEdit();
}

// ---------------------------------------------------------------- engine
static void servicePots() {
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (!frozen[ch]) potInc[ch] = incFromRaw(adcVal[ch]);
  }
}

static void outputTick() {
  uint32_t cvInc = incFromRaw(adcVal[3]);

  phase[0] += potInc[0] + cvInc;
  phase[1] += potInc[1] + cvInc;
  phase[2] += potInc[2] + cvInc;

  uint8_t o1 = renderWave(gWave[0], (uint16_t)(phase[0] >> 16), 0);
  uint8_t o2 = renderWave(gWave[1], (uint16_t)(phase[1] >> 16), 1);
  uint8_t o3 = renderWave(gWave[2], (uint16_t)(phase[2] >> 16), 2);

  OCR1A = o1;
  OCR1B = o2;
  OCR2A = o3;

  if (!editing) {
    ledSet(o1);
  } else {
    previewPhase += PREVIEW_INC;
    if (lastTouched < 0) ledSet(EDIT_IDLE_LED);
    else ledSet(renderWave(gWave[lastTouched], (uint16_t)(previewPhase >> 16), 3));
  }
}

// ---------------------------------------------------------------- loop
void loop() {
  serviceADC();
  serviceButton();
  if (editing) serviceEdit();
  else         servicePickup();
  servicePots();

  if (eeDirty && (millis() - eeDirtyMs) > EE_SAVE_DELAY_MS) {
    eeDirty = false;
    for (uint8_t ch = 0; ch < 3; ch++) EEPROM.update(ch, gWave[ch]);
  }

  uint32_t now = micros();
  if ((uint32_t)(now - lastTickUs) < TICK_US) return;
  lastTickUs += TICK_US;
  if ((uint32_t)(now - lastTickUs) > TICK_US * 4) lastTickUs = now;
  outputTick();
}

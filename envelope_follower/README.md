# envelope_follower

An envelope follower and gate extractor for the HAGIWO MOD1. The other three firmwares in this
repository generate; this one **listens**. A signal on F1 comes back out as its amplitude envelope,
a gate while that envelope sits above a threshold, and the envelope inverted for ducking — the job
of a Doepfer A-119 or the utility half of an Intellijel Audio Interface II.

With one important difference, which shapes everything below: **F1 is DC coupled and 0–5 V**. There
is no input AC coupling and no gain stage on the board, so this is most honestly described as a
**CV follower that also works on hot, externally biased audio**.

## Panel

```
POT1  A0   sensitivity / gain       F1  A3   signal in  0-5 V, DC coupled
POT2  A1   release time             F2  D9   envelope out         0-5 V
POT3  A2   gate threshold           F3  D10  gate out             0 or 5 V
BUTTON D4  short press: AC / DC     F4  D11  inverted envelope    0-5 V
LED   D3   envelope level, full bright while the gate is high
```

## The two modes

The MOD1 has no switched jacks and no AC coupling, so the firmware cannot know whether F1 is
carrying audio biased at some arbitrary voltage or a plain control voltage. The button picks:

| Mode | LED on switching | What the envelope is |
|---|---|---|
| **AC** | two bright flashes | the signal rectified about **its own slowly tracked average**, then smoothed |
| **DC** | one bright flash | the signal's **absolute level**, slew limited |

**AC is the envelope follower proper.** The rectifier's zero point is a one-pole estimate of the
input's own mean, about 205 ms, so audio biased anywhere at all reads correctly and you never have
to tell the module where its virtual ground is. A triangle LFO into F1 comes back as the rectified
triangle about its own centre.

**DC is a level follower.** A standing 2 V CV reads as 2 V of envelope rather than as silence, which
is what you want when F1 is carrying an envelope or a sequencer's CV and you want it smoothed,
thresholded, or inverted. POT2 becomes a slew rate.

The mode is saved to EEPROM about two seconds after it changes.

### POT1 — sensitivity

A square-law gain from **unity to about 33×**, applied to the rectified signal *before* the
smoothing. Square law because the first few degrees of travel are where a line-level signal lives
and the rest is for quiet ones; a linear knob would have crowded everything useful into a sliver.

At unity, a signal that uses the full 0–5 V of the input reaches the full 0–5 V of the output. A
1 V peak-to-peak signal biased into the middle of the range needs about 5×.

Because the gain lands before the filter, **overdriving POT1 behaves like a limiter** rather than
like a glitch: the envelope clips flat at 5 V and the shape of everything below the ceiling is
unchanged. Sweeping POT1 clockwise on a percussive source is a usable way to flatten its dynamics.

### POT2 — release

**Attack is fixed at 1.5 ms** and POT2 sweeps release from **5.6 ms to 1.9 s**, exponentially. That
is the classic follower law: an envelope follower that is slow to rise misses the transient it was
asked to find, so the only time constant worth a knob is the one on the way down.

- Fully anticlockwise, the envelope follows individual cycles of anything below a few hundred Hz —
  a usable rectified-waveform mangler rather than an envelope.
- Around the middle it reads as an amplitude envelope, tracking notes and hits.
- Fully clockwise it is effectively a slow average, a one-knob compressor sidechain.

The coefficients come from a 33-entry table in flash, interpolated, so the knob is smooth rather
than stepped.

### POT3 — gate threshold

Where on the envelope F3 flips, from about 40 mV to 98 % of full scale. Fully anticlockwise is the
lowest *usable* threshold, not zero — a threshold of zero would simply hold the gate high forever,
which is not a setting anyone wants.

Two things stop the gate chattering:

- **Hysteresis of one sixteenth of the threshold.** The gate opens at the threshold and does not
  close until the envelope has fallen a sixteenth below it, so a signal sitting exactly on the knob
  gives one clean gate instead of a burst.
- **A minimum gate width of about 5.6 ms**, which covers the case the hysteresis does not: a decay
  sliding slowly down through the threshold band.

The gate is derived from the envelope, *after* POT2. With a long release the gate therefore stays
high through the tail — turn POT2 down for tight gates. That is a consequence of the signal flow,
not a bug, and it is also how you get a gate that lasts as long as a phrase rather than as long as
a transient.

## Jacks

- **F1** is the only fast input on the MOD1, 1k + 0.01 µF, so it is the one that gets sampled.
- **F2** is the envelope, 0–5 V.
- **F3** is the gate: 0 V or a full 5 V, through the board's 1k + 1 µF like everything else.
- **F4** is the envelope subtracted from 5 V. Patch it at a VCA to duck something in time with
  whatever is on F1. With POT1 low it is a gentle lean; wound up, it is a hard sidechain.

## LED

A **dim meter** following the envelope, which **snaps to full bright while the gate is high**. One
LED therefore shows both the level and where POT3 has put the threshold inside it: wind POT3 until
the flashes land where you want them. This is the same bright/dim convention `clepz` uses to put
two things on one indicator.

## Button

- **Short press** toggles AC and DC, confirmed by two flashes or one.
- **Held for a second**, the bias estimate snaps to the input instead of taking its usual 205 ms to
  settle, confirmed by three dim flashes. Useful after repatching F1 to a source with a very
  different bias. It works in DC mode too, where it primes the estimate for the next switch to AC.

## Implementation notes

- **Two rates, which is the structural difference from the other three sketches.** A **20 kHz** fast
  tick samples F1 and peak detects it; a **1.25 kHz** control tick does the attack/release
  filtering, the gate, the outputs and the UI. Peak detecting fast and smoothing slow is what keeps
  the time constants honest without doing 20 kHz arithmetic on an ATmega328P, and 1.25 kHz is still
  eight times the board's 159 Hz reconstruction filter.
- **The ADC prescaler is changed to /16.** Arduino's `init()` leaves it at /128, which is 104 µs a
  conversion — a 50 µs sampling tick is simply impossible at that rate. At /16 the ADC clock is
  1 MHz and a conversion takes about 13 µs.
- F1 takes every ADC slot but **one in 64**, where a single pot is refreshed in rotation, so each pot
  lands every 9.6 ms. On the tick where the converter was busy with a pot, the previous F1 sample is
  accumulated again rather than skipped: without that hold, the 16-sample window mean would dip 6 %
  once every four windows, and DC mode would put that 312 Hz ripple straight on the output. Pots
  keep the 3/4 IIR de-jitter the other sketches use; the F1 sample is taken raw.
- **The envelope is a 24-bit one-pole**, emitted as its top 8 bits. The 16 bits below the output's
  LSB are the whole point: a 2 s time constant at 1.25 kHz moves the state by a fraction of an
  output step per tick, and an 8-bit or 16-bit accumulator would simply stall. The update is
  arranged as `env += ((err >> 10) * coef) >> 6` so the product stays inside an `int32_t` while
  still leaving 4096 counts of error enough to move the envelope.
- Verified on the host rather than guessed: attack reaches 63 % in **1.6 ms** (1.5 ms asked for, the
  difference being the 800 µs tick quantisation), release spans **5.6 ms to 1941 ms** across POT2,
  a full-scale input at unity gain lands on full-scale output, and the filter neither stalls short
  of its target nor underflows on the way to zero.
- No floating point, no interrupts, no `analogRead` after `setup()`, Timer0 and `millis()` left
  alone. **3324 bytes of flash (10 %) and 71 bytes of RAM (3 %)** — the smallest of the four.
- The worst-case `loop()` time against the 50 µs fast budget **has not been measured on hardware**.
  The control tick is one 16×16 multiply, a table interpolation, three `OCR` writes and the UI, so
  it should sit well inside the budget, but that is reasoning rather than a measurement.

## Caveats

- **F1 is DC coupled.** A signal swinging either side of 0 V loses its entire negative half at the
  input pin, before the firmware sees anything. Audio must be biased into the 0–5 V window
  outside the module — a passive offset, a mixer with an offset, or any module that already outputs
  unipolar. AC mode then finds the bias wherever it is.
- **/16 exceeds the ATmega's 200 kHz full-accuracy ADC clock**, so effective resolution is nearer
  9 bits than 10. For a follower that is irrelevant. The pots share that converter, and if pot
  jitter ever shows itself the fix is a /128 prescaler on the pot slots only.
- **F1's own input filter sits at ~15.9 kHz**, well above the 10 kHz Nyquist of a 20 kHz sampler, so
  content above 10 kHz aliases down rather than being filtered out. On an envelope follower this
  reads as a slight mis-estimate of brightness, not as a tone.
- **The gate rises through a 1 ms RC** like every other output on the board, so expect roughly
  0.3–0.5 ms before a downstream gate input sees its own threshold crossed. Fine for anything
  musical; not a clock-accurate trigger.
- **AC mode high-passes at about 0.8 Hz.** An LFO slower than roughly 1 Hz is partly tracked out as
  bias and reads smaller than it is. Use DC mode for anything that slow.
- The peak detector's window is 800 µs, so for signals below about 1.25 kHz a single window sees
  only a slice of the cycle. The 1.5 ms attack integrates across a couple of windows and the
  envelope still arrives at the true peak, a hair later than the instant it occurred.

# triple_lfo

Three independent LFOs for the HAGIWO MOD1, each with its own waveform. Based on HAGIWO's *MOD1
3ch LFO Ver1.0*, extended from three shared waveforms to **six per-channel** ones.

## Panel

```
POT1  A0   LFO1 frequency           F1  A3   frequency CV in, all channels
POT2  A1   LFO2 frequency           F2  D9   LFO1 out   0-5 V
POT3  A2   LFO3 frequency           F3  D10  LFO2 out   0-5 V
BUTTON D4  hold to edit waveforms   F4  D11  LFO3 out   0-5 V
LED   D3   LFO1 output, or the waveform preview while editing
```

Each pot sets its own channel's frequency, linearly from **0.02 Hz to 5 Hz**. **F1 adds the same
range again** to all three channels at once, so a full 5 V into F1 roughly doubles every rate.

## The six waveforms

| # | Waveform | |
|---|---|---|
| 0 | **TRIANGLE** | rises over the first half of the cycle, falls over the second |
| 1 | **SQUARE** | low for the first half, high for the second |
| 2 | **SINE** | from a 256-entry table in flash |
| 3 | **SAW UP** | ramps 0 → 5 V, then snaps back |
| 4 | **SAW DOWN** | ramps 5 V → 0, then snaps back |
| 5 | **STEPPED RANDOM** | eight random levels per cycle, each held flat |

The first three keep the numbering the original firmware used, so a chip flashed with HAGIWO's
version keeps its waveform on channel 1 after the upgrade.

Stepped random draws its eight levels per LFO cycle, so the pot runs it eight times livelier than
its marking suggests: fully anticlockwise gives a new level about every six seconds rather than
every fifty. The other five waveforms are one cycle per period as usual.

## Setting a waveform

**Hold the button.** That is the button's only job — a short tap does nothing at all.

While it is held, each pot picks its own channel's waveform. The pot's travel is divided into six
equal zones, in the table order above, so fully anticlockwise is triangle and fully clockwise is
stepped random.

- **A pot does nothing until it has moved**, about 24 ADC counts. Holding the button without
  turning anything cannot change a waveform by accident.
- **The three frequencies freeze** at the values they had when you pressed. The LFOs keep running
  at their old rates while you sweep, so you hear and see each shape as you land on it. Anything
  patched into F1 still modulates them.
- **The LED previews the shape** of the channel whose pot you last moved, running at about 1.5 Hz
  on its own phase. A fade up is saw up, a fade down is saw down, a hard on/off is square, and
  random steps flicker. Until you move a pot the LED sits at a steady dim level, which is how you
  know you are in edit mode.

When you let go, a channel whose pot you moved **keeps the frequency it had before the edit**. That
pot stays inert until you turn it back through where it started, at which point it takes over
again. Channels you did not touch never stop tracking. Nothing jumps, and nothing is retuned behind
your back.

All three waveforms are saved to EEPROM, about two seconds after the last change so that sweeping
across five zone boundaries costs one write rather than five.

## Implementation notes

- Every shape except sine is a few integer operations on the phase, so there is no wave table in
  RAM. The original kept a 1024-byte table there, which is why all three channels had to share one
  waveform. Sine is a 256-byte PROGMEM table. Global variables now total **104 bytes**.
- Phase is a 32-bit accumulator per channel advanced by a fixed-point increment, so there is no
  floating point anywhere in the tick. 2.5 kHz engine tick, 8-bit out through 62.5 kHz PWM on
  Timer1 (D9/D10) and Timer2 (D11), then the board's own ~159 Hz reconstruction filter.
- The ADC is a non-blocking round-robin over A0–A3. The original called `analogRead` four times
  inside its 400 µs tick, and each of those blocks for about 104 µs, so the tick could not hold its
  schedule and the real update rate was below the 2.5 kHz the increment math assumed. Every
  frequency therefore came out proportionally low. **This version runs faster at the same pot
  position than the original did** — that is the documented 0.02–5 Hz range finally being accurate.
  The exact factor depends on how far the old tick overran, which has not been measured on
  hardware.
- Stepped random is a xorshift32 seeded from the unconnected A6 pin.
- The LED is driven through `ledSet`, which clears `COM2B1` rather than writing `OCR2B = 0`;
  otherwise a square wave preview would never look fully off.

## Caveats

- Changing a waveform mid-cycle steps the output, since the new shape is read at the phase the old
  one had reached. The output filter rounds it off.
- Saw up, saw down and square all have a hard edge once per cycle. At the top of the frequency
  range that edge carries energy above the 159 Hz output filter, so the jack is a little softer
  than the code.

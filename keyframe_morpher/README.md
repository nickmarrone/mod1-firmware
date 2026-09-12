# keyframe_morpher

A **keyframe morpher** for the HAGIWO MOD1, in the spirit of Mutable Instruments Frames. Four
scenes are stored in EEPROM, each holding one value for each of the three outputs. One knob sweeps
continuously from scene 1 to scene 4 and the outputs interpolate between them.

The point is the divergence. Turning one knob drags three destinations along three *completely
different* paths — with the factory scenes, F2 rises straight while F3 zig-zags through two full
traversals and F4 falls through a plateau, all from the same knob position. It is a macro control:
one gesture, three voices moving in ways you chose rather than ways that follow each other.

Frames has four channel knobs sitting above its big one. MOD1 has three pots, and POT1 is spent on
position, so the twelve stored values are reached by holding the button instead.

## Panel

```
POT1  A0   position, scene 1 -> 4       F1  A3   position CV in, takes over from POT1
POT2  A1   (hold BUTTON) channel        F2  D9   channel 1 out        0-5 V
POT3  A2   (hold BUTTON) value          F3  D10  channel 2 out        0-5 V
BUTTON D4  hold to edit the nearest     F4  D11  channel 3 out        0-5 V
           scene.  A tap does nothing
LED   D3   channel 1 output, or the value being edited while the button is held
```

**POT2 and POT3 do nothing while you are playing.** They only come alive while the button is held.
That is deliberate: one knob is the instrument, and the other two are the editor.

## Playing

POT1 sweeps the whole set. Fully anticlockwise is scene 1, fully clockwise is scene 4, and the
three scene boundaries fall at one third and two thirds of the travel. Interpolation is **linear
and does not wrap** — scene 4 does not lead back round to scene 1 — so the panel reads the way it
looks, and the corner at each keyframe is audible as a change of direction rather than being
smoothed away.

The factory scenes, which is what a fresh chip comes up with:

| | scene 1 | scene 2 | scene 3 | scene 4 | shape |
|---|---|---|---|---|---|
| **F2** | 0 | 85 | 170 | 255 | straight rise |
| **F3** | 0 | 255 | 0 | 255 | zig-zag, two full traversals |
| **F4** | 255 | 128 | 128 | 0 | fall through a plateau |

## Editing a scene

Hold the button for about a third of a second. The firmware latches **the scene nearest wherever
POT1 is sitting** — so park the knob on the scene you mean to change before you hold — and hands
POT2 and POT3 to it. The LED goes to a steady dim level to show you are in.

- **POT2 picks the channel**, by thirds of its travel: bottom third F2, middle F3, top F4. One,
  two or three bright flashes confirm which.
- **POT3 then sets the value**, 0 V to 5 V across its travel. The LED follows what you are writing,
  so you can set a value by eye.

Release the button. The whole set is written to EEPROM two seconds after you stop changing it, so
sweeping POT3 around does not burn the cell.

### Neither pot does anything until you turn it

This is the part that is not guessable from the panel, and it is what makes the gesture safe. Both
pots start each edit **disarmed**: they do nothing until they have been moved about 24 counts, a
deliberate turn rather than a brush.

- Holding the button and releasing it **cannot change anything.** There is no way to enter edit
  mode and have a pot's resting position silently overwrite a stored value.
- **Changing the channel disarms POT3 again.** Otherwise picking a second channel would instantly
  stamp POT3's current position onto it, and you would wipe the very values you were trying to
  audition. Turn POT3 again for each channel you want to write.

The engine keeps running throughout, and POT1 is **not** frozen and has no pot pickup on release.
The scene being edited was latched when the hold began, so a brushed POT1 cannot move the target,
and leaving the outputs live means a CV-driven sweep keeps showing your edit in motion.

## F1 — position CV

F1 replaces POT1 rather than adding to it. The MOD1 has no switched jacks, so "patched" has to be
inferred: a rise above about 0.1 V latches the jack in, and it keeps the position from then on even
when the CV falls back to 0 V, so scene 1 stays reachable from the jack. **Turning POT1 is what
hands control back.**

`clepz` does the same thing on F3 but also hands control back after three seconds of silence. That
is wrong here, and the difference is worth stating: in `clepz` the CV sets a step count and 0 V is
not a musical destination, whereas here 0 V *is* one — it is scene 1. A timeout would snatch the
sweep back to the pot every time a slow LFO dwelt at the bottom of its travel.

An LFO on F1 is the obvious patch: it makes the module an animator that walks a whole set of
voices through your four scenes. A sequencer or a random source stepping F1 turns the four scenes
into four presets with glide between them.

## Implementation notes

- **The position is expanded from 10 bits to 16** (`pos16 = (pos << 6) | (pos >> 4)`) before being
  multiplied by three to get a segment and a fraction. Without the expansion, fully clockwise
  lands short of the last scene by a visible margin; with it, the shortfall is one LSB, about
  19 mV out of 5 V.
- **Interpolation is an 8-bit lerp with a 16-bit fraction**, split by sign so the arithmetic stays
  unsigned, in the same shape as `lerp16()` in `clepz`. Verified on the host across the full sweep:
  fully anticlockwise gives the scene 1 bytes exactly, fully clockwise lands within one LSB of
  scene 4, channel 1 is monotonic end to end, and the segment index never runs off the array.
- **The channel zones have 12 counts of hysteresis**, so POT2 resting on a boundary cannot dither
  between two channels. Measured on the host, the selection moves up at 353 and 694 and back down
  at 670 and 329 — a clean deadband around each third, with all three channels reachable.
- **The EEPROM version stamp is `0x4B`, not `1`.** A chip carrying one of the other MOD1 firmwares
  can easily have a small integer sitting at address 12, which would read as a valid stamp and
  leave you with whatever junk happened to be in the twelve bytes below it. A distinctive byte
  makes that collision unlikely.
- F1 is de-jittered with the same 3/4 IIR as the pots, unlike in `envelope_follower` where it is
  taken raw. Here it is a position CV feeding three interpolators at once, so converter noise would
  show up on all three outputs simultaneously.
- The engine runs at **2 kHz**, 8 bit out through the 62.5 kHz PWM and the board's 1k/1 µF
  reconstruction filter. No floating point, no interrupts, no `analogRead()` after `setup()`,
  Timer0 and `millis()` left alone. **3134 bytes of flash (10 %) and 76 bytes of RAM (3 %)**.

## Caveats

- **There is no way to reset the scenes to the factory set from the panel.** Once you have edited
  all twelve values there is no undo; re-flashing does not help either, because the version stamp
  survives. Clearing EEPROM address 12 from a separate sketch is the way back.
- **There are exactly four scenes and no way to add or remove one.** Frames lets you place
  keyframes anywhere along the sweep and add more; here they are fixed at 0, 1/3, 2/3 and 1.
- **No slew.** A stepped CV on F1 makes the outputs step. With linear interpolation and no
  smoothing, a sample-and-hold on F1 gives hard jumps — patch a slew limiter ahead of F1 if you
  want glide between scene positions.
- **Outputs are 0–5 V**, like everything else on the MOD1. A scene is a set of three positive
  voltages; there is no negative rail and therefore no bipolar morphing.
- **Scene boundaries land at one third and two thirds of POT1's travel**, which is not marked on
  the panel and is easy to miss by a few degrees when you are aiming to edit a particular scene.
  The nearest scene is the one that gets latched, so aim for the middle of a third, not its edge.

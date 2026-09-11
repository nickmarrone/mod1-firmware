# MOD1 firmwares

Three firmwares for [HAGIWO's MOD1](https://note.com/solder_state/n/nc05d8e8fd311), the Arduino
Nano based general-purpose Eurorack CV module.

| Firmware | What it is |
|---|---|
| [`smooth_random/`](smooth_random) | Three independent smooth random voltages, five algorithms, one rate pot per channel |
| [`clepz/`](clepz) | Step CV / random / smooth-noise LFO generator, inspired by the interface of Noise Engineering's Clep Diaz |
| [`triple_lfo/`](triple_lfo) | Three independent LFOs, six waveforms, one per channel. Extends HAGIWO's own 3ch LFO |

All three target `arduino:avr:nano` and are single-file sketches, so they work with the Arduino IDE
unchanged.

## The hardware

Read off the MOD1 schematic and HAGIWO's build article.

| Control | Pin | Notes |
|---|---|---|
| POT1 / POT2 / POT3 | `A0` / `A1` / `A2` | 100k, 0–5 V |
| Button | `D4` | to ground, `INPUT_PULLUP` |
| LED | `D3` (`OC2B`) | through 10k, so it is dim; PWM brightness works |

| Jack | Pin(s) | Direction |
|---|---|---|
| **F1** | `A3` (= `D17`) | **input only.** DC coupled and fast (1k + 0.01 µF) |
| **F2** | `A4` **and** `D9` (`OC1A`) | one net, so either an analog/digital input **or** a PWM output |
| **F3** | `A5` **and** `D10` (`OC1B`) | same |
| **F4** | `D11` (`OC2A`) | **output only** |

Three things about this hardware shape both firmwares:

- **Outputs are 0–5 V only.** There is no negative rail, so there is no bipolar output to be had.
- **F2/F3/F4 each have a 1 µF cap to ground behind a 1k series resistor** — a ~159 Hz
  reconstruction filter. Great for a PWM CV output (25 mV of ripple at 62.5 kHz, 1k output
  impedance), but it also means those jacks *as inputs* see edges smeared by 200–600 µs. Read
  gates on F2/F3 as analog values with threshold hysteresis, never with `digitalRead`.
- **F1 is the only fast input,** so in both firmwares it gets whatever needs to be sampled sharply.

Both sketches drive their outputs by writing `OCR` registers directly with the timers in fast PWM
at 62.5 kHz, the same approach as HAGIWO's own firmwares.

## Building

```bash
arduino-cli core update-index && arduino-cli core install arduino:avr
tools/build.sh                     # compiles all three, prints flash / RAM
```

Current usage on the ATmega328P (30720 B flash, 2048 B RAM):

| Firmware | Flash | RAM |
|---|---|---|
| `smooth_random` | 7058 B (22 %) | 179 B (8 %) |
| `clepz` | 4886 B (15 %) | 140 B (6 %) |
| `triple_lfo` | 3682 B (11 %) | 104 B (5 %) |

## Uploading

```bash
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano smooth_random
```

Most clone Nanos ship the older bootloader and need
`--fqbn arduino:avr:nano:cpu=atmega328old` instead.

## Credits

MOD1 hardware and the original firmware family by [HAGIWO](https://note.com/solder_state).
`triple_lfo` is derived from HAGIWO's own *MOD1 3ch LFO Ver1.0*. `clepz` is an independent homage
to the *interface* of Noise Engineering's Clep Diaz — it is not affiliated with, endorsed by, or
derived from any Noise Engineering code. Released under CC0.

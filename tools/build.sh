#!/usr/bin/env bash
# Compile the MOD1 firmwares for the Arduino Nano and report flash / RAM usage.
#
#   tools/build.sh              compile everything
#   tools/build.sh clepz        compile one sketch
#
# Needs arduino-cli with the AVR core:
#   arduino-cli core update-index && arduino-cli core install arduino:avr
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FQBN="arduino:avr:nano"
CLI="${ARDUINO_CLI:-arduino-cli}"

if ! command -v "$CLI" >/dev/null 2>&1; then
  echo "arduino-cli not found. Set ARDUINO_CLI=/path/to/arduino-cli or install it." >&2
  exit 1
fi

SKETCHES=("$@")
if [ ${#SKETCHES[@]} -eq 0 ]; then SKETCHES=(smooth_random clepz triple_lfo envelope_follower); fi

status=0
for s in "${SKETCHES[@]}"; do
  echo "=== $s ==="
  if "$CLI" compile --clean --fqbn "$FQBN" --warnings all "$ROOT/$s" 2>&1 | grep -E "Sketch uses|Global variables|error:|$s.ino:"; then :; fi
  if ! "$CLI" compile --fqbn "$FQBN" "$ROOT/$s" >/dev/null 2>&1; then
    echo "  FAILED"
    status=1
  fi
  echo
done
exit $status

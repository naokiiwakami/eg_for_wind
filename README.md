# Envelope Generator for Wind Controller

## Features

- Receives MIDI channel 1
- Two-channel output for VCA (exponential curve) and VCF (linear curve)
- Four analog input parameters, 0-5V, 1023 steps:
  1. Attack
  2. Decay
  3. Total Level
  4. Reserved
- Three control modes, rotated by control switch: BOTH -> VCF -> VCA
- Reads NOTE ON/OFF with velocity and BREATH CONTROL in MIDI messages

## Schematic

![](C:\Users\naoki\Dropbox\synth\sandbox\wind_ev\eg_for_wind\schematic.png)
# Keyword Spotting (KWS) on Arduino Giga R1

This sample runs keyword spotting using TensorFlow Lite Micro.
It is designed for offline testing with embedded WAV files.

## Current flow

1. At build time, all files in `src/test_wavs/*.wav` are embedded into firmware.
2. Each WAV is parsed and validated (should be `16 kHz`, `16-bit`, `mono`).
3. The audio preprocessor model (`audio_preprocessor_int8.tflite`) converts PCM to
   `49 x 40` int8 features (`1960` values total). Short clips are zero-padded.
4. The trained classifier model in `model.cc` (`g_model`) runs inference.
5. For each sample, the app prints a per-class confidence table, then `predicted`, `true`, and `PASS/FAIL`.

## Model I/O summary

- Audio preprocessor model input: `int16[480]` (30 ms @ 16 kHz)
- Audio preprocessor model output: `int8[40]` per frame
- Number of frames per sample: `49` (20 ms stride)
- Classifier model input: `int8[1960]` (`49 x 40` flattened)
- Classifier model output: class scores for label table order

## Labels

The runtime label table is:

- `silence`
- `unknown`
- `up`
- `down`
- `left`
- `right`

If a filename prefix is not one of the labels above, the true label is treated as `unknown`.

## Required files

- `model.cc` (trained quantized model as `g_model` and `g_model_len`)
- `src/test_wavs/*.wav` test files

## WAV requirements

- Format: PCM WAV
- Sample rate: `16000 Hz`
- Channels: `mono` (1 channel)
- Bits per sample: `16-bit`
- Recommended duration: around 1 second

## Window and stride behavior

- Window = `30 ms`: each feature frame is computed from a 30 ms audio chunk
  (`480` samples at `16 kHz`).
- Stride = `20 ms`: the next frame starts 20 ms later (`320` samples), so frames overlap.
- The pipeline always produces `49` frames:
  - shorter clips are zero-padded,
  - longer clips are truncated (only the first ~1 second is used).

## Build and flash

```bash
west build -b arduino_giga_r1/stm32h747xx/m7 samples/kws -p always
west flash
```

## Naming convention for tests

True label is read from the first token in filename (before `_`):

- `right_1.wav` -> `right`
- `silence_1000ms.wav` -> `silence`
- `yes.wav` -> `unknown` (not in label list)

## Example output (per sample)

```text
--- down ---
  silence  0.0000
  unknown  0.0078
  up       0.0000
  down     0.9922
  left     0.0000
  right    0.0000
  predicted: down
  true:      down
  PASS
```
# Magnetic Encoder Capture Results

Bench captures collected with:

```powershell
python -m software.host.main --encoder-port COM8 capture-encoder
```

## 2026-04-18 Captures

Raw capture files:

- `encoder_capture_20260418_112254.csv`
- `encoder_capture_20260418_113008.csv`
- `encoder_capture_20260418_113607.csv`

Summary:

- The host capture workflow recorded AS5600 data at about `100 Hz`.
- The encoder stayed connected throughout the reviewed captures.
- Near the downward resting position, the angle returned close to zero with roughly one-count quantization noise.
- The AS5600 magnetic diagnostics continued to report a weak field:
  - `magnet_detected = 0` for most or all samples
  - `magnet_too_weak = 1`
  - `agc = 255`
- Larger manual angle checks showed asymmetric angle response:
  - A roughly `90 deg` manual side-to-side check reported about `-89 deg` on one side and about `+150 deg` on the other.
  - A roughly `45 deg` manual side-to-side check reported about `-47 deg` on one side and about `+89 deg` on the other.
- The magnitude value changed substantially with angle, which suggests the current magnet/chip geometry is not well centered or planar.

Interpretation:

The host software capture path is working, and the encoder is stable near the downward rest position. The main remaining risk is mechanical: the magnet is currently sitting on the head of the pendulum axle bolt, so it may be tilted or off-center relative to the AS5600. A more dimensionally stable magnet holder and sensor alignment feature should be designed before trusting the encoder angle over the full control range.

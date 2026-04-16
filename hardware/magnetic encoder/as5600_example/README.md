# AS5600 Magnetic Encoder Example

Use this folder for the first Arduino example sketch that reads the AS5600 magnetic encoder.

Example sketch location:

- `hardware/magnetic encoder/as5600_example/AS5600_demo/AS5600_demo.ino`

Record the wiring, Arduino board, library version, and any calibration notes here after the example code is tested.

If the sketch prints only `0    0` for the angle readings, first check that the code is using `AS5600 as5600;` for a standard AS5600 board instead of `AS5600L as5600;`. Most AS5600 breakout boards use I2C address `0x36`; the AS5600L variant is a different address-programmable part.

The diagnostic sketch in `AS5600_demo/AS5600_demo.ino` prints:

- `connected`: whether the chip responds on I2C
- `detectMagnet`: whether the AS5600 sees the magnet
- `tooWeak`: whether the magnetic field is too weak
- `tooStrong`: whether the magnetic field is too strong
- `AGC`: the chip's automatic gain setting
- `magnitude`: the measured magnetic field magnitude
- `angle` and `rawAngle`: encoder output counts

Bring-up result: the test setup connected over I2C and returned smooth angle readings, but the status bits reported `detectMagnet = 0` and `tooWeak = 1`. The observed diagnostic values included `AGC = 255` and magnitude around `279`. For control work, the most important check is whether `rawAngle` is stable at rest and sweeps smoothly through `0` to `4095` as the shaft rotates. Treat the magnet status bits as diagnostics until the final magnet and shaft geometry are installed.

## Notes To Fill In

- Arduino board:
- AS5600 breakout/module:
- Supply voltage:
- I2C pins:
- Library used:
- Magnet orientation notes:
- Raw output range:
- Angle direction convention:

# Gas Meter

ESPHome gas meter reader for a SoCalGas meter (Elster/Honeywell AC-250 with the
Advanced Meter module). A magnetometer taped to the back cover picks up the
magnet in the diaphragm mechanism. Based on
[tronikos/esphome-magnetometer-water-gas-meter](https://github.com/tronikos/esphome-magnetometer-water-gas-meter).
Nothing touches the meter except double-sided tape.

## Hardware

  - Seeed Studio XIAO ESP32C6 (no headers)
  - GY-273 magnetometer board (QMC5883P, I2C address 0x2C)
  - 26-30AWG silicone hookup wire
  - USB cable (cut) + 5V USB power supply
  - 4x self-tapping screws, ~2mm thread, 11.5mm, 3.5mm round head
  - 3M VHB 5952 tape (4991 if the case rocks on the cover)
  - Printed case, see `case/` (ABS/ASA)
  - 2.4GHz antenna: U.FL/IPEX1 to RP-SMA female bulkhead pigtail + RP-SMA antenna

## Wiring

| GY-273 | XIAO ESP32C6 |
| ------ | ------------ |
| VCC    | 3V3          |
| GND    | GND          |
| SDA    | D4 (GPIO22)  |
| SCL    | D3 (GPIO21)  |
| DRDY   | not used     |

Power cable: red to `5V`, black to `GND`. Don't plug in USB while the 5V cable
is connected. Flash over USB once, then OTA.

## ESPHome

Install the [ESPHome CLI](https://esphome.io/guides/installing_esphome) and
create `secrets.yaml` in the project folder:

```
wifi_ssid: "YOUR_2.4GHZ_WIFI_SSID"
wifi_password: "YOUR_2.4GHZ_WIFI_SSID_PASSWORD"
home_assistant_key: "HOME_ASSISTANT_API_PRE_SHARED_KEY"
```

Then `esphome run config.yaml`.

On first boot check the I2C scan in the log. `0x2C` is the QMC5883P this config
is set up for. If you see `0x0D` you have a QMC5883L; remove `qmc5883p.yaml`
from the package `files:` list.

## Placement and calibration

 1. Run the furnace or water heater and move a phone magnetometer app over the
    back cover to find the spot with the biggest swing. Usually near the
    middle.
 2. Set `hide_magnetic_field_strength_sensors: 'false'`, flash, and hold the case
    on that spot to confirm before taping it down.
 3. Clean the cover with isopropyl alcohol, mount with VHB tape. Clip the cable
    below the case so it doesn't pull on the tape.
 4. With gas flowing, press **Calibrate axis** in Home Assistant (about 3
    minutes). Do this again if the case moves or you change the magnetometer
    `range` (8G in `config.yaml`).
 5. Set **Volume per half rotation**: set `hide_half_rotations_total_sensor: 'false'`,
    write down the count and the meter reading, and after a few days divide the
    meter difference (ft³) by the count difference.
 6. Use the `set_total` action to match the meter's dial.
 7. Set both hide options back to `'true'`.

Add the total sensor to the Energy dashboard as a gas source (ft³).

## Live flow

The package's `Flow` sensor counts half rotations, and at pilot flow the meter
turns once every 11 minutes, so that number lags by minutes. It was removed.
`Flow (live)` instead works out where the meter is *within* a rotation from
the Z waveform, which makes it respond in seconds. `Heat Input` is the same
thing in BTU/h. The code is in `gas_live_flow.h`.

How it works, roughly:

- A diaphragm meter is positive displacement, so the field only depends on how
  far the mechanism has turned. One cycle of the waveform was recorded from the
  water heater pilot; that maps a field reading back to a position at any flow.
- A Kalman filter tracks position, rate (the flow), and the field's offset and
  amplitude (they drift with temperature) from the 2 Hz readings. Around the
  peaks and troughs the waveform is flat and the field says nothing about
  position, so it coasts there and picks up a flow change once the field moves
  again. It also checks the rate against the positions it measured over the
  last 20 seconds; that's how something slow like the pilot going out gets
  caught.
- The waveform shape gets re-learned from steady slow flow (the pilot, in
  practice) and is saved across restarts together with the offset and
  amplitude. A shape that looks clearly different is only accepted once it
  repeats. `Reset Live Flow Shape` goes back to the recorded one but keeps the
  calibration.
- A single wild reading (bad I2C read) is ignored. If the field moves out of
  the calibrated range and stays there, say the case got bumped, the flow reads
  zero until it has seen a full swing at the new level and then it
  recalibrates itself. At pilot that takes about 20 minutes.
- `Calibrate axis` recalibrates it too. Same axis keeps the learned shape, a
  new axis starts from the recorded one.
- After a restart it takes about two minutes to climb up to the pilot reading.
- `Total` still comes from the counts and is the number to trust long-term.

## Diagnostics

`hide_diagnostics: 'true'` in `substitutions` hides the tuning and debug stuff
from Home Assistant: the live flow internals (`Live Flow Offset`, `Amplitude`,
`Noise`, `Shape Updates`, `Learning`, `Update Time`), Uptime, Reset Live Flow
Shape, and the package's Magnet Span, thresholds, Calibration time and minimal
axis range, Axis and Detection algorithm. They still run on the device. Set it
to `'false'` and flash to get them back.

`Magnetic Field Z (log)` is exactly what the tracker is fed, so its history
can be pulled out of Home Assistant and replayed offline. It adds about 170k
rows a day to the database, so the **Z Log** switch turns it on and off (no
flash needed; the setting survives restarts).

## Wi-Fi / antenna

External 2.4GHz antenna: U.FL (IPEX1) to RP-SMA bulkhead pigtail plus an RP-SMA
antenna through the case wall. The XIAO's RF switch is set in `config.yaml`:
GPIO3 LOW enables the switch, GPIO14 HIGH selects the external antenna. Set the
`rf_antenna_select` switch to `ALWAYS_OFF` to use the built-in one. **WiFi
Signal dB** at the meter should be better than about -75 dBm.

## Case

`case/gas_meter_case.scad` is parametric (OpenSCAD). The sensor sits chip-side
down on a 0.6mm membrane in the back (press-fit on two pegs plus hot glue), the
XIAO lies flat on four corner tabs about 12.6mm above the sensor chip. Mount it
with the power cable pointing down so the antenna points up.

 - `case/base.stl`: print back-face down
 - `case/lid_cap.stl`: lid with the RP-SMA bulkhead in the USB end, room for the
   pigtail coil, deep pockets for the 11.5mm screws. Print roof down, no supports.
 - `case/lid_plugs.stl`: four plugs for the screw pockets (top down; PETG or TPU)
 - `case/lid_plain.stl`: flat lid for the built-in antenna, no turret or plugs
   needed. Print outside-face down. Set the `rf_antenna_select` switch to
   `ALWAYS_OFF` to go with it.
 - `case/lid_cap_connector_test.stl`, `case/lid_plug_fit_test.stl`: test prints for
   the connector hole and the plug fit (1/2/3 dots = 0.0/0.1/0.2mm). Print these first.

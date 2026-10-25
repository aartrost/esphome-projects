# Gas Meter

Reads gas usage from the SoCalGas meter (Elster/Honeywell AC-250 with the
Advanced Meter module) by sensing the diaphragm movement through the back
cover with a magnetometer. Built on
[tronikos/esphome-magnetometer-water-gas-meter](https://github.com/tronikos/esphome-magnetometer-water-gas-meter).
Nothing is attached to the meter except double-sided tape.

## Hardware

  - Seeed Studio XIAO ESP32C6 (no headers)
  - GY-273 magnetometer board (QMC5883P, I2C address 0x2C)
  - 26-30AWG silicone hookup wire (black, orange, blue, yellow)
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

Power cable: red to `5V`, black to `GND`. Never plug in USB while the 5V
cable is connected: flash over USB first, then use OTA.

## ESPHome instructions

 - Pull project files
 - Install CLI [ESPHome](https://esphome.io/guides/installing_esphome)
 - create a `secrets.yaml` file in the project root with the following contents:
```
wifi_ssid: "YOUR_2.4GHZ_WIFI_SSID"
wifi_password: "YOUR_2.4GHZ_WIFI_SSID_PASSWORD"
home_assistant_key: "HOME_ASSISTANT_API_PRE_SHARED_KEY"
```
 - cd into project folder and run `esphome run config.yaml` to build and upload
 - First boot: check the log for the I2C scan. `0x2C` = QMC5883P (current config).
   `0x0D` = QMC5883L: remove `qmc5883p.yaml` from the package `files:` list.

## Placement and calibration

 1. Find the spot: run the furnace or water heater and move a phone magnetometer
    app (e.g. "Sensors") over the back diaphragm cover. The strongest swing is
    usually near the middle of the cover. Others report 10-20 µT on the Z axis.
 2. Temporarily set `hide_magnetic_field_strength_sensors: 'false'`, flash, and hold
    the case on that spot to confirm the swing before taping it down.
 3. Clean the cover with isopropyl alcohol and mount with VHB tape. Clip the cable
    below the case so it doesn't pull on the tape.
 4. With gas flowing steadily, press **Calibrate axis** in Home Assistant
    (takes ~180s for gas).
 5. Set **Volume per half rotation**: temporarily set
    `hide_half_rotations_total_sensor: 'false'`, note the count and the meter
    reading, and after a few days divide the meter difference (ft³) by the count
    difference. Default is 0.125 ft³; on meters where the magnet couples to the
    index drive it can be 8-16x more.
 6. Use the `set_total` action to match the meter's dial reading.
 7. Set both hide options back to `'true'`.

Add the total sensor to the Energy dashboard as a gas source (ft³).

## Live flow (experimental)

`Flow` waits for whole counts, so it lags by minutes at low flow. `Flow (live)`
reads the meter's position *within* each cycle from the Z waveform instead and
responds in seconds (code in `gas_live_flow.h`):

- A diaphragm meter is positive displacement, so the field depends only on how
  far the mechanism has turned. One cycle of the waveform (recorded from the
  water heater pilot) maps the field back to a position at any flow rate; flow
  is the slope of that position, fitted with steep parts of the waveform
  weighted most.
- It adapts by itself: offset and amplitude follow each peak and trough
  (temperature drift), and the shape is re-learned from every steady pilot
  cycle (~11 min). What it learned is saved across restarts and shown in the
  `Live Flow Offset`, `Live Flow Amplitude`, `Live Flow Shape Updates` and
  `Live Flow Learning` diagnostics.
- A clearly different shape (for example after moving the case) is only
  adopted once it repeats; `Reset Live Flow Shape` goes back to the recorded
  seed.
- `Total` and `Flow` are unaffected; `Total` remains the accurate long-term number.

## Wi-Fi / antenna

Uses an external 2.4GHz antenna: U.FL (IPEX1) to RP-SMA bulkhead pigtail plus an
RP-SMA antenna, mounted through the case wall. The XIAO's RF switch is set in
`config.yaml`: GPIO3 LOW enables the switch, GPIO14 HIGH selects the external
antenna (set the `rf_antenna_select` switch to `ALWAYS_OFF` for the built-in one).
Check **WiFi Signal dB** at the meter; better than about -75 dBm is fine.

## Case

`case/gas_meter_case.scad` is parametric (OpenSCAD). The sensor sits chip-side
down on a 0.6mm membrane in the back (press-fit on two pegs + hot glue), the XIAO
lies flat under the lid on four corner tabs, ~12.6mm above the sensor chip.

 - `case/base.stl`, `case/lid.stl`: print the base back-face down, the lid face down
 - `case/fit_test.stl`: quick print to check the PCB pockets, screw posts and cable hole
 - Waterproofing: Plasti Dip (white/grey) on the outside, masking the back face and the
   lid fit surfaces; neutral-cure silicone (GE Silicone 2) on the lid joint, cable entry
   and screw heads; liquid electrical tape or conformal coating on the solder joints;
   a silica gel packet inside.

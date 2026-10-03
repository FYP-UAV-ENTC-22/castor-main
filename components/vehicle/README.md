# vehicle

Flight controller interfacing, sensor drivers and manual drone control. Runs on
every drone's Raspberry Pi.

| Path | What it is |
|---|---|
| [`PX4-Autopilot/`](PX4-Autopilot/) | Fork. Flight firmware, including the RAPTOR neural-policy module (`src/modules/mc_raptor`). Its `msg/` definitions are what the companion's `px4_msgs` is generated from. |
| [`uwb_firmware/`](uwb_firmware/) | The `castor-localization` repo: DW3000 UWB ranging firmware for STM32 boards. The vehicle side reads it over serial or USB CDC and flashes it. |
| [`tools/`](tools/) | `mavlink_sniff.py` (receive-only MAVLink inspector) and `raptor_goto.py` (streams position targets to RAPTOR; never arms). |

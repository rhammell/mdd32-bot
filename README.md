# MDD32 Bot

A minimal two-wheeled differential-drive robot built around an ESP32-S3. The project includes the custom PCB, the 3D-printable chassis, and the firmware (packaged as an Arduino library).

The MDD32 board is a small, battery-powered controller for two-wheeled robots. It pairs an ESP32-S3 module with a dual H-bridge motor driver and connectors for two N20 gear motors with quadrature encoders. A single-cell Li-ion/LiPo battery powers the board, and it charges over USB-C. The same USB-C port is used for programming and serial debugging through the ESP32-S3's native USB.

<p align="center">
  <img src="images/board_top.png" alt="MDD32 Bot board, top view" width="700">
</p>

## Repository Layout

| Folder | Description |
| --- | --- |
| [`firmware/`](firmware/) | ESP32 firmware, packaged as the `MDD32_Bot` Arduino library, plus example, test, and tuning sketches. |
| [`hardware/`](hardware/) | KiCad project for the PCB (schematic, board layout, and footprint/symbol/3D-model libraries). |
| [`mechanical/`](mechanical/) | FreeCAD source (`mechanical/src/`) and exported STLs (`mechanical/export/`) for the chassis. |
| [`images/`](images/) | README images. |

## Board Features

- **ESP32-S3-MINI-1-N8** module with Wi-Fi and Bluetooth LE
- **Native USB-C** for programming, serial, and charging; no USB-UART bridge needed
- **Single-cell Li-ion/LiPo charging** with power-path management (TI BQ24072)
- **5 V boost converter** for the motor supply (TI TPS61023)
- **3.3 V LDO** for the logic supply (Diodes AP2114H-3.3)
- **Dual H-bridge motor driver** with per-channel current limiting (TI DRV8833)
- **Two 6-pin JST-SH motor ports** for N20 encoder motors
- **Battery voltage sensing** that only draws current while the board is switched on
- **Charge and USB-power status**, shown on LEDs and readable by the ESP32
- **Slide power switch**, plus RESET and BOOT buttons
- **4-layer, 62 × 28 mm PCB** with four M2.5 mounting holes

## Hardware

### Power

| Rail       | Source                       | Feeds                          |
|------------|------------------------------|--------------------------------|
| `VBUS`     | USB-C (5 V)                  | Battery charger input          |
| `VBAT`     | 1S Li-ion/LiPo via JST-PH 2-pin | Charger battery pin         |
| `VSYS`     | Charger power-path output, through the power switch | LDO and boost converter |
| `+3V3`     | AP2114H-3.3 LDO              | ESP32-S3 and encoders          |
| `+5V`      | TPS61023 boost (≈4.96 V)     | DRV8833 motor supply           |

- **Charger:** the BQ24072's input current limit is set to about 1.4 A (R9 = 1.1 kΩ) and its fast-charge current to about 330 mA (R10 = 2.7 kΩ). Size your battery for this charge rate, or change R10.
- **Motor current limit:** each DRV8833 channel limits at about 0.67 A (300 mΩ sense resistors).

### ESP32-S3 Pin Map

| GPIO   | Signal       | Function                              |
|--------|--------------|---------------------------------------|
| IO0    | `BOOT`       | BOOT button (strapping pin)           |
| EN     | `RESET`      | RESET button                          |
| IO10   | `VBAT_MON`   | Battery voltage, ½ divider (ADC)      |
| IO35   | `PGOOD_MON`  | USB power present (active low)        |
| IO36   | `CHG_MON`    | Charging (active low)                 |
| IO40   | `SLEEP`      | DRV8833 nSLEEP (high = enabled)       |
| IO47   | `MOT1_IN1`   | Left motor driver input 1 (AIN1)      |
| IO33   | `MOT1_IN2`   | Left motor driver input 2 (AIN2)      |
| IO38   | `MOT1_ENCA`  | Left encoder channel A                |
| IO39   | `MOT1_ENCB`  | Left encoder channel B                |
| IO48   | `MOT2_IN1`   | Right motor driver input 1 (BIN1)     |
| IO34   | `MOT2_IN2`   | Right motor driver input 2 (BIN2)     |
| IO41   | `MOT2_ENCA`  | Right encoder channel A               |
| IO42   | `MOT2_ENCB`  | Right encoder channel B               |
| IO19/20| `USB_D-`/`USB_D+` | Native USB                       |

### Connectors

| Ref | Connector            | Pinout                                              |
|-----|----------------------|-----------------------------------------------------|
| J1  | JST-PH 2-pin         | 1: `VBAT`, 2: `GND`                                 |
| J2  | USB-C (USB 2.0)      | Power and data                                      |
| J4  | JST-SH 6-pin (MOT L) | 1: `GND`, 2: `ENC_B`, 3: `ENC_A`, 4: `3V3`, 5: `M2`, 6: `M1` |
| J5  | JST-SH 6-pin (MOT R) | 1: `GND`, 2: `ENC_B`, 3: `ENC_A`, 4: `3V3`, 5: `M2`, 6: `M1` |

### Opening the PCB

1. Open `hardware/MDD32_Bot/MDD32_Bot.kicad_pro` in **KiCad 10** or newer.
2. The project's symbol, footprint, and 3D model libraries are referenced relative to the project folder, so no extra library setup is needed.
3. Each component in the schematic has an LCSC part number in its `LCSC` field for JLCPCB assembly. The [Fabrication Toolkit](https://github.com/bennymeg/Fabrication-Toolkit) plugin can generate the Gerbers, BOM, and placement file in one step.

## Firmware

The firmware lives in `firmware/MDD32_Bot/`, structured as an Arduino library with no external dependencies:

| Path | Description |
| --- | --- |
| `src/MDD32_Bot.h` / `.cpp` | Top-level `MDD32_Bot` class that ties the subsystems together. |
| `src/definitions/` | Compile-time configuration: `pin_definitions.h` (wiring) and `robot_definitions.h` (tuning/behavior). |
| `src/battery/` | `MDD32_Battery`: battery voltage and charger-status monitoring. |
| `src/drive/` | Differential drive stack: `MDD32_Motor`, `MDD32_Encoder`, `MDD32_PID`, `MDD32_Motor_Controller`, and `MDD32_Drive`. |
| `examples/Basics/` | Standalone feature demos: `BatteryTest`, `WifiTest`. |
| `examples/Tuning/` | Calibration sketches, numbered in the order they should be run: `1_FeedforwardTuning`, `2_PIDTuning`, `3_WheelSyncTuning`, `4_DistanceTuning`. |
| `tests/` | Post-build verification sketches: `ForwardTest`, `DriftTest`, `DriveCycleTest`. |

All sketches report over the Serial Monitor at 115200 baud.

### Installing

Copy or symlink `firmware/MDD32_Bot` into your Arduino `libraries` folder (for example `~/Documents/Arduino/libraries/MDD32_Bot`), and install the **esp32 by Espressif Systems** board package.

### Board settings

In the Arduino IDE, select **ESP32S3 Dev Module** and set:

- **USB CDC On Boot:** Enabled (required for Serial output over the native USB port)
- **Flash Size:** 8MB
- **PSRAM:** Disabled

The ESP32-S3 is programmed over USB-C. If the port doesn't appear, hold **BOOT**, press and release **RESET**, then release **BOOT** to enter the bootloader.

### Usage

```cpp
#include <MDD32_Bot.h>

MDD32_Bot bot;

void setup() {
    Serial.begin(115200);
    if (!bot.begin()) {
        Serial.println("MDD32 Bot init failed");
        while (1);
    }
    bot.drive.straight(6.0f);   // drive forward at 6 cm/s
}

void loop() {
    bot.drive.update();         // must be called frequently for PID control
}
```

`MDD32_Bot` exposes two subsystems: `battery` (`voltage()`, `charging()`, `chargerConnected()`) and `drive` (`straight()`, `spin()`, `stop()`, `update()`, `isMoving()`, `getTelemetry()`, `getDistance()`, `resetDistance()`).

## License

This project is licensed under the [MIT License](LICENSE).

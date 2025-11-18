# RaceChrono custom CAN preset for GR86 / BRZ Gen2

This document describes the CAN channels that the CCA Telemetry firmware exposes and how to add matching channels in RaceChrono. All equations use RaceChrono's CAN syntax where `A` is the first byte, `B` is the second byte, and so on. Byte order is Motorola (big-endian) unless noted otherwise.

## Virtual oil pressure (CAN 0x710)

The firmware publishes the analog oil-pressure input as a virtual CAN frame on ID `0x710` with a 16-bit big-endian value that represents pressure in 0.1 psi units.【F:firmware/cca_telemetry.ino†L149-L168】 Copy the block below and use *Add channel → Import from clipboard* in RaceChrono to create a ready-to-use gauge:

```
RaceChrono CAN channel
Name=Oil pressure (psi)
Short name=OilP
CAN ID=0x710
Length=2
Byte order=Motorola
Signed=No
Equation=(A*256+B)/10
Unit=psi
Frequency=25
```

### Optional kPa variant

RaceChrono can either handle the unit conversion automatically or you can create a second channel that converts the same raw value to kPa:

```
RaceChrono CAN channel
Name=Oil pressure (kPa)
Short name=OilP_kPa
CAN ID=0x710
Length=2
Byte order=Motorola
Signed=No
Equation=((A*256+B)/10)*6.89476
Unit=kPa
Frequency=25
```

## Key FT86/ZN8 CAN channels

The table below lists the most useful factory CAN IDs for the second-generation FT86 platform (GR86/BRZ). All of these IDs are whitelisted and rate-limited by the firmware for RaceChrono streaming.【F:firmware/pidmaps/gr86_2022.h†L9-L118】 Use one row per RaceChrono channel; make sure to select Motorola byte order and enter the equation exactly as shown.

| CAN ID | Signal | Bytes | RaceChrono equation | Unit | Notes |
|-------:|--------|-------|---------------------|------|-------|
| 0x118 | Engine RPM | 0-1 | `((A*256)+B)/4` | rpm | 0.25 rpm resolution, 0–16000 rpm range. |
| 0x118 | Accelerator pedal | 2 | `C/2` | % | Scales 0–100%; useful for throttle correlation. |
| 0x118 | Throttle plate | 3 | `D/2` | % | Actual throttle plate angle; 0–100%. |
| 0x118 | Coolant temperature | 4 | `E-40` | °C | Standard Toyota offset (–40°C = raw 0). |
| 0x118 | Intake air temperature | 5 | `F-40` | °C | Same offset as coolant temperature. |
| 0x138 | Engine oil temperature | 0-1 | `((A*256)+B)/10` | °C | Provides oil temperature once the engine ECU has validated the sensor. |
| 0x138 | Calculated load | 2 | `C/2` | % | ECU reported engine load. |
| 0x139 | Front-left wheel speed | 0-1 | `((A*256)+B)/128` | km/h | Also available for RR/FR/RL using the remaining byte pairs. |
| 0x139 | Front-right wheel speed | 2-3 | `((C*256)+D)/128` | km/h | 0.0078 km/h resolution. |
| 0x139 | Rear-left wheel speed | 4-5 | `((E*256)+F)/128` | km/h | Matches dashboard wheel-speed readouts. |
| 0x139 | Rear-right wheel speed | 6-7 | `((G*256)+H)/128` | km/h | Use for driven-wheel slip calculations. |
| 0x241 | Steering angle | 0-1 | `(((A*256)+B)-32768)/10` | ° | Positive = clockwise when facing forward. |
| 0x241 | Steering rate | 2-3 | `(((C*256)+D)-32768)/10` | °/s | Signed value around zero. |
| 0x2D2 | Yaw rate | 0-1 | `(((A*256)+B)-32768)/100` | °/s | Gyro signal from VSC/ABS ECU. |
| 0x2D2 | Lateral acceleration | 2-3 | `(((C*256)+D)-32768)/256` | g | Combine with longitudinal accel for grip analysis. |
| 0x2D2 | Longitudinal acceleration | 4-5 | `(((E*256)+F)-32768)/256` | g | Positive when accelerating. |
| 0x328 | Gear position | 0 | `(A&0x0F)` | gear index | 0=Neutral, 1=1st, etc. |
| 0x345 | Fuel level | 0-1 | `((A*256)+B)/100` | % | Average from tank sender; smoothing applied. |
| 0x390 | Brake master pressure | 0-1 | `((A*256)+B)/40` | bar | Useful for pedal trace. |
| 0x3A7 | Clutch switch | 0 | `(A&0x01)` | boolean | 1 when clutch depressed. |
| 0x6E1 | Intake manifold pressure | 0-1 | `((A*256)+B)/100` | kPa | Absolute manifold pressure. |
| 0x6E2 | Fuel rail pressure | 0-1 | `((A*256)+B)/10` | bar | Direct-injection rail pressure. |

> **Tip:** When adding multiple channels from the same CAN ID, create one RaceChrono channel per signal and keep the byte ranges aligned with the table above. RaceChrono lets you duplicate a channel and adjust the equation, which speeds up data entry.

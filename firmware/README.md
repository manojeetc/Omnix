# Omnix firmware

The ESP32 drives the legs and reads the MPU. The Beetle RP2350 renders the face and dashboards from ESP32 telemetry. Each board has one complete Arduino sketch; keep the headers and extra `.ino` tabs with its main file.

| Board | Open in Arduino IDE | Board package | Build settings |
| --- | --- | --- | --- |
| ESP32 DevKit V1 | `esp-v4.0.26/hexapod_walk/hexapod_walk.ino` | esp32 by Espressif Systems, tested with 3.3.12 | ESP32 Dev Module; 240 MHz; 4 MB flash; Huge APP partition scheme; erase flash disabled |
| Beetle RP2350 | `beetle-v3.8/hexapod_face/hexapod_face.ino` | Raspberry Pi Pico/RP2040/RP2350 by Earle Philhower, tested with 6.3.0 | Generic RP2350; ARM; 150 MHz; 2 MB flash with no filesystem; no RTOS |

Install **Adafruit PWM Servo Driver Library** for the ESP32. Install **Adafruit ST7735 and ST7789 Library** and **Adafruit GFX Library** for the Beetle, including their dependencies.

## Connections

| Connection | Pins / setting |
| --- | --- |
| ESP32 to both PCA9685 logic buses | SDA D27, SCL D14; left address `0x40`, right address `0x60` |
| ESP32 to MPU | SDA D21, SCL D22; MPU VCC from ESP32 3V3; shared ground; addresses `0x68` or `0x69` detected automatically |
| ESP32 to Beetle | ESP32 D17/TX2 to Beetle pin 1/RX; shared ground; 230400 baud |
| LCD SPI to Beetle | SCK 18, MOSI 19, DC 27, CS 26; signals at 3.3 V |
| LCD power | Regulated 5 V for the installed 3.3 V–5 V compatible module; ground shared with Beetle |
| Beetle power | Robot 5 V to VCC; remove that VCC feed when connecting Beetle USB-C; BAT unused |
| Battery measurement | Battery+ → 10k resistor → D34 → 4.7k resistor → ground |
| Touch input | ESP32 D13 to the touch pad; `USE_BUTTON` selects a button to ground instead |
| Optional PCA clock signals | Left PCA channel 15 SIGNAL to D35; right channel 15 SIGNAL to D4; never connect servo V+ to these inputs |

PCA channels 0–8 drive the three legs on each side, three joints per leg. Channel 15 is reserved for clock measurement. Physical leg order is FL, ML, RL, FR, MR, RR; `swapsides` and `swapends` adapt the user-facing mapping.

The installed robot uses a separate 6 V UBEC for servo V+ and a 5 V buck for the electronics. Keep grounds connected. Pull the robot's USB/power isolation jumper J1 while the ESP32 USB cable is connected. The LCD's 5 V power connection does not change its 3.3 V SPI signal levels.

## Commands

Send newline-terminated commands at 115200 baud to the appropriate board's USB Serial Monitor. ESP32 commands also work through classic Bluetooth, named `Hexapod`. Wi-Fi is disabled.

For the setup sequence, follow the [calibration guide](../calibration.md). It covers leg mapping, servo directions, trims and angle scales, battery measurement, IMU mounting, and optional sag and foot-height checks.

For servo holds, MPU errors, jerky motion, resets or display problems, follow the [troubleshooting guide](../troubleshooting.md).

### ESP32

`help` lists the commands in the running firmware. Common commands are:

| Purpose | Commands |
| --- | --- |
| Movement | `w` forward, `s` backward, `a` / `d` strafe, `q` / `e` turn, `x` stop, `stand` |
| Stance and gait | `gait tripod`, `gait ripple`, `gait wave`, `speed 50`, `height 55`, `width 10`, `lift 25`, `period 1` |
| Joint calibration | `dir`, `dir FL`, `flip FL f`, `trim FL f 20`, `cal FL f`, `swapsides`, `swapends`, `show` |
| Calibration persistence | `save` saves the current calibration/settings; `reset` clears them |
| Direct servo modes | `c` centres all joints; `off` relaxes them |
| Load tuning | `com X Y`, `sagtest`, `sag <mm>`, `lag <ms>`, `servospeed <deg/s>` |
| MPU and leveling | `tilt`, `motion`, `levelcal`, `mpufront`, `level on`, `level off`, `mpustatus`, `mpu` |
| Walking assists | `assist`, `assist on`, `assist off`, `assist heading/terrain/landing/balance on/off`, `turn 90`, `turn -45` |
| Assist tuning | `headkp 1.8`, `turnrate 20`, `roughgain 1`, `landsoft .3`, `guarddeg 27` |
| Screen | `screen`, `screen face/dash/legs/trail/stab/heat/drive/level/sys/off`, `fps`, `scrrot`, `scrinv` |
| Tricks | `tricks`, `trick`, `party`, `fidget on/off`, or a trick name such as `wave`, `bow`, `dance` |
| Battery and idle | `bat`, `batcal 7.84`, `idle 5`, `sleep` |
| Diagnostics | `params`, `mem`, `touch`, `i2c`, `i2cpins`, `pcaclock`, `pcamon on/off`, `mathdump`, `equations`, `equations bench`, `dynamics` |

Distances are mm, speeds are mm/s, and joint calibration angles are degrees. `dir` and servo scale calibration move joints, so support the robot during these checks. `levelcal` needs a settled robot on a flat surface; `mpufront` uses a front-up pose to determine the mounting direction. Saved calibration lives in that ESP32's NVS, separately from these source files.

While stopped in the normal stance, gyro auto-zero uses at least 75 valid samples over 1.5 seconds. It tolerates short read gaps, requires at least 60% usable reads and resets after a gap over 80 ms. Motion, vibration, acceleration and active foot-calibration tests prevent bias updates. `mpustatus` shows the quiet window and re-zero count; repeated transfer failures still need investigation.

Battery thresholds for the 2S pack are 6.9 V to stand at startup, 7.0 V for a warning and 6.6 V for lowering/relaxing and locking. Idle parking defaults to five minutes. A failed PCA update latches a servo hold; restoring communication does not automatically resume the previous movement.

### Beetle

Send these to the **Beetle** USB port:

- `anim calm`, `anim normal`, `anim lively`: animation intensity.
- `idle next`: advance the idle routine.
- `demo`, `demo idle`, `live`: demonstration or real robot telemetry.
- `info`, `help`: status and available commands.
- `single`, `dual`, `blocking`, `dma`, `hold`, `repeat`: rendering/transfer diagnostics.

`hold` freezes the displayed picture without writing further frames. `repeat` sends the frozen picture repeatedly. The FPS overlay reports each core's render time in milliseconds; the SPI MHz value is the display bus speed.

## Code layout

| ESP32 files | Responsibility |
| --- | --- |
| `hexapod_walk.ino` | Board setup, calibration/settings, servo output, gait loop and command dispatch |
| `gait_math.h`, `mpu_math.h`, `walk_control.h`, `loop_timing.h` | Geometry, contact/load estimates, attitude, walking assists and timing helpers |
| `walkctl.ino` | Connect walking assists to the robot state |
| `trick_poses.h`, `tricks.ino` | Physical poses, playback and returns to stance |
| `pca_monitor.h`, `pcamon.ino` | Optional servo-board clock diagnostics |
| `screen.ino`, `math_export.ino` | Display telemetry and offline configuration export |
| `equations_generated.h`, `model_generated.h` | Generated general equations and CAD mass/inertia estimates |
| `equations.h`, `equation_adapter.h`, `equations.ino` | Checked SI wrappers, firmware unit conversion and equation diagnostics |
| `dynamics_control.h`, `dynamics.ino` | Experimental demand recording and optional gait-clock shaping |

| Beetle files | Responsibility |
| --- | --- |
| `hexapod_face.ino` | UART input, frame scheduling, two-core rendering and SPI DMA |
| `state.h`, `drive_state.h` | Main and walking-assist telemetry parsing |
| `face.h`, `personality.h` | Eye rendering, particles, idle routines and event reactions |
| `gfx.h`, `fonts.h` | RGB565 drawing primitives and generated font data |
| `nav.h`, `pages.h`, `navpages.h`, `drive_page.h` | Page state, transitions and dashboards |

Motion math uses body axes x forward, y left, z up. Firmware geometry uses mm; generated dynamics uses metres, radians, kg and N m. Convert at `equation_adapter.h`, once. The CAD model is an estimated mass distribution scaled to total mass. Dynamics starts in monitor mode; recorded demand is advisory and does not provide measured servo torque.

Core 0 owns Beetle animation updates. Both cores render the same state into disjoint row bands, and SPI DMA retains its buffer until the final word has left the bus. ESP32 workspace searches run on a worker core and publish completed snapshots back to the control loop.

## Formatting and licensing

`.clang-format` defines the shared two-space C++ style. Keep include order, Arduino tab names and wire-protocol field order stable. Regenerate equation/model/font data through their respective tools when changing the generated content; formatting those files does not change their numeric data.

The repository's [GPL-3.0 license](../LICENSE) applies to its code. The font attribution in `fonts.h` identifies Rajdhani and Inter as SIL Open Font License 1.1 assets.

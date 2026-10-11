# Omnix troubleshooting

This guide covers the ESP32 v4.0.26 and Beetle v3.8 sketches in this repository. Check the [assembly guide](assembly.md) for mechanical fit and build order, the [firmware guide](firmware/README.md) for build settings and wiring, and the [calibration guide](calibration.md) for the setup sequence.

## Start here

Send `x` to the **ESP32**, then wait for the feet to land and the body to settle. Support the body before using `off`, which relaxes the servos. Switch power off before changing wiring.

Record the problem before restarting or clearing anything. On the ESP32, collect:

```text
params
mem
mpustatus
servostatus
bat
```

USB Serial Monitor uses **115200 baud** and newline-terminated commands. Robot commands go to the ESP32. Display diagnostics such as `info` and `hold` go to the Beetle's USB port.

| Symptom | Start with |
| --- | --- |
| Build or upload fails | [Build and upload](#build-and-upload) |
| Commands do nothing | [Serial and Bluetooth](#serial-and-bluetooth) |
| Wrong leg or joint moves | [Servo mapping and calibration](#servo-mapping-and-calibration) |
| Robot freezes after a PCA error | [Servo output hold](#servo-output-hold) |
| Walking locked or almost no stride | [Walking restrictions](#walking-restrictions) |
| Jerking or pauses during walking | [Jerky movement](#jerky-movement) |
| MPU missing, stale or failing reads | [MPU communication](#mpu-communication) |
| Drift, wobble or rejected calibration | [Tilt and calibration](#tilt-and-calibration) |
| Turn stops early | [Turns and balance recovery](#turns-and-balance-recovery) |
| ESP32 restarts | [Unexpected resets](#unexpected-resets) |
| Screen dims, freezes or looks wrong | [Display problems](#display-problems) |
| Screen says waiting for robot | [ESP32 to Beetle link](#esp32-to-beetle-link) |
| Saved settings seem missing | [Settings and persistence](#settings-and-persistence) |

## Build and upload

- Open the main `.ino` inside its complete sketch folder. Keep the other `.ino` tabs and headers alongside it. A missing project header usually means the folder is incomplete.
- Install the board packages and libraries listed in the [firmware guide](firmware/README.md). Its recorded package versions are the versions used for build checks.
- For the ESP32, use **ESP32 Dev Module**, 240 MHz, 4 MB flash and **Huge APP**. If the sketch exceeds the selected application partition, check that partition setting. Keep erase flash disabled to retain settings.
- For the Beetle, use **Generic RP2350**, ARM, 150 MHz, 2 MB flash with no filesystem and no RTOS. RP2350 hardware headers require the specified Philhower board package.
- Check the selected USB device and port before uploading. Close other applications using that port if it is busy.

The sketch folders are the version reference for this guide. The current ESP32 source still prints `v4.0.25` in its startup banner, even though it is stored under `esp-v4.0.26`. Include the uploaded folder name in reports.

## Serial and Bluetooth

**Unreadable output:** select 115200 baud on ESP32 Serial Monitor. **No response:** select a newline ending, send `help`, and confirm you are talking to the intended board. The ESP32 and Beetle have different command sets.

The ESP32 advertises classic Bluetooth as `Hexapod`. Use `bt` over USB to inspect its status. If pairing is stuck, `btforget` clears the ESP32's stored bonds; also remove the old `Hexapod` pairing on the PC or phone, then pair again. This changes Bluetooth pairing, not joint calibration.

If commands fail only while moving, stop and inspect `mem`. Capture any startup, crash or watchdog messages. A blocked control loop and a lost Bluetooth connection need different fixes.

## Servo mapping and calibration

Support the body with the legs free and send `dir FL`. Confirm both the physical leg and each printed movement prompt. Use the [direction checks](calibration.md#3-check-servo-directions) for all six legs.

- A single joint moving backward calls for `flip <leg> <c/f/t>`, a repeated direction test and `save`.
- Reversed side mapping calls for checking which PCA board drives each side. `swapsides` changes that mapping and saves immediately.
- `swapends` changes the selected front and the leg names. Recheck mapping and IMU orientation after changing the front.
- A joint consistently offset from neutral needs a horn-position or trim check. `trim FL f 10` **adds 10 microseconds** to its existing trim.
- A joint that reaches zero correctly but misses the positive or negative angle marks needs the three-point `cal` procedure.

`c` uses existing trims. `c90` sends 1500 microseconds with trims ignored. Use the calibration guide before reseating horns or changing scales.

If servos do not move at all, check both kinds of PCA power: logic VCC comes from 3V3, while servo V+ comes from the separate UBEC. An I2C response confirms logic communication, not servo-power delivery. Check `servostatus` for a latched hold as well.

## Servo output hold

The message `Output HOLD latched` means a PCA update or configuration check failed. The firmware freezes gait and leveling and keeps retrying the held outputs. A returning board does not resume the old movement.

1. Support the robot, stop movement and collect `servostatus`.
2. Check the PCA bus: SDA D27, SCL D14, left address `0x40`, right address `0x60`, and shared ground. Check logic and servo power separately.
3. With the robot stopped, use `i2c` to check addresses. Both servo-board addresses should appear on the PCA bus.
4. After correcting the problem, wait until `servostatus` reports `hold release ready yes`. Both boards must accept complete updates for one second.
5. Set the robot upright and still. If the MPU is calibrated, recovery also requires fresh data and acceptable tilt/rates. Send `servorecover` and wait for any raised feet to settle before issuing a new movement command.

PCA error **6** means a short register read; **7** means the configuration check differed from the expected setup. These are application diagnostics, separate from the Wire error codes below. Capture the configuration and fault snapshot instead of repeatedly trying to release an unresolved hold.

## Walking restrictions

**`Walking locked`:** directions/calibration have not been saved. Complete the [servo checks](calibration.md#3-check-servo-directions) and use `save`. Saving marks calibration complete; it does not physically verify the joints.

**`Almost no safe stride`:** `params` reports a stride below 10 mm. The workspace checks include stance height/width, joint calibration limits, lift clearance and modeled load. Stop, record `params`, `show` and `mathdump`, and review the last setting changed.

Adjust one setting at a time and inspect the result. A reduced lift or a different height/width can change the available workspace, but there is no single stance that fits every calibration. If the restriction followed sag or foot-height tuning, check those measurements in the [calibration guide](calibration.md#7-optional-load-and-foot-height-tuning). `sag 0` disables sag compensation and `zfix off` disables foot-height correction for a comparison; record the old settings before changing them.

Use measured `mass` and a defensible `stall` estimate. Increasing configured stall torque only relaxes a model limit. It does not increase servo strength. `torque`, `loads` and `heat` are estimates, not readings from torque or temperature sensors.

Battery lock, servo hold, centering and balance recovery can also prevent movement. Read the specific message rather than clearing calibration with `reset`.

## Jerky movement

Compare timing over the interval when the jerking actually happens. While stopped, send:

```text
mem reset
mem
mpustatus
```

Perform a short walk at a low speed, then send `x`, wait until settled and collect the same status commands plus `servostatus`. Keep the ESP32 running between the before/after reports.

| Observation | Next check |
| --- | --- |
| Loop gaps or ticks over 30 ms increase | Compare the `mem` stage peaks, especially MPU, input and feet/PCA time |
| Failed MPU frames/transfers increase | Follow the MPU communication checks |
| PCA failed writes or HOLD appears | Follow the servo output hold procedure |
| IK misses or slew-guard hits increase | Review stance, lift, joint calibration and commanded speed |
| Motion is smooth unloaded but jerks under weight | Inspect binding, horn security, payload, servo-power wiring and loaded servo response |
| One joint jerks even unloaded | Check that joint's connector, horn, gears and calibration |

A single worst-gap number does not establish the cause. Look for changes that coincide with the symptom. `mem reset` clears timing measurements, not calibration or accumulated fault counters.

If leveling seems involved, compare with `level off` after its correction has faded. If experimental dynamics shaping was enabled, `dynamics monitor` returns it to recording without shaping. Keep each comparison short and change one feature at a time. Record the outcome before saving any new settings.

## MPU communication

### Check the correct bus

The MPU uses **SDA D21 and SCL D22**, separate from the PCA bus. It needs VCC from ESP32 3V3 and a reliable shared ground. `i2c` should find it at `0x68` or `0x69` on the MPU bus. A successful address scan does not establish that configuration writes or full sample reads work.

With power off, check connector contact, solder joints, rail continuity and any split sections of a breadboard rail. If a rail is suspect, a direct MPU VCC/GND connection to the ESP32 is a useful comparison. In the reference robot, direct wiring identified a troublesome ground path; the 3V3 rail later worked with the ground corrected. That observation does not identify the cause on every build.

The MPU's red LED indicates that part of the board is powered. It does not prove that the chip supply, ground reference or I2C connection is healthy. Likewise, `SDA=1 SCL=1` only reports the line levels at that moment.

### Read the error and counters together

| Report | Meaning |
| --- | --- |
| Wire code 2 | NACK |
| Wire code 4 | General driver error |
| Wire code 5 | Timeout |
| Native probe `ESP_ERR_NOT_FOUND` (261) | The probe did not receive an address acknowledgement |
| Combined read `0/14 bytes` | No complete sensor sample was received |
| `failed read transfers` | Individual failed transfer attempts, including attempts later recovered by retry |
| `failed sample frames` | A complete sample was still unavailable after the retry/budget |
| `dropouts` | The firmware declared sustained sensor loss; zero dropouts can coexist with many failed reads |

For the ESP-IDF driver used with the checked ESP32 package, a synchronous transfer whose status is not complete can return `ESP_ERR_INVALID_STATE` (259). That code alone does not identify the electrical or timing cause. See [Espressif's I2C driver implementation](https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_driver_i2c/i2c_master.c#L683-L687).

In this firmware, 50 consecutive failed sample ticks declare the MPU lost and pause leveling. Recovery retries every three seconds. `MPU: reading` can still appear between intermittent failures, so compare counters and the age of the last failure across reports.

After correcting connections, hold the stopped robot still and send `mpu` to restart the sensor and remeasure its gyro bias. Check `mpustatus` again. If failures persist, `mpulog on` enables driver error output; collect a short log and use `mpulog off` afterward.

### MPU6050 label, MPU6500 identity

`WHO_AM_I` is a device-identity register, separate from the bus address. The documented values are `0x68` for MPU6050 and `0x70` for MPU6500. An MPU6500 can still answer at I2C address `0x68` or `0x69`. See the [MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf) and [MPU6500 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf).

The reference module was sold as MPU6050 but reports `0x70`; the current firmware recognizes that family and configures it accordingly. Identity does not establish authenticity or connection quality. A `0xFF` result during read failures is not a reliable chip identification.

## Tilt and calibration

**Readings drift while still:** check whether the trend continues over time and whether the gyro re-zero counter increases. A 0.1-degree change on its own does not establish a fault. While stopped in the normal stance, the current quiet window needs at least 75 valid samples over 1.5 seconds, at least 60% usable reads and no gap over 80 ms. Motion, vibration and noisy acceleration can reject a window. `mpustatus` shows its state.

**Leveling wobbles or corrects the wrong way:** send `level off`, wait for corrections to fade, then verify `tilt`: front up should give positive pitch and left up should give positive roll. Recheck mounting calibration and joint directions before enabling leveling again. A loose IMU mount or inconsistent foot contact also deserves inspection.

**`levelcal` or `mpufront` rejected:** stop with `x`, wait for all motion to finish, use `level off`, and wait for the correction to fade. Hold the required pose steadily throughout collection. The rejection message reports usable reads, acceleration norm/noise and gyro peak; use those values to distinguish missing data from motion. If the gyro reports motion while the robot is certainly still, run `mpu` while holding still, then retry.

`levelcal` needs a level body and clears the old front calibration. Follow it with `mpufront` while the selected front is raised about 20 degrees. `Not tilted enough` calls for a larger, steady front-up pose. These commands save accepted mounting data immediately. See the [full mounting procedure](calibration.md#6-calibrate-the-imu-mounting).

## Turns and balance recovery

**`Turn stopped: MPU stale/lost`:** the turn requires continuous, recent body samples. A missing frame or invalid sample interval can cancel it even when a later `mpustatus` says `reading`. Collect the turn's sensor-cause and transfer-at-cancellation lines, then follow the MPU and timing checks. Missing sensor motion is not integrated into the measured turn angle.

Turns use relative gyro heading. Their measured angle is not a compass bearing or a position measurement. They also need a calibrated robot, usable stride and speed, healthy boards/battery, and a settled stance before starting.

**Balance hold/recovery:** set the robot upright, let raised feet finish landing, and wait for at least one second of stable MPU data. Issue a new movement command after the hold clears. Recovery does not replay the old command.

**Stumble message:** the firmware reports a temporary slowdown based on IMU motion. If it appears beside a PCA error, also investigate that transfer failure; the adjacent messages do not establish that stumble detection caused the bus fault.

## Unexpected resets

Capture the first startup/reset message and any `Brownout`, `Guru Meditation`, panic or watchdog output. Send `mem` afterward to read the recorded restart reason and uptime.

| Report | Next step |
| --- | --- |
| Brownout | Check electronics power at the boards, supply connections, shared ground and the documented USB isolation |
| Panic/crash | Save the full crash text and the uploaded sketch/package versions |
| Watchdog | Save the log and compare recent timing/stage measurements |
| Power on | Record the circumstances; this alone does not identify why the previous run ended |
| Woke from deep sleep | Check the preceding idle/battery messages |

Opening a serial port can affect reset lines on some ESP32 boards and terminal setups, so a restart caused by reconnecting the monitor can replace evidence of the earlier event. [Espressif documents the DTR/RTS reset behavior](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html#automatic-bootloader). Record whether you reopened the monitor.

The reference robot uses a separate 6 V servo UBEC and 5 V electronics buck. Check voltage at the ESP32/Beetle connections during the symptom, rather than relying only on the converter's advertised rating or a battery reading. Do not bypass a battery lock to continue a test. Correct its supply or battery measurement using the [battery calibration steps](calibration.md#5-calibrate-the-battery-reading).

## Display problems

### Dim pulses or brief darkening

First distinguish a dimming picture/backlight from corrupted pixels or a replayed boot animation. On the **Beetle USB port**, send `hold` and confirm `DIAG mode=hold`; the picture should freeze and no further frames are sent. Observe the stationary robot first, then a short movement if its walking is otherwise healthy.

If the frozen picture still dims, that result points toward the LCD power/backlight/display path. In the reference robot, powering the installed **3.3 V to 5 V compatible LCD module** from regulated **5 V** resolved the dim pulses. This applies to that module's VCC input, while SPI signals remain 3.3 V. Use the power requirements of the actual module fitted.

Keep Beetle 3V3 separate from the ESP32 3V3 rail and check shared ground. Removing only Beetle VCC does not isolate it if another power connection remains. Follow the [power wiring guide](firmware/README.md#connections) before testing USB power. Send `live` to resume display rendering after `hold`.

### Corrupted pixels, bands or unstable transfers

On the **ESP32**, send `scrspeed 25` to compare at a lower requested SPI speed. It saves immediately. Use `fps` to check the displayed actual bus speed, since the Beetle rounds to a supported clock. Record the previous setting before changing it.

Inspect SCK 18, MOSI 19, DC 27 and CS 26 on the Beetle, along with power and ground. On **Beetle USB**, compare `single` with `dual`, or `blocking` with `dma`, one change at a time. Record which combination reproduces the problem. Restore `dual`, `dma` and `live` after the comparison.

`repeat` repeatedly transfers a frozen frame, while `hold` sends no new frames. Differences between those tests can help isolate transfer-related symptoms. Repeated `HEXAPOD` boot animation or a Beetle uptime restart points to a board reset instead; collect `info` and its USB startup log.

### Orientation, colors and timings

ESP32 commands `scrrot` and `scrinv` toggle the saved rotation and color inversion. `scrtest` advances the display color test. Use `screen face` to leave the screen-off page.

The `fps` overlay's core figures are render times in **milliseconds**, not CPU-use percentages. Its MHz value is the SPI clock, not frames per second. A value near 10 ms for a core does not mean it is using only 10% of that core.

## ESP32 to Beetle link

The status connection is **ESP32 D17/TX2 to Beetle pin 1/RX**, with a shared ground, at **230400 baud**. Its baud rate differs from USB Serial Monitor. The connection carries telemetry from the ESP32 to the Beetle.

Send `live` on Beetle USB to leave demo/hold modes. Check `info` for `rx_age`, bad lines and uptime, and use `screen sys` or the `fps` overlay from the ESP32 to inspect link activity. If it still says waiting for robot, verify the UART connection and upload the matching sketch pair.

Status transmission is scheduled about every 33 ms, so the displayed link rate is typically around 30 accepted status lines per second. It is separate from display frame rate. A brief dip during a long diagnostic command does not establish a connection fault. A sustained drop while centering or walking calls for checking ESP32 `mem` stage times, MPU errors, resets and the UART wiring. A rising `status lines dropped` count in `mem` means the ESP32 discarded an oversized status message.

## Settings and persistence

`save` stores joint calibration and the documented tuning settings in that ESP32's NVS. See [what gets saved](calibration.md#what-gets-saved) for commands that save immediately.

An EN-button restart retains settings. `reset` clears them. Another ESP32 has separate storage; uploading the same sketch does not transfer calibration. Erasing flash or changing the flash layout can affect stored data, so keep the documented board/partition settings for ordinary updates.

Use `show`, `params` and `mathdump` to inspect the current configuration before changing it. `mathdump` is an analysis export and does not contain a full NVS backup. If only screen diagnostic mode or dynamics mode changed after a restart, those runtime choices are separate from servo calibration.

## Details to include in a bug report

- ESP32 and Beetle sketch folder names, board selections and board-package versions.
- The command that triggered the issue, plus whether leveling, assists or dynamics shaping were enabled.
- Whether the robot was standing, walking or supported with its feet unloaded, and any recent payload/wiring changes.
- ESP32 `show`, `params`, `mem`, `mpustatus` and `servostatus` output from the same run, collected after stopping without a restart.
- For display issues, Beetle `info` output, whether `hold` still dims, and whether its boot animation repeats.
- The complete error/reset text and a clear photo of relevant connections when wiring is involved.

Return to the [project README](README.md), [assembly guide](assembly.md), [calibration guide](calibration.md) or [firmware guide](firmware/README.md).

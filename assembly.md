# Omnix assembly

This guide takes Omnix from printed parts to a robot ready for calibration. Read it once before you start: some screws get covered by later parts, so the order matters.

- Wiring pictures: [`wiring-diagrams/`](wiring-diagrams/)
- Pins and build settings: [`firmware/README.md`](firmware/README.md)
- After assembly: [`calibration.md`](calibration.md), then [`troubleshooting.md`](troubleshooting.md) if something is off

> **Directions used here.** "Front" is the end the firmware walks toward by default, and "left" / "right" are seen from behind the robot looking forward. The screen face sits at the **rear** end. You can flip which end counts as the front later with the `swapends` command (see [`calibration.md`](calibration.md)).

---

## 1. Print the parts

All files are in [`cad/`](cad/). PLA is fine for everything except the foot caps.

| Part | Qty | Notes |
|---|---|---|
| `chassis_V4.step` | 1 | flat, either face down |
| `top_deck_V3.step` | 1 | flat face down, no supports |
| `bottom_plate_V4.step` | 1 | top face down, battery rails up, no supports |
| `coxa_bracket_V5.step` | 6 | |
| `femur_bracket_V6.step` | 6 | |
| `tibia_V5.step` | 6 | |
| `hip_inner_link.step` | 6 | |
| `knee_inner_link.step` | 6 | |
| `servo_cap.step` | 12 | one on each femur and knee servo |
| `foot_cap_TPU.step` | 6 | **TPU** |
| `standoff_M3x25.step` | 4 | chassis to top deck |
| `standoff_M3x14.step` | 4 | chassis to bottom plate |
| `screen_base.step` | 1 | on its side as exported, no supports |
| `screen_frame_V4.step` | 1 | face down as exported, no supports |
| `head_hood_v2.step` | 1 | skirt down, tree supports **inside only** (under the roof, the sloped face and the ESP32 rails) |
| `head_front_clip.step` | 1 | on its side, no supports, 100 % infill (PETG if you have it) |
| `head_visor_v2.step` | 1 | optional orange accent, print flat, glue around the window |

Suggested settings: 0.2 mm layers, 3 to 4 walls, 15 to 30 % infill (30 % and 4 walls for the screen stand). The printed standoffs have 2.6 mm holes so M3 screws self-tap into both ends. Export the STEP files to STL or 3MF at their original size in millimetres, and check the orientation and supports in your slicer before printing.

## 2. Gather the parts

**Electronics** (full list with links in [`misc/BOM.xlsx`](misc/BOM.xlsx))

- ESP32 DevKit V1 (ESP-WROOM-32, 30 pin)
- 18 × MG90S servos (metal gear)
- 2 × PCA9685 16-channel servo boards
- MPU6050 / MPU6500 IMU board (GY-521 style)
- DFRobot Beetle RP2350
- 2.0" ST7789 240 × 320 SPI display
- 2S LiPo (about 2200 mAh, fits 86.5 × 34 × 18 mm rails)
- 6 V UBEC (servos) and 5 V buck converter (electronics)
- 2 × 2200 µF 16 V electrolytic capacitors
- Inline fuse, KCD1 round rocker switch (20 mm snap-in), 400-point breadboard
- 10 kΩ and 4.7 kΩ resistors (battery monitor)

**Hardware**

| Qty | Screw | Where |
|---|---|---|
| 4 | M3 × 8 flat-head | top deck into the 25 mm standoffs |
| 4 | M3 × 8 | chassis into the 25 mm standoffs (from below) |
| 8 | M3 × 8 | 14 mm standoffs: chassis from the top, bottom plate from below |
| 12 | M2.5 × 10 to 12 self-tapping | coxa servo tabs into the chassis (drill the servo tab holes to 2.5 mm) |
| 8 | M2.5 × 6 to 8 self-tapping | PCA9685 boards onto the deck posts |
| 12 | M3 × 8 to 10 self-tapping | inner links into their posts (from outside the femur / tibia) |
| 36 | M2 × 6 to 8 | horn screws, 2 per horn (bracket holes are 2.2 mm) |
| 2 + 2 | M3 × 6 + M3 nuts | screen base under the deck |
| 2 | M3 × 16 flat-head | through the hood face into the screen frame (replaces the frame's 2 × M3 × 12) |

Also: the servo tab screws from the servo bags, a 20 mm velcro strap for the battery, double-sided foam tape, a drop of CA glue, and 2 short pieces of 1.75 mm filament (ESP32 retaining pins).

These screw lengths are for the listed parts. Check engagement and clearance as you assemble, especially where a screw ends near a circuit board. Servo cases, horns and display boards can vary between suppliers.

## 3. Centre every servo first

Before fitting a horn, drive the servo to **1500 µs**. A servo tester set to that pulse width works, or use the ESP32 and PCA boards:

1. Upload the ESP32 sketch using the [firmware guide](firmware/README.md). Leave servo power disconnected during upload and follow the guide's USB power-isolation instructions.
2. Connect the PCA logic bus and servos to channels 0 through 8 on each board as shown in [section 7](#7-wiring). Power servo V+ from the 6 V UBEC, with its ground connected to the ESP32 ground. Keep the servos free to move and leave the horns off.
3. Open the **ESP32** Serial Monitor at **115200 baud** with a newline ending. Send these commands one at a time:

   ```text
   x
   level off
   assist off
   fidget off
   c90
   ```

4. Wait for `Every servo at exactly 1500 us`. Switch servo power off before fitting horns, and avoid turning the splines while fitting them.

The firmware briefly centers joints with the saved trims at startup, then moves toward its standing pose. Use `c90` explicitly for horn fitting: it ignores trims. Repeat it after a restart or another movement command before fitting more horns. Mark each horn and spline position so they can be paired again later.

With all servos at 1500 µs the robot should stand like this:

- **coxa** brackets point straight out from the body (45° at the corners, 90° in the middle)
- **femur** joint-center lines horizontal
- **tibia** joint-center lines pointing down, perpendicular to the femurs

Small errors are fine; calibration trims them out. Aim for within one spline tooth.

## 4. Body

1. **Coxa servos into the chassis.** 6 servos, shafts up, 2 × M2.5 tab screws each. One tab screw always ends up under the coxa bracket, so fit the servos *before* the brackets (or pull a bracket straight up off its spline to reach it).
2. **25 mm standoffs** onto the chassis, screws from below. Do this *before* taping the UBEC under the chassis: it covers one of these screws.
3. **14 mm standoffs** onto the underside of the chassis (screws from the top), then the **bottom plate** (screws from below).
4. **UBEC and buck converter** under the chassis with foam tape.
5. **Battery** into the bottom plate rails, velcro strap through the two slots.
6. **Top deck** onto the 25 mm standoffs with the 4 flat-head screws. Do this *before* the breadboard goes on: it covers the screw heads.
7. **PCA9685 boards** onto their deck posts (4 screws each), then stick the **breadboard** into its pocket.

## 5. Legs (× 6)

Build all six the same way. The legs are rotationally symmetric (the same parts on both sides), so there are no left and right versions.

1. **Coxa bracket** onto a coxa servo horn (2 × M2 screws into the horn), then press it onto the centred coxa servo, pointing straight out. The coxa horn has **no centre screw**; it just presses on.
2. **Femur servo** into the coxa bracket with its tab screws, **servo cap** on.
3. **Femur bracket** onto the femur servo horn (2 × M2), horizontal, then the **hip inner link** on the other side: it pivots on the servo cap. Screw the link into its post from outside (M3 self-tapping).
4. **Knee servo** into the femur bracket, **servo cap** on.
5. **Tibia** onto the knee horn (2 × M2), vertical, then the **knee inner link**, screwed in from outside.
6. **Foot cap** (TPU) pressed onto the tibia's foot post.

With servo power disconnected, check joint clearance and cable routing gently. Do not force the gears or push a joint against its stop.

## 6. Screen stand

1. Drop the 2 M3 nuts into the hex pockets on top of the base's jaw.
2. Slide the jaw under the rear PCA9685 (between the PCA and the deck) until the base's neck touches the deck's rear edge.
3. From under the deck, 2 × M3 × 6 up through the deck holes into the nuts. Use 6 mm: 8 mm pokes into the PCA.
4. Slide the screen into the frame from the top edge, display facing out, until it reaches the bottom. Plug in its cable (it exits through the open back). A dot of hot glue at the top keeps the screen in.
5. Set the frame on: its tab rests on the slanted top of the neck and the two legs stand on the deck. Leave the 2 screws out for now: the hood's M3 × 16 screws go in their place.

## 7. Wiring

Wire with the battery **unplugged** and USB cables disconnected. The diagrams are in [`wiring-diagrams/`](wiring-diagrams/), and the [firmware guide](firmware/README.md#connections) has the full pin table. Where a drawing differs from this list, follow this list (it matches the installed build).

### Power ([`esp-power-wiring.png`](wiring-diagrams/esp-power-wiring.png), [`pca-power-wiring.png`](wiring-diagrams/pca-power-wiring.png))

- Battery + → inline fuse → **rocker switch** → UBEC input and buck input. Battery − → UBEC and buck GND.
- **UBEC 6 V → V+ screw terminal on BOTH PCA9685 boards** (and their GND).
- One **2200 µF capacitor across V+ and GND on each PCA** (watch the polarity).
- **Buck 5 V → ESP32 VIN, Beetle VCC and the display's V pin.**
- Never connect the servo V+ rail to the ESP32, the Beetle or the display.
- All grounds connect together: battery, UBEC, buck, ESP32, both PCAs, Beetle, MPU, display.
- Run servo power and its return through suitable wires to the PCA screw terminals, rather than through the breadboard rail. The breadboard is for electronics and signal connections.
- When programming the ESP32 over USB, disconnect its external VIN feed or remove the build's USB/power isolation jumper J1. When programming the Beetle over USB-C, disconnect its external VCC feed.

### PCA9685 to ESP32 ([`pca-esp-wiring.png`](wiring-diagrams/pca-esp-wiring.png))

| PCA pin | ESP32 |
|---|---|
| VCC (logic) | 3V3 (not 5 V: the boards pull SDA / SCL up to VCC) |
| GND | GND |
| SDA | D27 |
| SCL | D14 |
| OE | not connected |

- The second board chains off the first one's header (same SDA, SCL, VCC, GND).
- **Addresses:** left board (FL, ML, RL) no jumpers = **0x40**; right board (FR, MR, RR) **A5 bridged = 0x60**.
- **Optional clock-measurement wires:** left board channel 15 **signal pin only** → D35, right board channel 15 signal pin → D4. These support the PCA clock diagnostics; channel 15 is not a servo channel.

### Servo channels

| Channel | Servo |
|---|---|
| 0, 1, 2 | front leg: coxa, femur, tibia |
| 3, 4, 5 | middle leg: coxa, femur, tibia |
| 6, 7, 8 | rear leg: coxa, femur, tibia |

This is the default mapping, before `swapsides` or `swapends`. Confirm it with `show` and `dir` during calibration. Brown / black servo wire goes to the pin marked GND; follow the board's labels rather than assuming every board has the same connector orientation.

### IMU ([`mpu-wiring.png`](wiring-diagrams/mpu-wiring.png))

| MPU pin | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SCL | D22 |
| SDA | D21 |

Leave INT, XDA and XCL unconnected. A GY-521 with AD0 held low uses address 0x68; AD0 high selects 0x69. The firmware probes both addresses and also tries reversed SDA / SCL pins at startup if the normal connection fails. Wire it according to the table. Mount the MPU securely and flat on the body so it cannot move relative to the robot; its axes are set during calibration.

### Display and Beetle ([`screen-wiring.png`](wiring-diagrams/screen-wiring.png))

| Display pin | Connects to |
|---|---|
| V | **5 V** from the buck for the installed 3.3 V–5 V compatible display |
| G | GND |
| SCL (SPI clock) | Beetle 18 |
| SDA (SPI data / MOSI) | Beetle 19 |
| DC | Beetle 27 |
| CS | Beetle 26 |

| Beetle pin | Connects to |
|---|---|
| 1 (RX) | ESP32 TX2 (D17) |
| GND | GND |
| VCC | 5 V from the buck |
| BAT | empty |

The installed display stopped dimming when its power input was moved to regulated 5 V. Check the input rating if using a different display module. The SPI signals remain at 3.3 V.

Keep the Beetle's 3V3 separate from the ESP32's 3V3. **Unplug the Beetle's VCC wire whenever its USB-C cable is plugged in.** Grounds stay shared during normal operation.

### Battery monitor ([`battery-monitor-wiring.png`](wiring-diagrams/battery-monitor-wiring.png))

Switched battery + → **10 kΩ** → **D34** → **4.7 kΩ** → GND. This scales a full 2S pack (8.4 V) down to about 2.7 V. Tap it after the switch so it doesn't drain the battery when the robot is off. Never connect the battery straight to D34. Fine-tune the reading with `batcal` during calibration.

### Optional

- Touch pad or button: D13 (`USE_BUTTON` in the sketch selects which).

## 8. Head

1. **Front clip:** slide its tall flat end **up** into the slot of the block inside the hood's front wall, hook lip pointing toward the face, pull tab outside. A dab of CA glue holds it.
2. **Power switch:** snap the KCD1 rocker into the round plinth on the roof. Use push-on connectors so the hood can come off.
3. **ESP32:** slide it **up** into the 2 rails under the roof, metal module and buttons toward the face, pins and jumpers toward the front, USB end on the side with the hole in the wall. Push a filament piece through the small hole under each end of the board so it can't drop. Plug the jumpers back on.
4. With power disconnected, position any capacitor taller than about 45 mm above the deck so it clears the hood. Keep its leads insulated and avoid straining them.
5. **Lower the hood front first:** the clip snaps under the front edge of the deck (servo cables pass either side of it). The face then rests on the screen frame. Fix it with 2 × M3 × 16 flat-head through the face into the frame tab and the neck.
6. Optional: glue the visor around the window.

**To remove the hood:** take out the 2 screws, pull the front up firmly so the clip lets go, and lift. The ESP32's USB port is reachable through the side hole with the hood on.

## 9. Final checks and calibration

- [ ] No bare wire can touch another (especially V+ near the ESP32)
- [ ] Capacitors the right way round
- [ ] UBEC output measured at 6 V and buck output at 5 V before connecting the boards
- [ ] PCA VCC on 3V3, PCA V+ on the UBEC
- [ ] Right PCA has A5 bridged
- [ ] Every servo plug the right way round (brown / black = GND)
- [ ] MPU secured to the body, with a reliable ESP32 ground connection
- [ ] No wires trapped under the hood or pulled tight by a joint
- [ ] USB power isolation checked for whichever board is being programmed
- [ ] The body is **firmly supported** with all feet clear of the table

Upload either sketch that has not already been installed using the [firmware guide](firmware/README.md). The ESP32 can move the joints after a restart, so keep the body supported. Open its Serial Monitor at **115200 baud** and follow the [calibration guide](calibration.md): leg mapping (`dir`), directions (`flip`), neutral trims (`trim`), angle scales (`cal`), battery voltage (`batcal`) and IMU orientation (`levelcal`, `mpufront`). Finish those checks before walking.

If a board does not respond or a joint binds, stop and use the [troubleshooting guide](troubleshooting.md).

Return to the [project README](README.md).

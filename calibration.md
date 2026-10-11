# Omnix calibration

These steps apply to the [ESP32 v4.0.26 sketch](firmware/esp-v4.0.26/hexapod_walk/hexapod_walk.ino). Send commands to the **ESP32**, using USB Serial Monitor at **115200 baud** with a newline ending, or its `Hexapod` Bluetooth connection. The Beetle displays telemetry and does not store joint calibration.

Send one command at a time and wait for each movement or measurement to finish. Numeric examples below are examples, so replace measurements with your own values.

For a new build, follow the [assembly guide](assembly.md), including servo centering before fitting horns, then complete sections 1 through 6 here before walking. Section 7 covers optional load and foot-height tuning. For an existing calibrated robot, use `show` first and repeat only the steps affected by your changes.

## 1. Prepare the robot

Check the [wiring and power connections](firmware/README.md#connections), including the USB power-isolation instructions. Support the body firmly with all six legs clear of the table for servo calibration. Keep the joints free to move.

Send:

```text
x
level off
assist off
fidget off
show
```

Wait for any trick, foot movement or leveling correction to finish. `x` stops movement while keeping servo output active. If a joint binds, support the body and send `off` to relax the servos.

Have a ruler or angle card ready. A phone level can help with femur and tibia angles if its reference is aligned with the link's joint centers.

## 2. Check leg names and horn centers

Leg names are relative to the robot's selected front. Left and right are the robot's own sides.

| Name | Leg |
| --- | --- |
| FL | Front left |
| ML | Middle left |
| RL | Rear left |
| FR | Front right |
| MR | Middle right |
| RR | Rear right |

Joint names are `c` for coxa, `f` for femur and `t` for tibia.

Send `c` and wait for the `Centred` message. It centers all joints using the existing trims. Match the links to the [CAD](cad/step/) calibration pose. In the firmware's nominal zero geometry, the femur joint-center line is horizontal and the tibia points down at a right angle to it; the coxa follows that leg's outward CAD direction. Use the joint centers as references rather than the edges of a printed bracket.

If you need to check or reseat servo horns, `c90` sends exactly **1500 microseconds** to every servo, ignoring trims. Turn servo power off before removing or reseating a horn. Place it as close to the neutral geometry as the spline allows, restore power, and use `c` for the remaining calibration.

Use `dir FL` to confirm which physical leg responds. If all left/right labels are reversed, use `swapsides`. If you want the original rear to become the front, use `swapends`. Both commands save immediately. Recheck all leg names after either change, before calibrating individual joints or the IMU.

## 3. Check servo directions

With the robot still supported, test each leg:

```text
dir FL
dir ML
dir RL
dir FR
dir MR
dir RR
```

Run these separately. Each test centers that leg, moves one joint by +15 degrees, and returns it to zero. Read the expected motion printed for each joint. The femur should raise the foot, and the tibia should swing it outward; use the printed front/rear prompt for the coxa.

If a joint moves the wrong way, flip that joint and repeat its test. For example:

```text
flip FL f
dir FL
```

`flip` toggles the direction each time and locks walking until `save`. Check all 18 joints, then send `save`.

## 4. Adjust neutral positions and angle scale

### Neutral trims

Send `c` again. Correct small neutral errors with pulse offsets:

```text
trim FL f 10
```

This **adds 10 microseconds** to the current trim. It does not set the trim to 10 and does not mean 10 degrees. Use small positive or negative increments, observe the movement, and use `show` to read the resulting trim. A positive pulse change can move opposite ways on mirrored servos.

Repeat as needed for each joint, then `save`. Large neutral errors are a reason to recheck the horn position and assembly.

### Three-point angle calibration

This also checks how far a joint actually moves on either side of zero. Support the robot with its legs free. For each joint:

1. Start with `cal FL f`, replacing the leg and joint as needed. All joints move to their current neutral positions.
2. At the **0-degree** mark, use `adj 5`, `adj -5` or other small pulse increments to align the joint. Send `cal next` to commit that mark.
3. At the **positive** mark, adjust until the measured angle matches the prompt, then send `cal next`.
4. At the **negative** mark, adjust again, then send `cal done`.
5. Repeat for the other joints, then send `save`.

Coxa marks are **0, +30 and -30 degrees**. Femur and tibia marks are **0, +40 and -40 degrees**. Measure these as rotations from the neutral pose about that joint. `adj` adds microseconds to the current calibration nudge.

`cal cancel` stops the session and returns the current joint to zero. Steps already committed remain in RAM; cancellation does not undo them. `trim zero` clears every trim and resets every angle scale to 1.00, so use it only when deliberately starting over.

## 5. Calibrate the battery reading

Use the installed battery divider and a multimeter reading across the battery. Keep the robot still. If the measured voltage is 7.84 V, send:

```text
batcal 7.84
bat
```

Replace 7.84 with your measurement. `batcal` saves immediately. Check the reported current voltage against the meter; the estimated rest voltage can take time to settle.

For a second calibration point, repeat later at a measured voltage about 0.5 V or more different. The firmware uses two-point calibration when the saved readings are sufficiently separated. `batcal reset` restores the default divider scale.

## 6. Calibrate the IMU mounting

Put the robot on a flat, level surface after the joint checks. Send `stand`, `x` and `level off`, then wait until the feet and body settle and the old leveling correction fades. If standing is not yet reliable, support the body in a repeatable level pose and use `off`.

First check:

```text
mpustatus
```

The MPU should be reading. Compare repeated status reports to see whether failed samples are still accumulating. Fix a missing sensor or repeated transfer failures before relying on leveling.

### Level reference

Keep the body level and still, then send:

```text
levelcal
```

Wait for `Level saved`. This saves the vertical reference immediately, turns leveling off, and clears the old front-direction calibration.

### Front direction

Raise the robot's selected **front** by about **20 degrees**, keeping its rear lower. Support that pose steadily before sending:

```text
mpufront
```

Wait for `Orientation saved`, then return the robot to level. If the firmware says the tilt is insufficient, raise the front farther and retry while still.

### Verify the signs

With leveling still off, send `tilt` in each pose:

| Pose | Expected reading |
| --- | --- |
| Level and still | Roll and pitch close to zero |
| Front raised | Positive pitch |
| Left side raised | Positive roll |

Return the robot upright on its feet before sending `level on`. If a sign is wrong, check the selected front and repeat `levelcal` followed by `mpufront`. Repeat mounting calibration whenever you move or remount the MPU.

While standing still in the normal stance, residual gyro auto-zero uses at least 75 valid samples over 1.5 seconds. `mpustatus` shows its quiet window and re-zero count. Rejection can mean motion, acceleration noise, insufficient valid reads or a long read gap. If the sensor reports motion while you are certain it is still, `mpu` restarts the sensor and remeasures gyro bias; hold still throughout that command, then retry mounting calibration.

## 7. Optional load and foot-height tuning

These steps refine an already working stance. Use the complete robot with its battery, head and other attachments fitted. Stop movement, use `level off`, and wait for corrections to fade before measuring.

### Mass and load assumptions

Weigh the whole robot and enter its mass in grams. For example, a measured 870 g robot uses `mass 870`. Use `stall <kgf cm>` only with a defensible servo rating at the actual supply voltage. Raising that number changes the software's load limit; it does not increase physical servo strength. `torque` and `loads` report model estimates.

`com X Y` sets a measured horizontal center-of-mass offset in mm using the firmware body axes, x forward and y left. Leave it at the existing setting if you have no measurement. It is separate from tilt zeroing. Keep servo-speed, lag and backlash settings unchanged until you have measurements for them.

### Sag measurement

1. Send `stand`, then `x` and `level off`. Wait until settled and note the current `params` output.
2. Send `sagtest`. It lifts FL, RL and MR for four seconds.
3. Measure how far the body drops at the middle of each side. Average the left and right drops.
4. Wait until all feet arwe down and settled, then send `sag <average drop in mm>`.
5. Check the response and stance before sending `save`.

For drops of 2.2 and 2.5 mm, the average is 2.35 mm, so the command is `sag 2.35`. The reported feed-forward value can differ from 2.35 because the firmware fits a compliance model. Existing sag compensation remains active during the test and is included in that fit. If the body rises, enter a negative drop. `sag 0` disables compensation. Repeat the measurement after changing stance geometry or payload; do not reuse a measurement from another stance.

### Foot heights with a kitchen scale

Use a flat table, a measured total `mass`, and a stable stance. For an initial uncorrected measurement, send `zfix off`. Measure the scale platform's height above the table, then set it with `zscale h <mm>`.

1. Send `zscale reset` to clear the temporary readings.
2. Send `zscale FL`. Wait for the foot to lift, then slide the zeroed scale underneath with only that foot over the platform.
3. Send `zscale down`. Wait two to three seconds for the reading to settle.
4. Send `zscale <grams>` with the measured load. Wait for the foot to lift, then remove the scale.
5. Repeat for ML, RL, FR, MR and RR, removing the scale between legs.
6. After the last reading, remove the scale and send `zscale done`. Wait for the foot to return to the table.

An accepted fit turns `zfix` on. Check the reported fit error and the stance, then send `save`. If the fit is noisy or rejected, use `zscale reset` and repeat the measurements.

### Foot heights with a paper strip

This is an alternative to the kitchen scale. Run sag measurement first for a better compliance estimate, keep leveling off, and send `zfix off` before measuring.

1. Send `zcal reset`.
2. Place a strip of paper under FL and send `zcal FL`.
3. Gently tug the paper as the foot rises. Send `zcal ok` when the paper slides free.
4. Wait for the foot to lower. Repeat for all six feet; two readings per foot are preferable.
5. Send `zcal done`, inspect the reported fit, then use `zfix on` and `save` if accepted.

`zcal done` calculates the errors but does not enable correction automatically. `zscale done` does enable it after an accepted fit.

## 8. Save and check a short walk

Send `save`, then `show` and `params`. With the robot upright on its feet, send `stand` and wait until settled. Set a low speed such as `speed 20`, send `w` for a short walk, then `x` and let the feet finish landing. Test the other movement directions separately.

Check `mem` and `mpustatus` after stopping if movement becomes jerky. Rising I2C errors or control-loop delays need investigation before further calibration. To enable assists again after the basic checks, use `assist on`; enable `level on` only after the IMU sign checks, and `save` the assist settings if you want them retained.

## What gets saved

| Command or setting | Persistence |
| --- | --- |
| Directions, trims, angle scales, mass, stall, sag, foot errors, zfix and assist settings | Send `save` |
| `levelcal`, `mpufront`, `level on/off`, `batcal` | Saved immediately |
| `swapsides`, `swapends`, `height`, `zscale h` | Saved immediately |
| Temporary zcal/zscale readings and gyro auto-zero | Runtime state |

Keep a copy of `show`, `params` and `mathdump` for reference. `mathdump` is an analysis export, not a full backup of the ESP32's NVS. Normal sketch uploads with erase flash disabled are intended to retain NVS settings; another ESP32 has its own storage and needs its calibration transferred or repeated. `reset` clears the saved calibration and settings; pressing the board's EN button restarts it without clearing them.

For rejected measurements, communication errors or unstable motion, see [troubleshooting](troubleshooting.md).

Return to the [project README](README.md) or [firmware guide](firmware/README.md).

# `base.yaml`: battery monitor and teleop

## `bagheera_battery_monitor`

Bagheera's pack is a 7S Li-Ion (25.2 V nominal, 29.4 V full) without a fuel
gauge. The mainboard measures only battery voltage, charge voltage and charge
current, and the current shunt sits in the charge path, so there is no
coulomb counting. The monitor therefore makes every decision from the
averaged, offset-corrected voltage. The percentage is only a display value.

Publishes:

| Topic | Type | Content |
|---|---|---|
| `/battery/voltage` | `std_msgs/Float32` | Averaged, corrected pack voltage. |
| `/battery/percentage` | `std_msgs/Float32` | 0–100 %, derived from the voltage. |
| `/battery/level` | `std_msgs/String`, latched | `NORMAL`, `LOW`, `CRITICAL` or `FULL`. |

Nothing drives back to the dock automatically based on the level, except the
[patrol](../operation.md#waypoint-patrol), which consumes `/battery/level`.

| Parameter | Value | Meaning |
|---|---|---|
| `power_topic` | `/hardware_bridge/power` | Source of `v_battery`, `v_charge` and `charge_current`. |
| `voltage_offset_v` | −0.80 V | Added to the firmware's `v_battery`. **Measured on Bagheera**, see below. |
| `cell_count` | 7 | Series cells, used for the percentage curve. |
| `filter_window_s` | 60 s | Moving-average window. Motor load sags the voltage, and the average hides that. |
| `warmup_s` | 10 s | No level decision until this much data has arrived. |
| `low_voltage` | 25.3 V | Below this: `LOW` (about 20 %). Latched until charging starts. |
| `critical_voltage` | 24.5 V | Below this: `CRITICAL` (about 10 %). Latched until charging starts. |
| `dock_voltage_threshold` | 10.0 V | `v_charge` above this counts as docked. |
| `charging_current_a` | 0.2 A | Charge current above this counts as charging. |
| `full_voltage` | 28.4 V | `FULL` needs: docked, averaged voltage ≥ this … |
| `full_current_a` | 0.15 A | … and charge current below this … |
| `full_hold_s` | 120 s | … for this long. |
| `full_release_voltage` | 28.0 V | `FULL` is released when the voltage drops below this. |
| `publish_rate` | 1.0 Hz | Output rate. |

### Battery voltage offset

The board's `v_battery` reading has a constant offset. It was measured on
Bagheera (MowgliNext firmware 1.9.10) with a multimeter at the pack, off the
dock and while charging:

| Multimeter | Firmware | Difference |
|---:|---:|---:|
| 25.71 V | 26.49 V | −0.78 V |
| 28.10 V | 28.90 V | −0.80 V |
| 28.37 V | 29.20 V | −0.83 V |

The offset is constant and there is no gain error, so `voltage_offset_v` is
−0.80 V. The correction is applied only in ROS. The firmware's own charge
cut-off (`max_charge_voltage` 29.4 V as measured) therefore ends at about
28.6 V real.

**Measure your own board** before trusting the thresholds: compare a
multimeter at the pack with `ros2 topic echo /hardware_bridge/power` at two
or three different charge levels. Measure again after a board or firmware
change.

## `bagheera_controller`

Game-controller teleop through pygame (any controller the Linux joystick
driver exposes under `/dev/input`). **Off by default**: the node polls the
joystick continuously, which costs about 10 % of a Pi 4 core even with no
controller connected. Set `enabled: true` and run `docker compose restart`
to use it (driving, mapping, calibration drives, waking the robot).
Without it, `/dock/wake` or an autonomous goal wakes the robot.

Holding the deadman button publishes
`/cmd_vel_teleop`. Releasing it sends zero velocity once. Teleop has
priority over autonomy in `twist_mux`. Pressing the deadman in the dock also
wakes a sleeping robot (`/dock/wake`).

| Parameter | Value | Meaning |
|---|---|---|
| `enabled` | false | Read by `manual_control.launch.py`: start the node at all. |
| `cmd_vel_topic` | `/cmd_vel_teleop` | Output lane. |
| `joystick_index` | 0 | Also a launch argument. |
| `throttle_axis` | 1 | Left stick vertical: forward/reverse. |
| `steering_axis` | 2 | Right stick horizontal: rotation. |
| `deadman_button` | 4 | Must be held to drive. |
| `deadzone` | 0.05 | Stick deflection ignored around centre. |
| `max_linear_speed` | 0.16 m/s | Full stick. |
| `max_angular_speed` | 0.30 rad/s | Full stick. |
| `publish_rate` | 20 Hz | Command rate while the deadman is held. |
| `joystick_rescan_interval` | 1.0 s | Reconnect interval after the controller disappears. |

Steering is ignored while the throttle commands translation, so the robot
either drives straight or turns in place. The limits keep motion per LiDAR
scan small (~1.7 cm and ~1.8° at ~9.6 scans/s), which matters for clean maps.
Axis and button numbers differ between controllers. Check them with
`jstest /dev/input/js0` on the host.

# carla_diff_drive_bridge

ROS2 bridge for controlling a CARLA actor with differential drive `cmd_vel` commands.

Uses **kinematic override** — bypasses CARLA's vehicle physics entirely and sets velocity directly. Perfect for testing perception/planning stacks where you assume low-level control is solved.

## Architecture

```
/cmd_vel (Twist) ──→ [bridge_node] ──→ CARLA Actor (set_target_velocity)
                          │
                          ├──→ /odom (Odometry) — ground-truth from CARLA
                          └──→ /tf (odom → base_link)
```

## Quick Start

```bash
# Terminal 1: Start CARLA
./CarlaUE4.sh

# Terminal 2: Spawn a robot actor
ros2 run carla_diff_drive_bridge spawn_robot --role-name ego_robot

# Terminal 3: Start the bridge
ros2 launch carla_diff_drive_bridge diff_drive_bridge.launch.py

# Terminal 4: Test with teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `carla_host` | `localhost` | CARLA server address |
| `carla_port` | `2000` | CARLA server port |
| `actor_id` | `-1` | Explicit actor ID (-1 = auto-find by role_name) |
| `actor_role_name` | `ego_robot` | Find actor by this role_name attribute |
| `control_rate` | `20.0` | Hz — how often velocity is applied |
| `cmd_vel_topic` | `/cmd_vel` | Input twist topic |
| `odom_topic` | `/odom` | Output odometry topic |
| `cmd_vel_timeout` | `0.5` | Seconds — zeros velocity if no cmd_vel received |
| `publish_tf` | `true` | Broadcast odom → base_link TF |

## Coordinate Conventions

The bridge handles ROS ↔ CARLA coordinate conversion:
- **ROS**: right-handed, +Y left, yaw CCW positive
- **CARLA**: left-handed, +Y right, yaw CW positive
- Y-axis and angular Z are inverted at the boundary

## Notes

- `spawn_robot` disables physics by default (pure kinematic). Pass `--enable-physics` if you want CARLA to simulate dynamics.
- The bridge auto-retries actor discovery if it's not spawned yet.
- On shutdown, velocity is zeroed (safety stop).
- Odometry origin is set at the actor's position on first tick.

# Tactile simulation bringup

This is the working Gazebo Sim 6 bringup for the shared `fractal_panda_description` model with the Fractal tactile pad. The project intentionally uses the same description package for simulation and the future project-owned real-robot launch. The control backend changes by mode; the robot model does not get duplicated.

## What the launch starts

`simulation_stack.launch.py` starts:

- Gazebo Sim 6 through `ros_gz_sim`;
- the `tactile_probe.sdf` world with gravity `0 0 -9.81`;
- `robot_state_publisher` using `fractal_panda_description/urdf/fractal_panda.urdf.xacro`;
- the custom Panda and, by default, the nine-link Fractal pad;
- the local `franka_ign_ros2_control` gravity-compensating simulation backend;
- `joint_state_broadcaster` and `panda_arm_controller`;
- MoveIt `move_group` and RViz;
- the simulated tactile sensor and exploration lifecycle node;
- the static `world` to `tactile_map` transform.

The exploration node is configured and activated by the launch. Use `run_smoke_test:=true` for a bounded topic-level check.

## Build and source

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  franka_ign_ros2_control \
  franka_gazebo_bringup \
  fractal_panda_description \
  fractal_tactile_exploration \
  --symlink-install
source install/setup.bash
```

The local `franka_ign_ros2_control` package is the Ignition/Gazebo 6 implementation. Do not replace its plugin names with newer Harmonic `gz_ros2_control` names without changing the complete simulation integration.

## Standard launch

Start the full simulation:

```bash
ros2 launch fractal_tactile_exploration simulation_stack.launch.py
```

For a lighter run without RViz:

```bash
ros2 launch fractal_tactile_exploration simulation_stack.launch.py \
  use_rviz:=false
```

Gravity must remain enabled when validating physical support. Zero gravity hides gravity-compensation and inertial problems.

## Shared description and control split

The shared model is:

```text
fractal_panda_description/urdf/fractal_panda.urdf.xacro
```

The xacro selects its control backend from `sim_gazebo`:

| Mode | `sim_gazebo` | Hardware plugin | Controller configuration |
| --- | --- | --- | --- |
| Gazebo simulation | `true` | `franka_ign_ros2_control/IgnitionSystem` | `fractal_tactile_exploration/config/simulation_controllers.yaml` |
| Future real launch | `false` | `franka_hardware/FrankaHardwareInterface` | Project-owned real controller YAML |

The current simulation launch is complete. The project-owned real launch is intentionally not created yet; the manufacturer launch files under `franka_arm_ros2` are reference code only. Do not use the manufacturer launch as the final real bringup because it hardcodes the manufacturer description package instead of the shared custom description.

## Simulation controller configuration

[simulation_controllers.yaml](../config/simulation_controllers.yaml) defines:

- controller-manager update rate: `250 Hz`;
- `joint_state_broadcaster`;
- `panda_arm_controller` using `joint_trajectory_controller/JointTrajectoryController`;
- effort command interfaces;
- position and velocity state interfaces;
- the seven Panda joints;
- the gains pulled from the working original repository configuration.

The active controller selects effort. The dedicated Ignition hardware plugin adds model-based gravity torques. This is why the simulation uses the effort path rather than treating the generic Gazebo position backend as equivalent to the Franka control stack.

Do not use this controller file for the real robot. Sharing the description does not mean sharing controller configuration.

## Fractal pad isolation

The launch argument `use_fractal_pad` defaults to `true`:

```bash
ros2 launch fractal_tactile_exploration simulation_stack.launch.py \
  use_fractal_pad:=true
```

Disable the pad to isolate Panda and controller behavior:

```bash
ros2 launch fractal_tactile_exploration simulation_stack.launch.py \
  use_fractal_pad:=false \
  use_rviz:=false
```

When disabled, the pad links, pad joints, pad visuals, pad collisions, and pad mass properties are omitted. `omega_contact_tip` remains in the model and is attached directly to `panda_link8`, so sensor and planning frame names remain available.

## Surface and experiment arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `surface_mode` | `bump` | Use `plane` to disable the bump |
| `surface_origin_x` | `-0.20` | Surface origin x position |
| `surface_origin_y` | `-0.20` | Surface origin y position |
| `surface_size_x` | `0.40` | Surface size in x |
| `surface_size_y` | `0.40` | Surface size in y |
| `surface_base_z` | `0.0` | Surface height |
| `bump_x` | `0.05` | Bump center x |
| `bump_y` | `0.05` | Bump center y |
| `bump_height` | `0.008` | Bump height |
| `bump_sigma` | `0.04` | Bump width |
| `max_probes` | `3` | Number of exploration probes |
| `run_smoke_test` | `false` | Run the bounded topic smoke test |
| `use_fractal_pad` | `true` | Include the tactile pad in the robot model |

Flat-surface smoke test:

```bash
ros2 launch fractal_tactile_exploration simulation_stack.launch.py \
  surface_mode:=plane \
  max_probes:=1 \
  run_smoke_test:=true
```

## Recommended debugging order

Change one factor at a time and keep gravity enabled:

1. Run with `use_fractal_pad:=false`. If the Panda still jitters, investigate controller gains, effort command behavior, update rates, and Panda inertial data.
2. Run with `use_fractal_pad:=true` and `surface_mode:=plane`. If jitter appears only after adding the pad, investigate pad mass, inertia, fixed-joint placement, and collision geometry.
3. Enable the bump. If jitter begins only during contact, investigate contact geometry and contact stiffness before changing arm gains.
4. Confirm the active controllers and interfaces:

```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
```

5. Inspect state timing and motion:

```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
ros2 topic echo /tf --once
```

A jitter that exists with the pad removed points away from pad inertia. A jitter that appears only with the pad enabled points toward the added model or contact path.

## Useful tactile and exploration topics

```bash
ros2 topic echo /omega_explorer_node/next_omega_target geometry_msgs/msg/PoseStamped
ros2 topic echo /omega_explorer_node/map_sample fractal_tactile_exploration/msg/FractalMapSample
ros2 topic echo /paxini/L5325_omega/tactile_sensor paxini_hardware/msg/TactileSensor
```

## Static checks without starting Gazebo

Generate both robot-description variants:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
controller_file="$PWD/fractal_tactile_exploration/config/simulation_controllers.yaml"

for use_pad in true false; do
  xacro fractal_panda_description/urdf/fractal_panda.urdf.xacro \
    sim_gazebo:=true \
    planning_mode:=true \
    use_fractal_pad:="$use_pad" \
    simulation_controllers:="$controller_file" \
    > "/tmp/fractal_panda_${use_pad}.urdf"
done
```

This catches xacro errors and missing links without starting ROS or Gazebo. Before a future real-robot launch is created, also expand the same xacro with `sim_gazebo:=false` and a real `robot_ip`; verify that it contains `franka_hardware/FrankaHardwareInterface` and no Gazebo plugin.

## Real-robot boundary

The future project-owned real launch must use this same xacro with simulation disabled, the real hardware plugin, and a separately reviewed real controller configuration. Do not use `simulation_controllers.yaml` on hardware, do not pass `sim_gazebo:=true`, and do not edit the shared description casually: a model change affects both simulation and the future real path. See [real_robot_bringup.md](real_robot_bringup.md) for the boundary and hardware checklist.

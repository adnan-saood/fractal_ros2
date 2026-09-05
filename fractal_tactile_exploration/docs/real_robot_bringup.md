# Real Franka robot bringup

This document records the real-hardware boundary for the Panda. The project does not use the manufacturer launch files as its final real-robot launch; a project-owned real launch will be created later.

## Important separation

The shared model requirement is:

- Description package: `fractal_panda_description`
- Shared xacro: `fractal_panda_description/urdf/fractal_panda.urdf.xacro`

The project-owned real launch must use that same xacro with simulation disabled and the real hardware backend selected. The reference control pieces under `franka_arm_ros2` are:

- Hardware and control node: `franka_control2`
- Bringup: `franka_bringup`
- Hardware plugin: `franka_hardware/FrankaHardwareInterface`
- Controllers: `franka_arm_ros2/franka_bringup/config/controllers.yaml`
- Main launch: `franka_arm_ros2/franka_bringup/launch/franka.launch.py`

The simulation uses the same description package, but a different control path:

- Simulation controller configuration: `fractal_tactile_exploration/config/simulation_controllers.yaml`
- Simulator: Gazebo Sim 6 through `ros_gz_sim`
- Simulation hardware plugin: `franka_ign_ros2_control/IgnitionSystem`

Do not use the simulation controller YAML on the real robot. The description is shared intentionally; the future real launch must use the same xacro with `sim_gazebo:=false`, which selects `franka_hardware/FrankaHardwareInterface` instead of the Gazebo plugin. Do not modify or run a manufacturer launch as the project real bringup.

## Preconditions

Complete these checks before connecting to the robot:

1. The robot is in a safe operating state and the emergency stop is released.
2. The robot network connection is configured according to the robot and libfranka documentation.
3. The robot FCI is enabled and the robot IP address is reachable from the control computer.
4. The installed libfranka version is compatible with the robot FCI version and this workspace. This workspace was tested with libfranka 0.9.2.
5. No other process is already commanding the robot.
6. The control computer has the required real-time permissions and network configuration.

Use the actual robot IP address supplied by the robot configuration. Do not use the simulation value `0.0.0.0`.

## Build and source

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  franka_description \
  franka_hardware \
  franka_control2 \
  franka_robot_state_broadcaster \
  franka_bringup \
  franka_gripper \
  franka_example_controllers \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If libfranka is installed outside the system default location, use the same `Franka_DIR` and library path settings used when the workspace was originally built. Do not rebuild against a different libfranka version without checking FCI compatibility first.

## Reference only: manufacturer launch

The existing manufacturer launch is retained under `franka_arm_ros2/franka_bringup` for reference and is not the project real-robot workflow. It hardcodes the manufacturer `franka_description` model, so it does not satisfy the shared-description requirement.

Do not use it as the project bringup. In particular, do not treat this command as the final real launch:

```bash
ros2 launch franka_bringup franka.launch.py \
  robot_ip:=<ROBOT_IP> \
  load_gripper:=false \
  use_rviz:=true
```

Example form, with a placeholder IP:

```bash
ros2 launch franka_bringup franka.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  use_rviz:=true
```

The project-owned real launch will later need equivalent arguments for robot IP, gripper selection, RViz, and fake hardware, while using `fractal_panda_description` and the real controller configuration. Until that launch is created and validated, do not connect the custom shared model to hardware.

For reference only, the manufacturer launch also supports:

```bash
ros2 launch franka_bringup franka.launch.py \
  robot_ip:=<ROBOT_IP> \
  load_gripper:=true \
  use_rviz:=true
```

`use_fake_hardware:=true` is for a non-hardware test of the manufacturer bringup stack. It does not connect to or move a physical robot and must not be confused with the Gazebo simulation.

## What the real launch starts

`franka.launch.py`:

1. The future project launch must expand `fractal_panda_description/urdf/fractal_panda.urdf.xacro` with `sim_gazebo:=false`.
2. Starts `robot_state_publisher`.
3. Starts `franka_control2_node` with `franka_bringup/config/controllers.yaml`.
4. Loads `joint_state_broadcaster`.
5. Loads `franka_robot_state_broadcaster` when fake hardware is disabled.
6. Optionally starts the gripper launch.
7. Optionally starts RViz.

The default real controller configuration contains the Franka example controllers and `joint_trajectory_controller`. It is separate from `fractal_tactile_exploration/config/simulation_controllers.yaml`.

## First checks after startup

In another sourced terminal:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /franka/joint_states --once
ros2 topic echo /franka_robot_state_broadcaster/robot_state --once
```

Confirm that the hardware interface is active and that joint states are changing plausibly before sending a motion command. Stop immediately if the robot reports an error, unexpected joint motion, or a controller fails to activate.

## Controller safety

The real controller file contains gains intended for the real Franka control stack. Treat changes to these values as hardware changes, not simulation tuning. Do not copy the lower simulation gains from `simulation_controllers.yaml` into the real controller file, and do not copy the simulation effort controller into the real bringup.

For the first hardware test, use the existing launch and existing controller configuration unchanged. Test state publication first, then a small, supervised motion using the controller workflow already established for this robot. Keep the emergency stop accessible.

## Relationship to the tactile simulation

The tactile simulation and future real launch must use `fractal_panda_description`. The simulation passes `sim_gazebo:=true` and loads the simulation controller YAML. The real launch must pass `sim_gazebo:=false`, use the real hardware plugin, and load a real-robot controller configuration compatible with the hardware node. A physical tactile-pad integration requires a review of:

- measured pad mass and inertia;
- the pad attachment and collision geometry;
- real joint/load limits;
- controller torque and collision behavior;
- the tactile hardware driver and electrical interface.

Do not connect the shared custom model to the real robot until the project-owned launch, controller handoff, mass properties, collision geometry, and safety behavior have been validated. Do not copy simulation gains or the simulation controller YAML into the real path.

# 01_3dof_arm: 3-DOF Articulated Robotic Arm with Parallel Gripper

A fully articulated 3-DOF robotic manipulator with a rotating base turret, parallel pitch shoulder and elbow pivots, and a functional two-finger parallel gripper. Designed for ROS 2 Humble and Gazebo Fortress (Ignition).

---

## Kinematics & Joint Structure

| Joint | Type | Axis | Parent -> Child | Motion | Limits |
| :--- | :---: | :---: | :---: | :--- | :---: |
| `joint1` | Revolute | `[0, 0, 1]` (Z) | `base_link` -> `link1` | Base yaw rotation (left / right) | [-3.14, +3.14] rad |
| `joint2` | Revolute | `[0, 1, 0]` (Y) | `link1` -> `link2` | Shoulder pitch (raise / lower arm) | [-1.74, +1.74] rad |
| `joint3` | Revolute | `[0, 1, 0]` (Y) | `link2` -> `link3` | Elbow pitch (bend / extend forearm) | [-2.09, +2.09] rad |
| `tool_joint` | Fixed | - | `link3` -> `gripper_base_link` | Wrist mounting flange | - |
| `left_finger_joint` | Prismatic | `[0, 1, 0]` (Y) | `gripper_base_link` -> `left_finger_link` | Left finger sliding open/close | [0.0, 0.035] m |
| `right_finger_joint` | Prismatic | `[0, 1, 0]` (Y) | `gripper_base_link` -> `right_finger_link` | Right finger symmetric mimic motion | [-0.035, 0.0] m |
| `tcp_joint` | Fixed | - | `gripper_base_link` -> `tool_link` | Tool Center Point (TCP) frame | - |

> [!NOTE]
> **Mechanical Plane Alignment**: Both `joint2` (shoulder) and `joint3` (elbow) rotate about parallel Y-axes (`[0, 1, 0]`), ensuring natural planar bending in the vertical elevation plane without lateral twisting.

---

## Controllers & ros2_control

Configured in [`config/controllers.yaml`](file:///media/bumble/DATA_DISK/robotics/projects/01_3dof_arm/config/controllers.yaml):
- `joint_state_broadcaster`: Broadcasts joint states at 100 Hz.
- `arm_controller`: `joint_trajectory_controller` for `joint1`, `joint2`, `joint3`.
- `gripper_controller`: `joint_trajectory_controller` for `left_finger_joint`, `right_finger_joint`.
- `joint_position_controller` & `gripper_position_controller`: Forward group position controllers for direct setpoint commanding.

---

## How to Run

### 1. View in RViz2 (with joint sliders)
```bash
ros2 launch launch/display.launch.py
```

### 2. Launch in Gazebo Simulator
```bash
ros2 launch launch/gazebo.launch.py
```

### 3. Run Verification Tests
```bash
# Basic URDF and link connectivity check
python3 test_robot.py

# Kinematics, inertia, limits, and mimic verification
python3 test_extra.py

# Individual joint direction, limit, and alignment tests
python3 test_joints.py

# 9-Step Autonomous Physics Pick-and-Place Demonstration
python3 test_pick_and_place.py
```

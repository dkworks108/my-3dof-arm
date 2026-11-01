import os
import sys
import math
import pybullet as p
import pybullet_data
import numpy as np

def run_joint_validation():
    print("==================================================")
    print("  01_3dof_arm: Individual Joint & Kinematic Tests ")
    print("==================================================")

    cid = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)

    cur_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(cur_dir, 'urdf', 'robot.urdf')
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

    joint_map = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        joint_map[info[1].decode('utf-8')] = i

    j1 = joint_map['joint1']
    j2 = joint_map['joint2']
    j3 = joint_map['joint3']
    left_finger = joint_map['left_finger_joint']
    right_finger = joint_map['right_finger_joint']

    all_tests_passed = True

    # 1. Test J1: Base Rotation around Z (left vs right)
    print("\n--- Test 1: J1 Base Rotation (Z-Axis) ---")
    # Pitch shoulder forward slightly so the tool is offset from the rotation axis
    p.resetJointState(robot_id, j2, 0.50)
    p.resetJointState(robot_id, j3, 0.0)

    p.resetJointState(robot_id, j1, 0.785)  # +45 deg (turns left towards +Y)
    pos_pos = p.getLinkState(robot_id, joint_map['tool_joint'])[4]
    p.resetJointState(robot_id, j1, -0.785) # -45 deg (turns right towards -Y)
    pos_neg = p.getLinkState(robot_id, joint_map['tool_joint'])[4]

    j1_dir_ok = pos_pos[1] > 0 and pos_neg[1] < 0
    print(f"J1 +45 deg Y-coord: {pos_pos[1]:.4f} (expected > 0, turns left)")
    print(f"J1 -45 deg Y-coord: {pos_neg[1]:.4f} (expected < 0, turns right)")
    print(f"J1 direction check: {'PASS' if j1_dir_ok else 'FAIL'}")
    all_tests_passed = all_tests_passed and j1_dir_ok

    # 2. Test J2: Shoulder Pitch around Y (raise vs lower)
    print("\n--- Test 2: J2 Shoulder Pitch (Y-Axis) ---")
    p.resetJointState(robot_id, j1, 0.0)
    p.resetJointState(robot_id, j3, 0.0)
    p.resetJointState(robot_id, j2, 0.50)  # positive pitch: forward/down
    pos_fwd = p.getLinkState(robot_id, joint_map['tool_joint'])[4]
    p.resetJointState(robot_id, j2, -0.50) # negative pitch: backward/up
    pos_back = p.getLinkState(robot_id, joint_map['tool_joint'])[4]

    j2_dir_ok = pos_fwd[0] > 0 and pos_back[0] < 0 and pos_fwd[1] == 0.0 and pos_back[1] == 0.0
    print(f"J2 +0.5 rad X-coord: {pos_fwd[0]:.4f} (forward towards +X)")
    print(f"J2 -0.5 rad X-coord: {pos_back[0]:.4f} (backward towards -X)")
    print(f"J2 planar motion (Y=0.0): {pos_fwd[1] == 0.0 and pos_back[1] == 0.0}")
    print(f"J2 direction check: {'PASS' if j2_dir_ok else 'FAIL'}")
    all_tests_passed = all_tests_passed and j2_dir_ok

    # 3. Test J3: Elbow Pitch around Y (bend vs extend)
    print("\n--- Test 3: J3 Elbow Pitch (Y-Axis) ---")
    p.resetJointState(robot_id, j2, 0.50)
    p.resetJointState(robot_id, j3, 0.50)  # further bend forward/down
    pos_bend = p.getLinkState(robot_id, joint_map['tool_joint'])[4]
    p.resetJointState(robot_id, j3, -0.50) # extend upward
    pos_ext = p.getLinkState(robot_id, joint_map['tool_joint'])[4]

    j3_dir_ok = pos_bend[0] > pos_fwd[0] and pos_bend[2] < pos_ext[2] and pos_bend[1] == 0.0
    print(f"J3 +0.5 rad (bend down) Z-coord: {pos_bend[2]:.4f}")
    print(f"J3 -0.5 rad (extend up) Z-coord: {pos_ext[2]:.4f}")
    print(f"J3 direction check: {'PASS' if j3_dir_ok else 'FAIL'}")
    all_tests_passed = all_tests_passed and j3_dir_ok

    # 4. Test Parallel Rotational Axes for J2 and J3
    print("\n--- Test 4: Shoulder & Elbow Parallel Axis Alignment ---")
    j2_info = p.getJointInfo(robot_id, j2)
    j3_info = p.getJointInfo(robot_id, j3)
    axis_j2 = np.array(j2_info[13])
    axis_j3 = np.array(j3_info[13])

    dot_prod = np.dot(axis_j2, axis_j3)
    parallel = math.isclose(dot_prod, 1.0, abs_tol=1e-5)
    print(f"J2 joint rotation axis: {axis_j2}")
    print(f"J3 joint rotation axis: {axis_j3}")
    print(f"Dot product of rotational axes: {dot_prod:.6f} (expected 1.000000)")
    print(f"Parallel mechanical plane alignment: {'PASS' if parallel else 'FAIL'}")
    all_tests_passed = all_tests_passed and parallel

    # 5. Test Gripper Symmetry
    print("\n--- Test 5: Gripper Symmetric Motion ---")
    test_displacements = [0.0, 0.010, 0.020, 0.030]
    symmetry_ok = True
    for d in test_displacements:
        p.resetJointState(robot_id, left_finger, d)
        p.resetJointState(robot_id, right_finger, -d)

        lf_pos = p.getLinkState(robot_id, left_finger)[0]
        rf_pos = p.getLinkState(robot_id, right_finger)[0]
        tcp_pos = p.getLinkState(robot_id, joint_map['tcp_joint'])[0]

        d_left = np.linalg.norm(np.array(lf_pos) - np.array(tcp_pos))
        d_right = np.linalg.norm(np.array(rf_pos) - np.array(tcp_pos))
        diff = abs(d_left - d_right)

        if diff > 1e-4:
            symmetry_ok = False
        print(f"Command d={d*1000:.1f}mm -> Left distance: {d_left*1000:.2f}mm, Right distance: {d_right*1000:.2f}mm (delta: {diff*1000:.3f}mm)")

    print(f"Gripper symmetry check: {'PASS' if symmetry_ok else 'FAIL'}")
    all_tests_passed = all_tests_passed and symmetry_ok

    p.disconnect()

    print("\n==================================================")
    if all_tests_passed:
        print(" PASS: All Individual Joint Validations Succeeded! ")
    else:
        print(" FAIL: One or more joint validations failed.")
    print("==================================================")
    return all_tests_passed

if __name__ == '__main__':
    ok = run_joint_validation()
    sys.exit(0 if ok else 1)

import os
import sys
import time
import math
import pybullet as p
import pybullet_data
import numpy as np
from PIL import Image

def run_pick_and_place():
    print("==================================================")
    print("  01_3dof_arm: Physics Pick-and-Place Validation  ")
    print("==================================================")

    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)

    # Load plane
    plane_id = p.loadURDF("plane.urdf", [0, 0, 0])

    # Load robot
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(cur_dir, 'urdf', 'robot.urdf')
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

    # Map joint names and link names to indices
    joint_name_to_id = {}
    link_name_to_id = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        j_name = info[1].decode('utf-8')
        l_name = info[12].decode('utf-8')
        joint_name_to_id[j_name] = i
        link_name_to_id[l_name] = i

    j1 = joint_name_to_id['joint1']
    j2 = joint_name_to_id['joint2']
    j3 = joint_name_to_id['joint3']
    left_finger = joint_name_to_id['left_finger_joint']
    right_finger = joint_name_to_id['right_finger_joint']
    tool_link_idx = link_name_to_id['tool_link']

    # Set finger friction parameters
    for fid in [left_finger, right_finger]:
        p.changeDynamics(robot_id, fid, lateralFriction=4.0, spinningFriction=0.1, rollingFriction=0.01)

    # Determine pick target coordinates from kinematics
    p.resetJointState(robot_id, j1, 0.70)
    p.resetJointState(robot_id, j2, 1.25)
    p.resetJointState(robot_id, j3, 1.50)
    tcp_pos = p.getLinkState(robot_id, tool_link_idx)[0]
    pick_x = tcp_pos[0]
    pick_y = tcp_pos[1]
    obj_z = 0.030

    print(f"Target object pick position: X={pick_x:.3f}, Y={pick_y:.3f}, Z={obj_z:.3f}")

    # Create target graspable cylinder object (mass = 40g, radius = 10mm, height = 60mm)
    col_cyl = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.010, height=0.060)
    vis_cyl = p.createVisualShape(p.GEOM_CYLINDER, radius=0.010, length=0.060, rgbaColor=[0.9, 0.15, 0.15, 1.0])
    obj_id = p.createMultiBody(
        baseMass=0.04,
        baseCollisionShapeIndex=col_cyl,
        baseVisualShapeIndex=vis_cyl,
        basePosition=[pick_x, pick_y, obj_z]
    )
    p.changeDynamics(obj_id, -1, lateralFriction=4.0, spinningFriction=0.1, rollingFriction=0.01)

    # Set robot to HOME pose
    p.resetJointState(robot_id, j1, 0.0)
    p.resetJointState(robot_id, j2, 0.0)
    p.resetJointState(robot_id, j3, 0.0)
    p.resetJointState(robot_id, left_finger, 0.030)
    p.resetJointState(robot_id, right_finger, -0.030)

    shots_dir = os.path.join(cur_dir, 'screenshots')
    os.makedirs(shots_dir, exist_ok=True)

    def capture_frame(filename, camera_eye=[0.85, 0.70, 0.65], camera_target=[0.20, 0.0, 0.25]):
        view_mat = p.computeViewMatrix(camera_eye, camera_target, [0, 0, 1])
        proj_mat = p.computeProjectionMatrixFOV(55.0, 4.0/3.0, 0.1, 5.0)
        _, _, rgb, _, _ = p.getCameraImage(800, 600, view_mat, proj_mat, renderer=p.ER_TINY_RENDERER)
        img = Image.fromarray(np.reshape(rgb, (600, 800, 4))[:, :, :3])
        img.save(os.path.join(shots_dir, filename))

    curr_pose = [0.0, 0.0, 0.0, 0.030]

    def interpolate_to(target, steps=120):
        nonlocal curr_pose
        traj = [
            [curr_pose[k] + (target[k] - curr_pose[k]) * (i + 1) / steps for k in range(4)]
            for i in range(steps)
        ]
        for pt in traj:
            p.setJointMotorControl2(robot_id, j1, p.POSITION_CONTROL, pt[0], force=60.0, maxVelocity=1.5)
            p.setJointMotorControl2(robot_id, j2, p.POSITION_CONTROL, pt[1], force=60.0, maxVelocity=1.5)
            p.setJointMotorControl2(robot_id, j3, p.POSITION_CONTROL, pt[2], force=50.0, maxVelocity=1.5)
            p.setJointMotorControl2(robot_id, left_finger, p.POSITION_CONTROL, pt[3], force=35.0, maxVelocity=0.2)
            p.setJointMotorControl2(robot_id, right_finger, p.POSITION_CONTROL, -pt[3], force=35.0, maxVelocity=0.2)
            p.stepSimulation()
        curr_pose = target

    # ==================== Step 1: Move arm to home position ====================
    print("\n[Step 1/9] Move arm to HOME position (upright, gripper open)...")
    interpolate_to([0.0, 0.0, 0.0, 0.030], steps=30)
    capture_frame("pnp_step1_home.png")

    # ==================== Step 2: Rotate base towards object ====================
    print("[Step 2/9] Rotate base towards object (J1 = 0.70 rad)...")
    interpolate_to([0.70, 0.0, 0.0, 0.030], steps=120)
    capture_frame("pnp_step2_rotate.png")

    # ==================== Step 3 & 4: Lower shoulder and extend elbow to approach object ====================
    print("[Step 3 & 4] Pre-approach and lower shoulder / extend elbow to envelope object...")
    interpolate_to([0.70, 0.80, 0.90, 0.030], steps=120)
    interpolate_to([0.70, 1.25, 1.50, 0.030], steps=120)
    capture_frame("pnp_step3_approach.png")

    # ==================== Step 5: Close gripper around object ====================
    print("[Step 5/9] Close gripper firmly around the object...")
    interpolate_to([0.70, 1.25, 1.50, 0.000], steps=180)
    contacts = p.getContactPoints(robot_id, obj_id)
    print(f" -> Contact points between fingers and object: {len(contacts)}")
    capture_frame("pnp_step4_grasp.png")

    # ==================== Step 6: Lift the object ====================
    print("[Step 6/9] Lift the object into the air...")
    interpolate_to([0.70, 0.50, 0.50, 0.000], steps=200)
    lifted_pos, _ = p.getBasePositionAndOrientation(obj_id)
    print(f" -> Lifted object height: Z = {lifted_pos[2]:.3f} m (initial was {obj_z:.3f} m)")
    lift_success = lifted_pos[2] > 0.25
    print(f" -> Physical lift successful (>25cm off ground): {lift_success}")
    capture_frame("pnp_step5_lift.png")

    # ==================== Step 7: Rotate towards destination ====================
    print("[Step 7/9] Rotate base towards destination (J1 = -0.70 rad)...")
    interpolate_to([-0.70, 0.50, 0.50, 0.000], steps=220)
    transit_pos, _ = p.getBasePositionAndOrientation(obj_id)
    print(f" -> Object position in transit: X={transit_pos[0]:.3f}, Y={transit_pos[1]:.3f}, Z={transit_pos[2]:.3f}")
    capture_frame("pnp_step6_transit.png")

    # ==================== Step 8: Lower arm at destination ====================
    print("[Step 8/9] Lower the arm to place the object at destination...")
    interpolate_to([-0.70, 1.25, 1.50, 0.000], steps=200)
    capture_frame("pnp_step7_lower.png")

    # ==================== Step 9: Open gripper and release object ====================
    print("[Step 9/9] Open gripper and release the object...")
    interpolate_to([-0.70, 1.25, 1.50, 0.030], steps=120)
    capture_frame("pnp_step8_release.png")

    # Retract arm safely
    interpolate_to([-0.70, 0.50, 0.50, 0.030], steps=120)
    capture_frame("pnp_step9_retract.png")

    final_pos, _ = p.getBasePositionAndOrientation(obj_id)
    print(f"\nFinal object placement coordinates: X={final_pos[0]:.3f}, Y={final_pos[1]:.3f}, Z={final_pos[2]:.3f}")

    transport_dist = math.sqrt((final_pos[0] - pick_x)**2 + (final_pos[1] - pick_y)**2)
    print(f"Total transport distance: {transport_dist:.3f} m")

    place_success = transport_dist > 0.30 and final_pos[2] < 0.06

    p.disconnect()

    if lift_success and place_success:
        print("\n==================================================")
        print(" PASS: Complete 9-step Pick-and-Place Succeeded! ")
        print("==================================================")
        return True
    else:
        print("\nFAIL: Pick-and-place sequence did not complete successfully.")
        return False

if __name__ == '__main__':
    ok = run_pick_and_place()
    sys.exit(0 if ok else 1)

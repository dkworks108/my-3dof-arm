import os
import sys
import xml.etree.ElementTree as ET

# find urdf file
urdf_file = os.path.join(os.path.dirname(__file__), 'urdf', 'robot.urdf')

if not os.path.exists(urdf_file):
    print(f"FAIL: File not found: {urdf_file}")
    sys.exit(1)

try:
    tree = ET.parse(urdf_file)
    root = tree.getroot()
except Exception as e:
    print(f"FAIL: XML parse error: {e}")
    sys.exit(1)

# get links and joints
links = [link.attrib['name'] for link in root.findall('link')]
joints = root.findall('joint')

print(f"Total links: {len(links)}")
print(f"Total joints: {len(joints)}")

link_set = set(links)
movable_joints = []
arm_joints = []
gripper_joints = []
has_tool_link = "tool_link" in link_set
all_joints_valid = True

joint_axes = {}

for j in joints:
    name = j.attrib.get('name')
    jtype = j.attrib.get('type')
    parent = j.find('parent').attrib.get('link') if j.find('parent') is not None else None
    child = j.find('child').attrib.get('link') if j.find('child') is not None else None
    axis_elem = j.find('axis')
    axis_xyz = axis_elem.attrib.get('xyz') if axis_elem is not None else None
    joint_axes[name] = axis_xyz

    print(f" - Joint: {name} | Type: {jtype} | Axis: {axis_xyz} | Parent: {parent} -> Child: {child}")

    if parent not in link_set or child not in link_set:
        print(f"   ERROR: Joint {name} links missing: parent={parent}, child={child}")
        all_joints_valid = False

    if jtype in ['revolute', 'continuous', 'prismatic']:
        movable_joints.append(name)
        if jtype == 'revolute':
            arm_joints.append(name)
        elif jtype == 'prismatic':
            gripper_joints.append(name)

print(f"Movable joints count: {len(movable_joints)} (3 revolute arm + 2 prismatic gripper)")
print(f"Arm revolute joints: {arm_joints}")
print(f"Gripper prismatic joints: {gripper_joints}")

# Check shoulder and elbow alignment: axes must both be [0, 1, 0] or parallel
j2_axis = [float(v) for v in joint_axes.get('joint2', '').split()]
j3_axis = [float(v) for v in joint_axes.get('joint3', '').split()]
parallel_pitch = (len(j2_axis) == 3 and len(j3_axis) == 3 and j2_axis == j3_axis == [0.0, 1.0, 0.0])
print(f"Shoulder (J2) and Elbow (J3) parallel pitch alignment: {parallel_pitch}")

# checks
if (all_joints_valid and len(arm_joints) == 3 and len(gripper_joints) == 2 
    and parallel_pitch and has_tool_link):
    print("PASS: 3-DOF Arm + Parallel Gripper URDF verification passed!")
    sys.exit(0)
else:
    print("FAIL: Verification failed!")
    sys.exit(1)

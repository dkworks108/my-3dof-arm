# 3dof arm

my first urdf robot. just testing how links and joints connect together.

links:
- base_link (cylinder)
- link1, link2, link3
- tool_link

joints are all revolute around z, y, x.
remember: z is up in ros!

to view in rviz:
ros2 launch launch/display.launch.py

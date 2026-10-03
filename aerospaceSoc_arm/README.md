# Aerospace Society Arm

The Aberdeen University Aerospace Society started a side project to create a robot arm. The plan is for an initial arm with basic functionality which can be adapted to various more specific use cases by members as desired. The project is currently on hold while the society focuses time and resources on remediating issues with its main UAV project.

Mechanical design was done by Ben Pickering, electronics, PCB design and firmware by Dave Riley.

3D CAD rendering of the arm:
![A 3D CAD rendering of the robot arm.](RobotArm_3D.png)

Note that the design was updated to replace the ATTiny1616 MCU with a Pi Pico development board. This was done because because a potential future contributor wanted to use the RP2040/2350 for a specific project, and hand soldering these MCUs appeared very challenging.

[PCB schematic](RobotArmPCB_schematic_v0.2_DRAFT.pdf)

3D views of the PCB (board will be resized based on final design of mounting points in controller enclosure):
![3D rendering of the PCB, top view](RobotArmPCB_3D-A_v0.2_DRAFT.png)
![3D rendering of the PCB, angled view](RobotArmPCB_3D-B_v0.2_DRAFT.png)

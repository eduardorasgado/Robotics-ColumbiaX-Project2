First transformation (class example):\
Use of quaternions and transforms into rotation matrix, forecasting T1 and T2.
We have 3 coordinate frames: world, T1 and T2.

For the example:\
	1) rosrun rviz rviz\
	2)rosrun project2 kinematics_node.py

Main project\
Transformation applying to 3 lements in space:\
	a) A cylinder
	b) A cube(robot)
	c) A camera

For running the simulation:\
	1) roscore
	2) rosrun rviz rviz
	3) Set TF and Markers in Rviz
	4) rosrun project2 marker_publisher
	5) rosrun project2 solution.py\

We use dot and cross product and matrix-vector multiplication to obtain T3-camera orientation given T2 and T1.
Given th object x,y,z localization exactly.

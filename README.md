# Ros2-Vision-Pick-And-Place
I built a ROS 2-based robot arm that uses inverse kinematics and Raspberry Pi camera for vision-guided object pick and place.

#### Video Link:
https://youtu.be/Kb_mZ1KI4Cs  

#### Code Layout:
The demo video is largely associated with the code in my_cpp_pkg/src, this location contains my ROS pipeline.  The code for the arduino program is in Object_Detection_With_Pi.ino. The inverse kinematics math that is located in July 2026 IK.txt, is also inside of my ROS pipeline in IK_node.py.

#### System Architecture:
<img width="1694" height="929" alt="a2e0fc59-aaf7-4219-8efe-ed4d1df5b0a0" src="https://github.com/user-attachments/assets/06a93221-e3bb-40a0-bc91-e12937bd42eb" />

#### Hardware Used: 
Rasberry Pi, Rasberry Pi Camera Module, Camera Ribbon, Camera Articulating Magic Arm, SD Card, Li-ion Battery Pack, 4DOF Robotic Arm Kit to assemble, LM2596 Buck Converter, Dupont Jumper Wires, 1-to-5 Wire Splice Connectors, 4 MG90S Micro Servo's, Arduino Uno R3, DC Barrel Jack Connectors, Inline DC Power Switch

#### Electrical Wiring Diagram  
<img width="1643" height="957" alt="5c0ea36a-96cb-41e6-8110-33c109ec0224" src="https://github.com/user-attachments/assets/2619ae88-4990-4135-87e2-52cc41459a46" />

#### Camera Location:  
<img width="300" height="400" alt="IMG_0553 copy" src="https://github.com/user-attachments/assets/02252125-4dfc-4fd9-9648-e0487cdd415a" />  

#### Scene View:  
<img width="633" height="468" alt="Screenshot 2026-07-11 at 12 08 10 AM" src="https://github.com/user-attachments/assets/75a38706-59c9-44b4-b0d8-ac6f5f12a059" />  

#### Robot Picture:
<img width="600" height="800" alt="IMG_0551" src="https://github.com/user-attachments/assets/d3de69ee-e0f5-40e1-954c-38b1a93dee25" />

#### Computer Vision Analysis:
Pixel-to-Robot Coordinate

The vision node (Color_Detector_Node.py) detects the center of each red object in image pixel coordinates (u,v). Four calibration points were manually measured by placing a block at known robot coordinates (x,y,z) and recording the corresponding camera pixels. These point correspondences were used to compute an affine transformation that converts any detected pixel location into the robot's coordinate frame.

An affine transformation is used instead of a simple proportional scale because the camera is mounted at an angle above the workspace. Due to perspective and viewing geometry, equal distances in image pixels do not correspond to equal distances on the tabletop across the entire field of view. The affine model compensates for these non-uniform scaling and skew effects, allowing the manipulator to accurately reach objects throughout the workspace.

robot_x = ax * u + bx * v + cx  
robot_y = ay * u + by * v + cy  
robot_z = tabletop_z  

#### Inverse Kinematics Analysis:
Closed-Form Inverse Kinematics

The robotic arm computes joint angles directly from a desired Cartesian target (x,y,z) using a closed-form inverse kinematics solution. Rather than using iterative numerical methods (such as Jacobian-based IK), the algorithm exploits the arm's simple three-degree-of-freedom geometry. The base angle is computed with atan2(), the elbow angle is solved using the Law of Cosines, and the shoulder angle is determined from right-triangle trigonometry. After the inverse kinematics solution is obtained, calibrated servo offsets convert the ideal joint angles into physical servo commands that account for the arm's mechanical assembly. Because the manipulator has a simple kinematic structure, this approach does not require Denavit–Hartenberg parameters or homogeneous transformation matrices, resulting in an efficient, exact solution for reachable targets.  

<img width="1536" height="1024" alt="9620da34-e03c-4d55-a7a1-bf02a158416a" src="https://github.com/user-attachments/assets/7be8d1c4-b96c-47f7-a078-e1623d91aa5e" />




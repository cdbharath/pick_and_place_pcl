Vision based Robotic Manipulation - WPI course project

Refer my [Franka Panda](https://github.com/cdbharath/franka_panda "Franka Panda") repository for the manipulation pipeline used in this project. 

# Point cloud processing based grasp synthesis

1. Initial setup:
<p align="left">
<img width="460" height="300" src="./media/pc_setup.png">
</p>

2. Point cloud extracted from eye in hand Kinect sensor:
<p align="left">
<img width="460" height="300" src="./media/pc.png">
</p>

3. Result of RANSAC over the point cloud:
<p align="left">
<img width="460" height="300" src="./media/ransac.png">
</p>

4. Extracting only the outliers from RANSAC:
<p align="left">
<img width="460" height="300" src="./media/ransac_outliers.png">
</p>

5. Point cloud of the pudding box object after passthrough filter:
<p align="left">
<img width="460" height="300" src="./media/pc.png">
</p>

6. Franka Panda grasping the object using the resulting point cloud:
<p align="left">
<img width="460" height="300" src="./media/grasping.png">
</p>

# Visual servoing of a RR robot

1. Initial setup:
<p align="left">
<img width="460" height="300" src="./media/vs_setup.png">
</p>

2. Initial camera output from the RR manipulator:
<p align="left">
<img width="460" height="300" src="./media/initial.png">
</p>

3. Camera output after the RR manipulator end effector reaches the setpoint:
<p align="left">
<img width="460" height="300" src="./media/final.png">
</p>

4. Response of the controller:
<p align="left">
<img width="460" height="300" src="./media/vs_result.png">
</p>

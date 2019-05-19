# A ball chaser

This is a simple differential-drive robot haued in single floor building simulated in Gazebo.
The robot is controlled by ROS. The robot is equipped with camera and Hokuyo laser scaner. 

## Nodes
Two nodes are essential for **white** ball chasing:
* process image - which subsribes images form camera and finds a **white** blob on it. This node also calls drive_bot service to steer robot
* drive_bot - it is a service to drive robot (it could move froward, backward or turn around its veritcal axis)

## License
The contents of this repository are covered under the [MIT License](./LICENSE.txt)


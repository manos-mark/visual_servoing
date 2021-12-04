# Visual Servoing Project
## Introduction
The purpose of this project is to implement an end to end visual sevoing project using a fish-eye camera and a Turtlubot3 with ROS. 
1. The first objective is to move the robot from the current position to the target position. To specify those positions and their poses we use two Aruco markers. 
2. The second task for the robot is to avoid obstacles that are specified using red color, we implemented this task using A-star path finding algorithm.
3. Finally, when the robot is at the target position we have to do "parking", that means that the targets position pose should be the same as the robot's pose.

Following on this report we will analyse the mathematical theoritical background and the code implementation we implemented.

## What is ROS?
The Robot Operating System (ROS) is an open source middleware which contains a set of libraries,softwares and tools that are used to facilitate the development of robotic applications. There is a plethora of features from sensor drivers to state-of-the-art algorithms. As middleware, it contains characteristics of both software and hardware, hencee, it is able to perform various actions like hardware abstraction and low level control.
Until now, different version of ROS exists with some crusial differences, so for compatibility reasons we are using the Melodic release.

## Robot used for this scenario
### Turtlebot Description
For the project, the mobile robot used is a Turtlebot3 Burger. The Turtlebot3 is a compact, modular and programmable mobile robot. It uses ROS and it is able to create multiple application for training research and development.

# First Objective - Move from current to target position

## 1. Camera Calibration 
Camera calibration is an integral part of this project. For this phase, the project uses the [camera_calibration](http://wiki.ros.org/camera_calibration) package which allows easy calibration of monocular cameras using a checkerboard calibration target. The packagess uses OpenCV library which contains the camera calibration method.  
![Checkerboard](images/checkerboard.jpg)

#### **Intrinsic calibration**  
As we aforementioned, it uses the [camera_calibration](http://wiki.ros.org/camera_calibration) package. This package allows easy calibration of monocular or stero cameras. The checkerboard was the tool in order to fix the *Radial Distortion* of the acquired image. *Radial or Barrel Distortion* can be presented as:

<p align="center"><img src="https://render.githubusercontent.com/render/math?math=\begin{aligned}x_{distorted}=x(1%2Bk_{1}r^2%2Bk_{2}r^4%2Bk_{3}r^6)\end{aligned}"></p>

<p align="center"><img src="https://render.githubusercontent.com/render/math?math=\begin{aligned}y_{distorted}=y(1%2Bk_{1}r^2%2Bk_{2}r^4%2Bk_{3}r^6)\end{aligned}"></p>


In the same manner, tangenial distortion occurs because the imaging-taking lense is not aligned perfectly parallel to the imaging plane. So, some images look nearer than expexted. The amount of tangenial distortion can be presented as below:  

<p align="center"><img src="https://render.githubusercontent.com/render/math?math=\begin{aligned}x_{distorted}=x%2B[2p_{1}xy%2Bp_{2}(r^2%2B2x^2)]\end{aligned}"></p>

<p align="center"><img src="https://render.githubusercontent.com/render/math?math=\begin{aligned}y_{distorted}=y%2B[p_{1}(r^2%2B2x^2)%2B2p_{2}xy]\end{aligned}"></p>

According to the equation above, we can find the five paremeters, known as distortion coefficients


<p align="center"><img src="https://render.githubusercontent.com/render/math?math=DistortionCoefficients=(k_{1},k_{2},p_{1},p_{2},k_{3})"></p>

Furthermore, **intrinsic parameters** allows a mapping between camera coordinates and pixel coordinates in the image frame. They include information like local length <img src="https://render.githubusercontent.com/render/math?math=(f_{x},f_{y})">, and optical center <img src="https://render.githubusercontent.com/render/math?math=(C_{x}, C_{y})">. This parameters can be expressed in camera matrix:

<p align="center"><img src="https://render.githubusercontent.com/render/math?math=%5Cbegin%7Baligned%7D%0Acamera%20matrix%20%3D%0A%5Cbegin%7Bbmatrix%7D%0A%20%20%20f_%7Bx%7D%20%26%200%20%26%20C_%7Bx%7D%5C%5C%0A%20%20%200%20%26%20f_%7By%7D%20%26%20C_%7By%7D%5C%5C%0A%20%20%200%20%26%200%20%26%201%20%0A%5Cend%7Bbmatrix%7D%20%0A%5Cend%7Baligned%7D%0A"></p>

## 2. Receive image
The next step is to receive image frames by subscribing to the ROS topic "/camera/image_raw" and convert it to numpy array. Then we need to crop the image according to our needs, by specifing the window that we need to work on. Afterwards we undistord the received image using the camera matrix and the distortion coefficients received on the previous step.

## 3. Detect Markers
Using the OpenCV library we can detect two Aruco markers that are placed on the top of the robot, one marker for the current and one for the target position. We only have to call the function `cv2.detectMarkers` from wich we receive the corners of each marker and we can move on to the pose estimation.

## 4. Pose estimation
 Next step is to estimate the current and target poses, simple by calling the function `cv2.estimatePoseSingleMarkers` in the [pose estimation module](https://github.com/manoskout/visual_servoing/blob/master/scripts/pose_estimation.py). From which we receive two vectors for each marker, one translational vector `[x, y, z]` and one rotational vector `[x, y, z]`. Using the ROS publisher we sent those vectors to the robot controller, who is responsible to translate those matrices in a way that the robot should be able to move. This is implemented with the ROS publisher 
 ```python 
 rospy.Publisher('current_position', Pose_estimation_vectors, queue_size=10)
 ```
 where the *current_position* is the ROS topic that the controller will subscribe to fetch the custom message *Pose_estimation_vectors* tha we created, in order to send those vectors.

The structure of this message is the following:          
`geometry_msgs/Vector3 rotational`  
`geometry_msgs/Vector3 translational`

Here is the image with both of the poses:  
<p align="center"><img src=images/poses.png></p>

## 5. Controller - Convert rotational and translational matrices to homogeneous matrices
The homogeneous transformation is encoded by the extrinsic parameters `R` and `t` and represents the change of basis from world coordinate system `w` to the camera coordinate sytem `c`. Thus, given the representation of the point `P` in world coordinates, `Pw`, we obtain `P`'s representation in the camera coordinate system, `Pc`, by:

<p align="center"><img src=images/camera_points.png> </p>

This homogeneous transformation is composed out of `R`, a 3-by-3 rotation matrix, and `t`, a 3-by-1 translation vector:

<p align="center"><img src=images/homogeneous_matrix.png></p>

Combining the projective transformation and the homogeneous transformation, we obtain the projective transformation that maps 3D points in world coordinates into 2D points in the image plane and in normalized camera coordinates:

<p align="center"><img src=images/homogeneous_matrix2.png></p>

The [controller module](https://github.com/manoskout/visual_servoing/blob/master/scripts/controller.py) have to convert the rotational vector into rotational matrix using the Rodrigues transformation with the OpenCV function:   
```python
rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
```  

Then we stack the rotational matrix horizontally with the transformational vector and append at the end the row *[0, 0, 0, 1]* in order to receive the homogeneous matrix. 

## 6. Calculate alpha (α) angle and distance rho (ρ) between current and target positions
<p align="center">![Angles](images/angles.png)</p>

The homogeneous matrices we obtain from the previous step, describe the position of each position in respect with the camera's frame. We need to combine them in order to receive the position from one position in respect to the other position. To do that, we multiply the inverse of the current homogeneous matrix with the target homogeneous matrix to receive the combined homogeneous matrix (t):  
```python
t = np.matmul(np.linalg.inv(self.curr_homogeneous_matrix), self.target_homogeneous_matrix)
```

<p align="center"><img src=images/combined_homogeneous_matrix.jpg></p>

Following, we need to calculate the angle (alpha) and the distance (rho) that the robot should move:  

We get the dx, dy from the matrix above:  
```python
dx = t[0][3]
dy = t[1][3]
```

Then we calculate the arctan using dx, dy:  
```python 
self.beta = math.atan2(dy, dx)
```

And finally we get the distance to target (rho) by applying the Euclidean distance:  
```python
self.rho = math.sqrt(math.pow(dy, 2) + math.pow(dx, 2))
```

## 7. Fix the angle and move the robot to target
Now we fix the beta angle by publishing to the ROS topic `cmd_vel` only angular velocity, and when the angle is correct according to the target angle we publish again to the same topic only linear velocity until the distance to target is near to zero. We use a proportional controller, so the velocities that are given to the robot are multiplied by two constants, one for angular and one for linear velocity.

# Second Objective - Avoid obstacles
## 1. Obstacle detection
The [obstacle detection module](https://github.com/manoskout/visual_servoing/blob/master/scripts/obstacle_detection.py) is using the input image and slice it into boxes of size equal to the robot size. This way we have an array with all the possible moves that the robot can achive. To distinguish the obstacles, that mean if there is an obstacle -a red box- in the image the robot should not be able to move there. 

Then we iterate for every box of the image and convert the box to HSV. Next we mask the image if the box contains any pixels in the range of red color, and apply the bitwise mask to the output. If the box contains red pixels we assume that it is an obstacle.

The output of this step is an one directional array with lenght equals to the number of the boxes wich contains zero's (when there is no obstacle) and one's (where there is an obstacle).

<p align="center"><img src=images/obstacles.png></p>

## 2. Find shortest path using A-star Algorithm
Using the obstacles map array of the previous step we implement the [Path planning module](https://github.com/manoskout/visual_servoing/blob/master/scripts/path_planning.py) to receive the shortest path the robot should move to go into the target faster. 

This is a graph based algorithm which is using an heuristic method for better performance. The core of this is f = g + h, where:
- F is the total cost  
- G is the distance between the current node and the start node.  
- H is the heuristic — estimated distance from the current node to the end node.

Read the following [article](https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2) for more informations.

## 3. Estimate middle-point pose
Shortest path contains some middlepoints that the robot should move to find the target position faster without any collision with the obstacles. 

Here, we face a problem because we only have the indexes of those middlepoints. So, we decided to convert those indexes to pixels according to our boxed frame. Using their corners we calculate their poses the same way we calculate the pose for the aruco markers with the function: `cv2.aruco.estimatePoseSingleMarkers`.   

<p align="center"><img src=images/midlepoint_poses.png></p>

## 4. Draw shortest path
Then, we itterate throught the obstacles map and draw on the image each middle point of the shortest path we calculated in the previous steps.

This is the final map, where blue zeros specify that there is a valid movement for the robot, light blue X specify there is an obstacle, orange circles specify the shortest path, bold white S and G specify the starting and goal point of the path respectively.  
<p align="center"><img src=images/final_map.png></p>

## 5. Move on each middle point
Our controller receives the current pose vectors each time the `on_receive_image` callback is executed. It calculates the homogeneous matrix as explaind earlier and save it as class variable `self.current_homogeneous_matrix`.

For each middle point the controller receives the same vectors and update the list `target_position_path` class variable. When this list has no values the robot is not moving, as it doesn't know where to go. When it receive some values it calculate the homogeneous matrix for this specific middlepoint and save it as the class variable `self.target_homogeneous_matrix`.  

The robot is able to move now on every midlepoint specified in the `target_position_path` list by fixing the angle and then moving to the middlepoint. When the robot is approaching the middlepoint and the distance error is small we get the next element (midlepoint) of the list and continue moving until the list is empty!


# Third Objective - Parking
## 1. Calculate beta (β) Euler angle
On the controller `move_robot` there are two loops for the implementation of the previous steps (fix angle, move forward). For the purposes of parking we need to add another loop that will be executed only when the robot is on the target position, and the objective of this loop is to fix the final angle in respect with the specified target pose.

The additinal code we need is simple:  
```python
rotational_matrix = np.array([
            [t[0][0], t[0][1], t[0][2]],
            [t[1][0], t[1][1], t[1][2]],
            [t[2][0], t[2][1], t[2][2]],
        ])
                
r = R.from_matrix(rotational_matrix)
self.beta = r.as_euler('XYZ', degrees=False)[2]
```

We convert the rotational matrix to Euler angles, we receive a vector `[x,y,z]` and get the third value because we only need the z angle, which we save as the class variable beta.

## 2. Move the robot to fix the angle error
Finally, we publish the desired angular velocity and the robot fix the beta angle!

# Demo 
Execute Roscore: 
```bash
roscore
```

Launch the camera
```bash
roslaunch ueye_cam rgb8.launch
```

Remote monitor and control, for more details [click here](http://wiki.ros.org/robotican/Tutorials/Remote%20monitoring%20and%20control):
```bash
ssh ubuntu@192.168.0.200
```

On the turtlebot run, for more details [click here](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/):
```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

On the remote computer launch our implementation:
```bash 
roslaunch visual_servoing visual_servoing.launch
```

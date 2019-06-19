

# How to fuse odometry & IMU with *robot_localization*

## What is the *robot_localization* package

It is a package for mixing different sources of odometry into a more stable one.

![enter image description here](https://github.com/rwbot/cheatsheets/blob/master/img/robot_localization_graph.png?raw=true)


## Configuring the *robot_localization* package


Then create a launch file named `start_filter.launch` with the following content:


```xml
<launch> 
    <!-- Run the EKF Localization node -->
    <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization">
        <rosparam command="load" file="$(find PKG_NAME)/config/ekf_localization.yaml"/>
    </node>
</launch>
```

Basically, what we are doing here is to launch the ROS program named **ekf_localization_node** (which uses the Extended Kalman Filter), from the **robot_localization** package with a specific configuration file named **ekf_localization.yaml**.

### The configuration file

Now, inside this folder, create the configuration file, named **ekf_localization.yaml**. Inside this file, you will place the following configuration. 


```yaml
#Configuation for robot odometry EKF
frequency: 50
    
two_d_mode: true
    
publish_tf: false

# Complete the frames section 
odom_frame: odom
base_link_frame: base_link
world_frame: odom
map_frame: odom

# Complete the odom0 configuration
odom0: /odom0
odom0_config: [false, false, false,
               false, false, false,
               true, true, false,
               false, false, true,
               false, false, false,]
odom0_differential: false

# Complete the imu0 configuration
imu0: /imu/data
imu0_config: [false, false, false,
               false, false, false,
               false, false, false,
               false, false, true,
               true, false, false,]
imu0_differential: false

process_noise_covariance": [0.05, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0.05, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0.06, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0.03, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0.03, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0.06, 0,     0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0.025, 0,     0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0.025, 0,    0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0.04, 0,    0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01, 0,    0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01, 0,    0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.02, 0,    0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01, 0,    0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01, 0,
                                              0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.015]


initial_estimate_covariance: [1e-9, 0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    1e-9, 0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    1e-9, 0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    1e-9, 0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    1e-9, 0,    0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    1e-9, 0,    0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    1e-9, 0,    0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    1e-9, 0,    0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    1e-9, 0,     0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    1e-9,  0,     0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     1e-9,  0,     0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     1e-9,  0,    0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     1e-9, 0,    0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    1e-9, 0,
                                                      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    0,    1e-9]
```

### The reference frames

* **base_link_frame**: It is the frame that is in the robot itself, to which any sensor can be referenced. It is usually located in the center of the robot. It travels with it.


* **odom_frame**: It is the frame that is used to report the odometry.


* **map_frame**: It is the frame that is used to report a global position from a system that knows where the robot is. For instance an AMCL system. If you are not using any external Localization system, the this can be ignored.


* **world_frame**: It is the frame that references which one of the two previous frames is going to be used to get the absolute coordinates of the robot in the world.

In our case that would be:


```yaml
base_link_frame: base_link
odom_frame: odom
world_frame: odom
```

### Adding sensors to fuse

Any sensor that produces messages in any of these formats, can be fed to the robot_localization package to estimate the robot position.

* <a href="http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html" target="_blank">nav_msgs/Odometry</a>

* <a href="http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html" target="_blank">sensor_msgs/Imu</a>

* <a href="http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html" target="_blank">geometry_msgs/PoseWithCovarianceStamped</a>

* <a href="http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html" target="_blank">geometry_msgs/TwistWithCovarianceStamped</a>

And, most importantly, you can have many of each sensor. This means that, for instance, you can have the odometry provided by two different sensors (wheel encoders and visual odometry), or two different Inertial Measurement Units.

#### a. First indicate the topic of the sensor

In our case are:

* /noisy_odom
* /imu/data

#### b. Configure the variables matrix

Now, from each of the input sensors, we must specify which components of their messages are going to be merged (fused) in the Kalman Filter to compute the final state estimation. To specify this, you must fill a 3x5 value matrix. The matrix means the following:


```yaml
[  X,        Y,       Z
  roll,    pitch,    yaw
  X/dt,     Y/dt,    Z/dt
 roll/dt, pitch/dt, yaw/dt
  X/dt2,    Y/dt2,   Z/dt2]
```

The above values mean the following:

* **X, Y, Z**: These are the [x,y,z] coordinates of the robot.
* **roll, pitch, yaw**: These are the rpy axis, which specify the orientation of the robot.
* **X/dt, Y/dt, Z/dt**: These are the velocities of the robot.
* **roll/dt, pitch/dt, yaw/dt**: These are the angular velocities of the robot
* **X/$dt^2$, Y/$dt^2$, Z/$dt^2$**: These are the linear accelerations of the robot:

So, in this case, the values we are taking into account for the Kalman Filter are, for the odometry data:

* linear velocities in X and Y, and angular velocity in Z.


```yaml
odom0_config: [false, false, false
 false, false, false
 true,  true,  false
 false, false, true
 false, false, false,]
```

And for the IMU data:

* yaw(orientation), angular velocity in Z and linear acceleration in X. 

#### c. Covariance matrices

Use the default values

* **process_noise_covariance**: This parameter is used to model uncertainty in the prediction stage of the filtering algorithms. WHAT!!?? Basically, it's used to improve the results produced by the filter. The values on the diagonals are the variances for the state vector, which include pose, then velocities, then linear acceleration. It is not mandatory to set, but you will achieve superior results by tunning it. Anyways, unless you are an expert on the matter, it's not easy to set at all. So, the best option in that case would be to test different values, and see how they improve or decrease the results.


* **initial_state_covariance**: The estimate covariance defines the error in the current state estimate. This parameter allows to set the initial value for the matrix, which will affect how quickly the filter converges.
Here's the rule you should follow: if you are measuring a variable, make the diagonal value in initial_estimate_covariance larger than that measurement's covariance. So, for example, if your measurement's covariace value for the variable in question is 1e-6, make the initial_estimate_covariance diagonal value 1e-3 or something like that.

## Launch *robot_localization*

Now it is time to launch the *robot_localization* with your configuration:


```
$ roslaunch PKG_NAME start_filter.launch
```

Check now in Rviz that the odometry is now filtered and stabilized. You can find the result in topic:

*/odom/filtered*

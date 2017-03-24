# SRRG New World Calibration Ros Package #

### Dependencies ###

* eigen3
* [srrg_nw_calibration](https://gitlab.com/srrg-software/srrg_nw_calibration)

### Installation ###
Standard ros package installation

    cd ROS-PACKAGES-DIR
    git clone https://gitlab.com/srrg-software/srrg_nw_calibration_ros.git
    cd ROS-WORKSPACE/src
    ln -s ROS-PACKAGES-DIR/srrg_nw_calibration_ros .
    cd ..
    catkin_make --pkg srrg_nw_calibration_ros

### Example ###
The node srrg_nw_calibration_ros_node provides an help by typing '-h'. 
Usual command

    rosrun srrg_nw_calibration_ros new_world_calibration_ros_node -guess init_guess.txt

An initial guess file has to be provided. An example of this file can be found in the folder initial_guess.
The init_guess.txt file contains the list of sensors to be calibrated, the relative tf to be red,
and the intial set of parameters the solver will use.

### Testing Dataset ###

A bag (with only tf recorder) of a Hokuyo URG scanner, and 3 Asus Xtion mounted on a custom platform while performing an eight-shaped path.
The tf attached to the Laser Scanner is obtained by scan matching, while the 3 tfs attached to the cameras have been recorder with [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) properly modified to publish the transforms.

Download: [dataset 8-shape-path](https://drive.google.com/file/d/0BxhfDMgREiwXMjY2X29OWjlEbUU/view?usp=sharing)

Run the bag and test the calibration:

    rosrun srrg_nw_calibration_ros new_world_calibration_ros_node -guess init_guess.txt

### Unsupervised Calibration ###
The node srrg_nw_calibration_ros_auto_node provides an autonomous way to calibrate your platform.

    rosrun srrg_nw_calibration_ros new_world_calibration_ros_auto_node -guess init_guess.txt


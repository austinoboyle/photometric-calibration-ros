# Online Photometric Calibration - ROS Wrapper

## Usage

1. `catkin_make #or catkin build`
2. `source ./devel/setup.bash`

### Run Range Correction Node:

`rosrun range_correction correct`

### Run Photometric Calibration Node

`rosrun online_photometric_calibration online_pcalib_demo`

**NOTE**: You won't see anything unless you have a ROS publishers publishing on the
image topics specified in the app's settings. These by default is
/SwissRanger/intensity/image_raw and /SwissRanger/distance/image_raw for the
intensity and distance images respectively.

-   Test data located at /data.
-   Run sample data using `rosbag play --loop sr_test_00.bag`

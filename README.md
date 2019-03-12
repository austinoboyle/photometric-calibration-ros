# Online Photometric Calibration - ROS Wrapper

## Usage

1. `catkin_make #or catkin build`
2. `source ./devel/setup.bash`
3. rosrun online_photometric_calibration online_pcalib_demo

**NOTE**: You won't see anything unless you have a ROS publishers publishing on the
image topics specified in the app's settings. These by default is
/SwissRanger/intensity/image_raw and /SwissRanger/distance/image_raw for the
intensity and distance images respectively.

-   You can run a bag from
    https://github.com/austinoboyle/swissranger-image-correction/tree/master/data
    using `rosbag play --loop sr_test_00.bag` to run some sample data through the algorithm.

#!/bin/bash
rosbag record /SwissRanger/distance/image_raw /SwissRanger/distance/image_raw16 /SwissRanger/intensity/image_raw /SwissRanger/parameter_updates /SwissRanger/parameter_descriptions /SwissRanger/pointcloud2_raw --duration=$1 -O $2

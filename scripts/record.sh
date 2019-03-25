#!/bin/bash
rosbag record /SwissRanger/intensity/image_raw16 /SwissRanger/distance/image_raw16 /SwissRanger/exposure /SwissRanger/pointcloud2_raw --duration=$1 -O $2

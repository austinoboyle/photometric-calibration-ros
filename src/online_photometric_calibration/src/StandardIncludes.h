//
//  OpencvIncludes.h
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

/**
 * Defines all kinds of standard includes that are used in most of the classes
 */

#ifndef OnlineCalibration_StandardIncludes_h
#define OnlineCalibration_StandardIncludes_h

/**
 * Opencv includes
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * Standard library includes
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <vector>
#include <fstream>

/**
 * ROS includes
 */
// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

#endif

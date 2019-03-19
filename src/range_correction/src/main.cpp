// #define EIGEN_NO_STATIC_ASSERT 1

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <boost/foreach.hpp>

// NOTE: Needed to move /eigen3/Eigen to /Eigen and /pcl1.7/pcl -> /pcl in
// /usr/include for these to work.  Some of the include paths are hard-coded.
// May be able to fix this with a CMAkelist entry
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace sensor_msgs;
using namespace message_filters;

// void cloud_cb(const PointCloud2ConstPtr &input, ros::Publisher &pub)
// {
//     int X_OFFSET = 0;
//     int Y_OFFSET = 4;
//     int Z_OFFSET = 8;
//     int I_OFFSET = 12;
//     int C_OFFSET = 16;
//     for (int i = 1; i < input->height - 1; i++)
//     {
//         for (int j = 1; j < input->width - 1; j++)
//         {
//             /**
//              * Get the points and mean point of a 3x3 region centered on the
//              * point at i, j.  This is used to calculate the normal vector
//              */

//             std::vector<Eigen::Vector3d> xyz_pts;
//             Eigen::Vector3d mean = Eigen::Vector3d::Zero();
//             for (int row = i - 1; row <= i + 1; row++)
//             {
//                 for (int col = j - 1; col <= j + 1; col++)
//                 {
//                     // This is used to get the value of the
//                     float *x = (float *)&input->data.at(X_OFFSET + (col + row * input->width) * input->point_step);
//                     float *y = (float *)&input->data.at(Y_OFFSET + (col + row * input->width) * input->point_step);
//                     float *z = (float *)&input->data.at(Z_OFFSET + (col + row * input->width) * input->point_step);
//                     Eigen::Vector3d tmp = Eigen::Vector3d::Zero();
//                     tmp << (double)x[0], (double)y[0], (double)z[0];
//                     xyz_pts.push_back(tmp);
//                     mean += tmp;
//                 }
//             }
//             mean /= 9;

//             Eigen::Matrix3d N = Eigen::Matrix3d::Zero();

//             // Calculate the N matrix based on equation A.1 in Hewitt's thesis
//             BOOST_FOREACH (Eigen::Vector3d v, xyz_pts)
//             {
//                 Eigen::Vector3d diff = v - mean;
//                 N += diff * diff.transpose();
//             }
//             Eigen::EigenSolver<Eigen::Matrix3d> es(N);
//             Eigen::Vector3cd evals = es.eigenvalues();
//             Eigen::Matrix3cd evecs = es.eigenvectors();

//             // std::cout << "VEC 1" << es.eigenvectors().col(0) << std::endl;
//             int min_index = 0;
//             int min_val = evals(0).real();
//             for (int i = 1; i < 3; i++)
//             {
//                 if (evals(i).real() < min_val)
//                 {
//                     min_index = i;
//                     min_val = evals(i).real();
//                 }
//             }
//             Eigen::Vector3d norm = evecs.col(min_index).real();
//             double dot_product = norm.dot(xyz_pts[4]) / xyz_pts[4].norm(); //the central point is the 5th point
//             // std::cout << "DOT_PRODUCT: " << dot_product << std::endl;
//         }
//     }
//     std::cout << "PUB" << std::endl;
//     std_msgs::String msg;
//     msg.data = "TEST";
//     pub.publish(msg);
// }

void callback(const ImageConstPtr &intensity, const ImageConstPtr &distance, image_transport::Publisher &pub)
{
    cv_bridge::CvImagePtr distance_ptr;
    cv_bridge::CvImagePtr intensity_ptr;
    cv_bridge::CvImagePtr corrected;
    try
    {
        intensity_ptr = cv_bridge::toCvCopy(intensity, image_encodings::MONO16);
        distance_ptr = cv_bridge::toCvCopy(distance, image_encodings::MONO16);
        corrected = cv_bridge::toCvCopy(distance, image_encodings::MONO16);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    intensity_ptr->image.convertTo(intensity_ptr->image, CV_64FC1);
    distance_ptr->image.convertTo(distance_ptr->image, CV_64FC1);
    corrected->image.convertTo(corrected->image, CV_64FC1);

    corrected->image = intensity_ptr->image.mul(distance_ptr->image.mul(distance_ptr->image));
    corrected->image.convertTo(corrected->image, CV_16UC1, 1.0 / 255.0);
    std::cout << corrected->encoding << std::endl
              << corrected->image << std::endl;

    // corrected->image /= 1 << 48;
    pub.publish(corrected->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/image_converter/converted", 1);
    // test_pub = nh.advertise<std_msgs::String>("image_converter/test_cpp", 1000);

    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

    Subscriber<Image> intensity_sub(nh, "/SwissRanger/intensity/image_raw", 1);
    Subscriber<Image> distance_sub(nh, "/SwissRanger/distance/image_raw", 1);
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), intensity_sub, distance_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, image_pub));
    // ros::Publisher test_pub = nh.advertise<std_msgs::String>("image_converter/test_cpp", 1000);
    // ros::Subscriber pc_subsriber = nh.subscribe<PointCloud2>("/SwissRanger/pointcloud2_raw", 100, boost::bind(&pointcloud_cb, _1, test_pub));
    // ros::Subscriber pc_subsriber2 = nh.subscribe<PointCloud2>("/SwissRanger/pointcloud2_raw", 1000, boost::bind(&cloud_cb, _1, test_pub));
    std::cout << "image_converter node is running." << std::endl;
    ros::spin();
    return 0;
}
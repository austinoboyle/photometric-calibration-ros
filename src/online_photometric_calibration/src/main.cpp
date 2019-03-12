//
//  main.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//
#include "StandardIncludes.h"

#include "ImageReader.h"
#include "Tracker.h"
#include "RapidExposureTimeEstimator.h"
#include "Database.h"
#include "NonlinearOptimizer.h"
#include "CLI11.hpp"

using namespace std;

// Optimization thread handle
pthread_t opt_thread = 0;

// This variable indicates if currently an optimization task is running in a second thread
pthread_mutex_t g_is_optimizing_mutex;
bool g_is_optimizing = false;

struct Settings
{
    int start_image_index;   // Start image index.
    int end_image_index;     // End image index.
    int image_width;         // Image width to resize to.
    int image_height;        // Image height to resize to.
    int visualize_cnt;       // Visualize every visualize_cnt image (tracking + correction), rather slow.
    int tracker_patch_size;  // Image patch size used in tracker.
    int nr_pyramid_levels;   // Number of image pyramid levels used in tracker.
    int nr_active_features;  // Number of features maintained for each frame.
    int nr_images_rapid_exp; // Number of images for rapid exposure time estimation.
    int nr_active_frames;    // Number of frames maintained in database.
    int keyframe_spacing;    // Spacing for sampling keyframes in backend optimization.
    int min_keyframes_valid; // Minimum amount of keyframes a feature should be present to be included in optimization.
    string image_folder;     // Image folder.
    string exposure_gt_file; // Exposure times ground truth file.
    string calibration_mode; // Choose "online" or "batch".

    string intensity_topic; // ROS Topic for intensity data
    string range_topic;     // ROS Topic for range data
};

struct Globals
{
    int image_num;
    double vis_exponent;
    int optimize_cnt;
    int safe_zone_size;
};

void *run_optimization_task(void *thread_arg)
{
    std::cout << "START OPTIMIZATION" << std::endl;

    pthread_mutex_lock(&g_is_optimizing_mutex);
    g_is_optimizing = true;
    pthread_mutex_unlock(&g_is_optimizing_mutex);
    // The nonlinear optimizer contains all the optimization information
    NonlinearOptimizer *optimizer = (NonlinearOptimizer *)thread_arg;

    optimizer->fetchResponseVignetteFromDatabase();
    // Perform optimization
    optimizer->evfOptimization(false);
    optimizer->evfOptimization(false);
    optimizer->evfOptimization(false);
    // Smooth optimization data
    optimizer->smoothResponse();
    // Initialize the inverse response vector with the current inverse response estimate
    // (in order to write it to the database later + visualization)
    // better to do this here since currently the inversion is done rather inefficiently and not to slow down tracking
    optimizer->getInverseResponseRaw(optimizer->m_raw_inverse_response);
    pthread_mutex_lock(&g_is_optimizing_mutex);
    g_is_optimizing = false;
    pthread_mutex_unlock(&g_is_optimizing_mutex);

    pthread_exit(NULL);
}

// Split a string into substrings given a char delimiter
std::vector<string> split(const string &s, char delim)
{
    stringstream ss(s);
    string item;
    vector<string> tokens;
    while (getline(ss, item, delim))
    {
        tokens.push_back(item);
    }
    return tokens;
}

void image_callback(const sensor_msgs::ImageConstPtr &intensity,
                    const sensor_msgs::ImageConstPtr &distance,
                    ros::Publisher &pub,
                    Globals *g,
                    Settings *run_settings,
                    Database *database,
                    Tracker &tracker,
                    RapidExposureTimeEstimator &exposure_estimator,
                    NonlinearOptimizer &backend_optimizer)
{
    // Convert ROS Images to CV image ptrs
    cv_bridge::CvImagePtr distance_ptr;
    cv_bridge::CvImagePtr intensity_ptr;
    cv_bridge::CvImagePtr corrected;
    try
    {
        intensity_ptr = cv_bridge::toCvCopy(intensity, sensor_msgs::image_encodings::TYPE_16UC1);
        distance_ptr = cv_bridge::toCvCopy(distance, sensor_msgs::image_encodings::TYPE_16UC1);
        corrected = cv_bridge::toCvCopy(distance, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    g->image_num += 1;

    // Correct infrared image for range. Multiply by range^2
    corrected->image = intensity_ptr->image.mul(distance_ptr->image.mul(distance_ptr->image));
    cv::Mat new_image;

    // Convert coloured SwissRanger img to Grayscale for keypoint detection
    cv::cvtColor(corrected->image, corrected->image, CV_BGR2GRAY);

    // Read GT exposure time if available for this frame
    // Todo: Check if index is out of bounds, if gt exp file has not enough lines
    double gt_exp_time = -1.0;

    // If enough images are in the database, remove once all initial images for which no exposure time could be optimized
    // Since those frames will not be that good for backend optimization
    if (g->image_num == run_settings->nr_images_rapid_exp * 2 + g->safe_zone_size)
    {
        for (int ii = 0; ii < run_settings->nr_images_rapid_exp; ii++)
        {
            database->removeLastFrame();
        }
    }

    // If the database is large enough, start removing old frames
    if (g->image_num > run_settings->nr_active_frames)
    {
        database->removeLastFrame();
    }

    // Track input image (+ time the result)
    tracker.trackNewFrame(new_image, gt_exp_time);
    // Rapid exposure time estimation (+ time the result)
    double exposure_time = exposure_estimator.estimateExposureTime();
    database->m_tracked_frames.at(database->m_tracked_frames.size() - 1).m_exp_time = exposure_time;
    database->visualizeRapidExposureTimeEstimates(g->vis_exponent);

    // Remove the exposure time from the radiance estimates
    std::vector<Feature *> *features = &database->m_tracked_frames.at(database->m_tracked_frames.size() - 1).m_features;
    for (int k = 0; k < features->size(); k++)
    {
        for (int r = 0; r < features->at(k)->m_radiance_estimates.size(); r++)
        {
            features->at(k)->m_radiance_estimates.at(r) /= exposure_time;
        }
    }

    // Visualize tracking
    if (g->image_num % run_settings->visualize_cnt == 0)
        database->visualizeTracking();

    pthread_mutex_lock(&g_is_optimizing_mutex);
    bool is_optimizing = g_is_optimizing;
    pthread_mutex_unlock(&g_is_optimizing_mutex);

    // Optimization is still running, don't do anything and keep tracking
    if (is_optimizing)
    {
        return;
    }

    //optimization is currently not running
    // (1) Fetch the current optimization result and update database
    // (2) Try to extract a new optimization block and restart optimization in the background

    // Fetch the old optimization result from the optimizer, if available
    if (g->optimize_cnt > 0)
    {
        // Write the result to the database, visualize the result
        database->m_vignette_estimate.setVignetteParameters(backend_optimizer.m_vignette_estimate);
        database->m_response_estimate.setGrossbergParameterVector(backend_optimizer.m_response_estimate);
        database->m_response_estimate.setInverseResponseVector(backend_optimizer.m_raw_inverse_response);

        g->vis_exponent = backend_optimizer.visualizeOptimizationResult(backend_optimizer.m_raw_inverse_response);
    }

    // Try to fetch a new optimization block
    bool succeeded = backend_optimizer.extractOptimizationBlock();

    if (succeeded)
    {
        // TODO: reuse thread
        //start a new optimization task here
        pthread_create(&opt_thread, NULL, run_optimization_task, (void *)&backend_optimizer);
        g->optimize_cnt++;
    }

    return;
}

int main(int argc, char **argv)
{
    CLI::App app("Photometric Calibration");

    Settings run_settings;
    run_settings.start_image_index = 0;
    run_settings.end_image_index = -1;
    run_settings.image_width = 640;
    run_settings.image_height = 480;
    run_settings.visualize_cnt = 1;
    run_settings.tracker_patch_size = 3;
    run_settings.nr_pyramid_levels = 2;
    run_settings.nr_active_features = 200;
    run_settings.nr_images_rapid_exp = 15;
    run_settings.image_folder = "images";
    run_settings.exposure_gt_file = "times.txt";
    run_settings.calibration_mode = "online";
    run_settings.nr_active_frames = 200;
    run_settings.keyframe_spacing = 15;
    run_settings.min_keyframes_valid = 3;
    run_settings.intensity_topic = "/SwissRanger/intensity/image_raw";
    run_settings.range_topic = "/SwissRanger/distance/image_raw";

    app.add_option("-i,--image-folder", run_settings.image_folder, "Folder with image files to read.", true);
    app.add_option("--start-image-index", run_settings.start_image_index, "Start reading from this image index.", true);
    app.add_option("--end-image-index", run_settings.end_image_index, "Stop reading at this image index.", true);
    app.add_option("--image-width", run_settings.image_width, "Resize image to this width.", true);
    app.add_option("--image-height", run_settings.image_height, "Resize image to this height.", true);
    app.add_option("--exposure-gt-file", run_settings.exposure_gt_file, "Textfile containing ground truth exposure times for each frame for visualization.", true);
    app.add_option("--calibration-mode", run_settings.calibration_mode, "Choose 'online' or 'batch'", true);

    app.add_option("--intensity-topic", run_settings.intensity_topic, "Choose a ros topic to subscribe to for intensity data", true);
    app.add_option("--range-topic", run_settings.range_topic, "Choose a ros topic to subscribe to for intensity data", true);

    app.add_option("--nr-active-frames", run_settings.nr_active_frames, "Maximum number of frames to be stored in the database.", true);
    app.add_option("--keyframe-spacing", run_settings.keyframe_spacing, "Number of frames that keyframes are apart in the backend optimizer.", true);
    app.add_option("--min-keyframes-valid", run_settings.min_keyframes_valid, "Minimum number of frames a feature has to be tracked to be considered for optimization.", true);

    CLI11_PARSE(app, argc, argv);

    printf("Image width %d\n", run_settings.image_width);
    printf("Image height %d\n", run_settings.image_height);

    //  Set up the object to read new images from
    // ImageReader image_reader(run_settings->image_folder, cv::Size(run_settings->image_width, run_settings->image_height));
    Globals g;
    g.image_num = 0;
    g.optimize_cnt = 0;
    g.vis_exponent = 1.0;
    g.safe_zone_size = run_settings.nr_images_rapid_exp + 5;
    // Set up the information database
    Database database(run_settings.image_width, run_settings.image_height);

    // Setup the rapid  exposure time estimator
    RapidExposureTimeEstimator exposure_estimator(run_settings.nr_images_rapid_exp, &database);

    // Setup the nonlinear optimizer
    NonlinearOptimizer backend_optimizer(run_settings.keyframe_spacing,
                                         &database,
                                         g.safe_zone_size,
                                         run_settings.min_keyframes_valid,
                                         run_settings.tracker_patch_size);

    // Set up the object that handles the tracking and receives new images, extracts features
    Tracker tracker(run_settings.tracker_patch_size, run_settings.nr_active_features, run_settings.nr_pyramid_levels, &database);

    ros::init(argc, argv, "online_photometric_calibration");
    ros::NodeHandle nh;
    ros::Publisher test_pub = nh.advertise<std_msgs::String>("image_converter/test_cpp", 1000);

    // Listen for ros topic and publish to image_callback with bound args
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/image_converter/converted", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
        MySyncPolicy;

    message_filters::Subscriber<sensor_msgs::Image> intensity_sub(nh, run_settings.intensity_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> distance_sub(nh, run_settings.range_topic, 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), intensity_sub, distance_sub);
    sync.registerCallback(boost::bind(&image_callback, _1, _2, test_pub, &g, &run_settings, &database, tracker, exposure_estimator, backend_optimizer));
    ros::spin();

    // Moved this from the online_calibration to after ROS exits.
    pthread_join(opt_thread, NULL);

    if (g.optimize_cnt > 0)
    {
        // Write the result to the database, visualize the result
        database.m_vignette_estimate.setVignetteParameters(backend_optimizer.m_vignette_estimate);
        database.m_response_estimate.setGrossbergParameterVector(backend_optimizer.m_response_estimate);
        database.m_response_estimate.setInverseResponseVector(backend_optimizer.m_raw_inverse_response);
        g.vis_exponent = backend_optimizer.visualizeOptimizationResult(backend_optimizer.m_raw_inverse_response);
    }
    // wait for key-press, then exit
    std::cout << "Finished. Press key to exit." << std::endl;
    cv::waitKey(0);
    return 0;
}

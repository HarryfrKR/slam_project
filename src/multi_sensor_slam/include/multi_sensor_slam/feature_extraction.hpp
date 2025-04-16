#ifndef CAMERA_UTILS_HPP_
#define CAMERA_UTILS_HPP_

#ifdef forEach
#undef forEach
#endif

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <karto_sdk/Karto.h>
//#include <DBoW2/DBoW2>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <multi_sensor_slam/ORBextractor.h>
//#include <multi_sensor_slam/vocaburary.h>

namespace camera_utils {

class FeatureExtractor {
public:
    FeatureExtractor();
    void extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

private:
    std::shared_ptr<orb::ORBextractor> orb_extractor_; 
};
}  // namespace camera_utils

#endif  // CAMERA_UTILS_HPP_

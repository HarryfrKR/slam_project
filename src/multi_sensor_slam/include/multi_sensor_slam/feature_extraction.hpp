#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <karto_sdk/Karto.h>

#include <DBoW2/TemplatedVocabulary.h>
#include <DBoW2/FORB.h>
#include <DBoW2/FeatureVector.h>
#include <DBoW2/BowVector.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <multi_sensor_slam/ORBextractor.h>
#include <multi_sensor_slam/ORBVocabulary.h>

namespace camera_utils {
class FeatureExtractor {
public:
    FeatureExtractor();
    void extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
    static void transformToBoW(const cv::Mat& descriptors,
        std::shared_ptr<orb::ORBVocabulary> vocab,
        DBoW2::FeatureVector& feat_vec,
        DBoW2::BowVector& bow_vec);

private:
    std::shared_ptr<orb::ORBextractor> orb_extractor_; 
    std::shared_ptr<orb::ORBVocabulary> vocab_; // do we need this here?
};
}  // namespace camera_utils


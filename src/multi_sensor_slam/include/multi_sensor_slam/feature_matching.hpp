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
// #include <DBoW2/DBoW2>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
// #include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <multi_sensor_slam/ORBextractor.h>
#include <multi_sensor_slam/keyframes.hpp>

namespace camera_utils {

class FeatureMatcher {
public:
    std::vector<cv::DMatch> matchFeatures(const cv::Mat &descriptors1, const cv::Mat &descriptors2);
    std::vector<cv::DMatch> matchFeaturesFLANN(const cv::Mat& descriptors1, const cv::Mat& descriptors2); 
    std::vector<cv::DMatch> filterMatchesWithFundamentalMatrix(const std::vector<cv::DMatch>& matches,
                                                                const std::vector<cv::KeyPoint>& keypoints1,
                                                                const std::vector<cv::KeyPoint>& keypoints2);
    std::vector<cv::DMatch> filterMatchesWithRANSAC(const std::vector<cv::DMatch>& matches, 
                                                        const std::vector<cv::KeyPoint>& keypoints1, 
                                                        const std::vector<cv::KeyPoint>& keypoints2);
    bool computeRelativePose(const std::vector<cv::DMatch>& matches, 
                            const std::vector<cv::KeyPoint>& keypoints1, 
                            const std::vector<cv::KeyPoint>& keypoints2, 
                            karto::Pose2 &visualPose);

private:
    // std::shared_ptr<orb::ORBextractor> orb_extractor_;  // orb namespace is defined in ORBextractor.h
};

// class ORBBoWMatcher {
//     public:
//         ORBBoWMatcher(float nn_ratio = 0.6f, int hamming_thresh = 50);   
//         std::vector<cv::DMatch> match(const Keyframe& kf1, const Keyframe& kf2) const;
    
//     private:
//         int descriptorDistance(const cv::Mat& a, const cv::Mat& b) const;
    
//         float nn_ratio_;
//         int hamming_thresh_;
//     };
    
}  // namespace camera_utils

#endif  // CAMERA_UTILS_HPP_

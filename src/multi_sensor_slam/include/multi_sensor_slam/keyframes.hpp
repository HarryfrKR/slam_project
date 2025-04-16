#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
// #include <DBoW2/DBoW2/FeatureVector.h>
#include <karto_sdk/Karto.h>

namespace camera_utils {

struct Keyframe {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    //DBoW2::FeatureVector feat_vec;
    karto::Pose2 estimated_robot_pose;
    rclcpp::Time timestamp;
    int index;
};

class KeyframeHolder {
public:
    KeyframeHolder();
    ~KeyframeHolder();

    void addKeyframe(const Keyframe& keyframe);
    const Keyframe& getKeyframe(int id) const;
    void addKeyframeImage(const cv::Mat& keyframe_image);
    cv::Mat& getKeyframeImage(int index);
    std::vector<Keyframe> getAllKeyframes() const;
    size_t size() const  { return keyframes_.size(); };
    void clear();

private:
    std::vector<Keyframe> keyframes_;
    std::vector<cv::Mat> keyframe_images_;
};

}  // namespace camera_utils

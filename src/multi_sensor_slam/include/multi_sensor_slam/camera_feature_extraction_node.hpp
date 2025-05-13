#ifndef CAMERA_FEATURE_EXTRACTION_NODE_HPP_
#define CAMERA_FEATURE_EXTRACTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <karto_sdk/Karto.h>

#include "multi_sensor_slam/ORBextractor.h"
#include "multi_sensor_slam/keyframes.hpp"
#include "multi_sensor_slam/feature_matching.hpp"
#include "multi_sensor_slam/feature_extraction.hpp"
#include "multi_sensor_slam/ORBVocabulary.h"


class CameraFeatureExtractionNode : public rclcpp::Node {
public:
    CameraFeatureExtractionNode(std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder);
    std::shared_ptr<camera_utils::FeatureExtractor> feature_extractor_; 
    std::shared_ptr<camera_utils::FeatureMatcher> feature_matcher_;
    karto::Pose2 getRobotPose();
    karto::Pose2 previous_keyframe_pose_;
    std::shared_ptr<orb::ORBVocabulary> vocab_;
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr orb_feature_pub_;
    rclcpp::TimerBase::SharedPtr keyframe_timer_;

    std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder_; 
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Time keyframe_time;
    std::shared_ptr<sensor_msgs::msg::Image> last_image_msg_; 
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_; 

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void processKeyframe();
    void publishKeypoints(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat &image);
    std::pair<std::vector<cv::KeyPoint>, cv::Mat> extractFeatures(const cv::Mat &image);
    bool isKeyframe(const sensor_msgs::msg::Image::SharedPtr msg, const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors, rclcpp::Time time);
};


#endif  // CAMERA_FEATURE_EXTRACTION_NODE_HPP_


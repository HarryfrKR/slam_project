#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>  
#include <karto_sdk/Karto.h>
#include <filesystem>
#include <rcpputils/filesystem_helper.hpp>

#include <DBoW2/FeatureVector.h>
#include <DBoW2/BowVector.h>

#include "multi_sensor_slam/camera_feature_extraction_node.hpp"
#include "multi_sensor_slam/feature_matching.hpp"
#include "multi_sensor_slam/feature_extraction.hpp"

using namespace std;
using namespace cv;
using namespace camera_utils;
using namespace karto;
using namespace orb;


CameraFeatureExtractionNode::CameraFeatureExtractionNode(
    std::shared_ptr<camera_utils::KeyframeHolder> keyframe_holder)
    : Node("camera_feature_extractor"), keyframe_holder_(keyframe_holder) {  

    RCLCPP_INFO(this->get_logger(), "Initializing Camera Feature Extraction Node...");

    feature_extractor_ = std::make_shared<camera_utils::FeatureExtractor>();
    feature_matcher_ = std::make_shared<camera_utils::FeatureMatcher>();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    last_image_msg_ = nullptr;
    
    vocab_ = std::make_shared<ORBVocabulary>();
    std::string vocab_path = "src/multi_sensor_slam/test/ORBvoc.txt";
    if (!vocab_->loadFromTextFile(vocab_path)) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load vocabulary!");
        rclcpp::shutdown();
    }

    std::filesystem::create_directory("loop_matches");
    RCLCPP_INFO(this->get_logger(), "Vocabulary loaded. Ready for loop closure test.");


    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw", rclcpp::QoS(1), 
        std::bind(&CameraFeatureExtractionNode::imageCallback, this, std::placeholders::_1));
    keyframe_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&CameraFeatureExtractionNode::processKeyframe, this));
    orb_feature_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/slam_toolbox/orb_features", 10);

}

void CameraFeatureExtractionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!msg || msg->data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR: Received NULL or empty image message!");
        return;
    }
    last_image_msg_ = msg;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // change encoding if not BGR8

    // Convert to ORB-compatible format (CV_8UC1)
    cv::Mat gray_image;
    gray_image = cv::Mat(cv_ptr->image.size(), CV_8UC1);

    if (cv_ptr->image.type() == CV_8UC3) {
        cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    } else if (cv_ptr->image.type() == CV_16UC1) {
        cv_ptr->image.convertTo(gray_image, CV_8UC1, 1.0 / 256.0);
        RCLCPP_INFO(this->get_logger(), "type 16");
    } else if (cv_ptr->image.type() == CV_8UC1) {
        gray_image = cv_ptr->image.clone(); 
        RCLCPP_INFO(this->get_logger(), "Already Gray");
    } else {
        RCLCPP_ERROR(this->get_logger(), " ERROR: Unsupported image format: %d", cv_ptr->image.type());
        return;
    }

    // auto [keypoints, descriptors] = extractFeatures(gray_image);
    std::tie(keypoints_, descriptors_) = extractFeatures(gray_image);
    //keypoints_ready.store(true);
    publishKeypoints(keypoints_, cv_ptr->image);
    
}

void CameraFeatureExtractionNode::processKeyframe() {
    if (!last_image_msg_ || keypoints_.empty() || descriptors_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Waiting for valid image message or extracted features...");
        return;
    }

    if (isKeyframe(last_image_msg_, keypoints_, descriptors_, this->now())) {
        // RCLCPP_INFO(this->get_logger(), "New keyframe detected and processed successfully.");
    } else {
        RCLCPP_INFO(this->get_logger(), "No new keyframe detected.");
    }
}

std::pair<std::vector<cv::KeyPoint>, cv::Mat> CameraFeatureExtractionNode::extractFeatures(const cv::Mat &image) {
    if (image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR: Received an empty image for feature extraction!");
        return {{}, {}};
    }

    vector<KeyPoint> keypoints;
    Mat descriptors;
    feature_extractor_->extractFeatures(image, keypoints, descriptors);

    return {keypoints, descriptors};
}

bool CameraFeatureExtractionNode::isKeyframe(const sensor_msgs::msg::Image::SharedPtr msg, const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors, rclcpp::Time time) {
    rclcpp::Time keyframe_time;
    cv::Mat image;

    DBoW2::FeatureVector feat_vec;
    DBoW2::BowVector bow_vec;

    try {
        image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return false;
    }

    std::string keyframe_dir = "keyframes/";
    if (!rcpputils::fs::exists(keyframe_dir)) {
        rcpputils::fs::create_directories(keyframe_dir);
    }

    Pose2 estimatedPose = getRobotPose();
    
    if (keyframe_holder_->size() == 0) {
        previous_keyframe_pose_ = estimatedPose; 
        feature_extractor_->transformToBoW(descriptors, vocab_, feat_vec, bow_vec);

        Keyframe new_keyframe{keypoints, descriptors.clone(), bow_vec, estimatedPose, keyframe_time, static_cast<int>(keyframe_holder_->size()) - 1};
        keyframe_holder_->addKeyframe(new_keyframe);
        keyframe_holder_->addKeyframeImage(image);
        // RCLCPP_INFO(this->get_logger(), "First frame - Saving as keyframe.");
        std::string filename = keyframe_dir + "keyframe_0.png";
        cv::imwrite(filename, image);
        return true;
    }

    try {
        // Get last keyframe and match features
        const Keyframe& last_kf = keyframe_holder_->getKeyframe(keyframe_holder_->size() - 1);
        vector<DMatch> matches = feature_matcher_->matchFeatures(descriptors, last_kf.descriptors);

        if (matches.size() < 20) { 
            RCLCPP_WARN(this->get_logger(), "Too few matches for relative pose estimation.");
            return false;
        }

        // Compute relative pose
        Pose2 relativePose;
        feature_matcher_->computeRelativePose(matches, keypoints, last_kf.keypoints, relativePose);

        // Define motion thresholds
        double translation_threshold = 0.1;  // cm
        double rotation_threshold = 0.05;    // rad

        // Compute pose difference from last saved keyframe
        double dx = estimatedPose.GetX() - previous_keyframe_pose_.GetX();
        double dy = estimatedPose.GetY() - previous_keyframe_pose_.GetY();
        double dtheta = std::fabs(estimatedPose.GetHeading() - previous_keyframe_pose_.GetHeading());

        double translation_magnitude = std::sqrt(dx * dx + dy * dy);

        // Check if movement is significant
        if (translation_magnitude > translation_threshold || dtheta > rotation_threshold) {
            previous_keyframe_pose_ = estimatedPose; 
            feature_extractor_->transformToBoW(descriptors, vocab_, feat_vec, bow_vec);
            Keyframe new_keyframe{keypoints, descriptors.clone(), bow_vec, estimatedPose, keyframe_time, static_cast<int>(keyframe_holder_->size()) - 1};
            keyframe_holder_->addKeyframe(new_keyframe);
            keyframe_holder_->addKeyframeImage(image);
            // RCLCPP_INFO(this->get_logger(), "Added new keyframe with estimated pose (%.2f, %.2f, %.2f)",
            //             estimatedPose.GetX(), estimatedPose.GetY(), estimatedPose.GetHeading());
               
            // Save the image with the keyframe ID
            int keyframe_id = keyframe_holder_->size() - 1;
            std::string filename = keyframe_dir + "keyframe_" + std::to_string(keyframe_id) + ".png";
            cv::imwrite(filename, image);

            return true;
        } else {
            //RCLCPP_INFO(this->get_logger(), "No significant motion detected. Skipping keyframe.");
            return false;
        }

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in keyframe processing: %s", e.what());
        return false;
    }
}

void CameraFeatureExtractionNode::publishKeypoints(const vector<KeyPoint>& keypoints, const cv::Mat &image) {
    if (keypoints.empty()) {
        // RCLCPP_WARN(this->get_logger(), "No keypoints detected, skipping publish.");
        return;
    }

    cv::Mat image_with_keypoints = image.clone();
    cv::drawKeypoints(image, keypoints, image_with_keypoints, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

    sensor_msgs::msg::Image image_msg = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_with_keypoints).toImageMsg());
    image_msg.header.stamp = this->now();
    //image_msg.header.frame_id = "camera_frame";

    if (orb_feature_pub_) {
        orb_feature_pub_->publish(image_msg);
    } else {
        RCLCPP_ERROR(this->get_logger(),"Image publisher is not initialized.");
    }
}

karto::Pose2 CameraFeatureExtractionNode::getRobotPose() {
    geometry_msgs::msg::TransformStamped transform;

    try {
        transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
        return karto::Pose2(0.0, 0.0, 0.0); // Return a default pose if transformation fails
    }

    tf2::Quaternion quat(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);

    double yaw = tf2::getYaw(quat);
    return karto::Pose2(transform.transform.translation.x, 
                        transform.transform.translation.y,
                        yaw);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto keyframe_holder = std::make_shared<camera_utils::KeyframeHolder>();

    auto camera_node = std::make_shared<CameraFeatureExtractionNode>(keyframe_holder);
    rclcpp::spin(camera_node);
    rclcpp::shutdown();
    return 0;
}
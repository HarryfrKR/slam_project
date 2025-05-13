#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iomanip>

#include "multi_sensor_slam/feature_extraction.hpp"
#include "multi_sensor_slam/ORBVocabulary.h"

using namespace std;
using namespace cv;
using namespace camera_utils;
using namespace orb;

class LoopClosureTestNode : public rclcpp::Node {
public:
    LoopClosureTestNode() : Node("loop_closure_test_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&LoopClosureTestNode::imageCallback, this, std::placeholders::_1));

        vocab_ = std::make_shared<ORBVocabulary>();
        std::string vocab_path = "src/multi_sensor_slam/test/ORBvoc.txt";
        if (!vocab_->loadFromTextFile(vocab_path)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load vocabulary!");
            rclcpp::shutdown();
        }

        std::filesystem::create_directory("loop_matches");
        RCLCPP_INFO(this->get_logger(), "Vocabulary loaded. Ready for loop closure test.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat image_gray;
        try {
            image_gray = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cvtColor(image_gray, image_gray, COLOR_BGR2GRAY);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        extractor_.extractFeatures(image_gray, keypoints, descriptors);
        if (descriptors.empty()) return;
        int exclusion_window = 30;

        DBoW2::FeatureVector feat_vec;
        DBoW2::BowVector bow_vec;
        
        FeatureExtractor::transformToBoW(descriptors, vocab_, feat_vec, bow_vec);
        for (size_t i = 0; i < keyframe_bows_.size(); ++i) {
            if (i + exclusion_window >= current_id_) continue; 
        
            float score = vocab_->score(bow_vec, keyframe_bows_[i].bow_vec);
            if (score > 0.03f) {
                RCLCPP_INFO(this->get_logger(), "Loop candidate: Frame %zu — Score: %.3f", i, score);
                saveLoopMatchImage(image_gray, keypoints, descriptors, i,
                                   keyframe_bows_[i].image, keyframe_bows_[i].keypoints,
                                   keyframe_bows_[i].descriptors, current_id_);
            }
        }
        
        keyframe_bows_.push_back({
            .bow_vec = bow_vec,
            .image = image_gray.clone(),
            .keypoints = keypoints,
            .descriptors = descriptors.clone()
        });

        ++current_id_;
    }

    void saveLoopMatchImage(const cv::Mat& img1, const std::vector<cv::KeyPoint>& kp1, const cv::Mat& desc1,
                             size_t id2, const cv::Mat& img2, const std::vector<cv::KeyPoint>& kp2, const cv::Mat& desc2,
                             size_t current_id) {
        std::vector<cv::DMatch> matches;
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.match(desc1, desc2, matches);
        std::sort(matches.begin(), matches.end(), [](const auto& a, const auto& b) {
            return a.distance < b.distance;
        });
        matches.resize(std::min<size_t>(170, matches.size()));

        cv::Mat img_matches;
        cv::drawMatches(img1, kp1, img2, kp2, matches, img_matches);

        std::ostringstream ss;
        ss << "loop_matches/match_" << std::setfill('0') << std::setw(4) << current_id
           << "_vs_" << std::setw(4) << id2 << ".png";

        cv::imwrite(ss.str(), img_matches);
        RCLCPP_INFO(this->get_logger(), "Saved match visualization: %s", ss.str().c_str());
    }

    struct KeyframeEntry {
        DBoW2::BowVector bow_vec;
        cv::Mat image;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
    };

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<ORBVocabulary> vocab_;
    FeatureExtractor extractor_;
    std::vector<KeyframeEntry> keyframe_bows_;
    size_t current_id_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoopClosureTestNode>());
    rclcpp::shutdown();
    return 0;
}

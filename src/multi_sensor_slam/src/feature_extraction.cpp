
#include <cmath>
#include <string>
#include <vector>
#include <memory>

#include "multi_sensor_slam/feature_extraction.hpp"

using namespace cv;
using namespace std;
using namespace orb;
using namespace karto;

namespace camera_utils {

FeatureExtractor::FeatureExtractor() {
    try {
        int nFeatures = 600;
        float scaleFactor = 1.2f;
        int nLevels = 8;
        int iniThFAST = 20;
        int minThFAST = 10;

        orb_extractor_ = std::make_unique<orb::ORBextractor>(
            nFeatures, scaleFactor, nLevels, iniThFAST, minThFAST);

        if (!orb_extractor_) {
            throw std::runtime_error("ORBextractor failed to initialize.");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureExtraction"), " Exception in ORBextractor constructor: %s", e.what());
    }
}


void FeatureExtractor::extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    if (image.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureExtraction"), "ERROR: Received empty image! Skipping feature extraction.");
        return;
    }

    std::vector<int> vLappingArea = {0, image.cols};
    // cv::Mat mask = cv::noArray();  // Empty mask

    if (!orb_extractor_) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureExtraction"), "ERROR: ORBextractor is NULL!");
        return;
    } 

    orb_extractor_->operator()(image, noArray(), keypoints, descriptors, vLappingArea);
    orb_extractor_->operator()(image, noArray(), keypoints, descriptors, vLappingArea);
    // RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), "Extracted %lu keypoints.", keypoints.size());

} 

void FeatureExtractor::transformToBoW(const cv::Mat& descriptors, std::shared_ptr<ORBVocabulary> vocab, DBoW2::FeatureVector& feat_vec, DBoW2::BowVector& bow_vec) {
    std::vector<cv::Mat> descriptor_vec;
    for (int i = 0; i < descriptors.rows; ++i)
        descriptor_vec.push_back(descriptors.row(i).clone());

    vocab->transform(descriptor_vec, bow_vec, feat_vec, 6);
}
}  // namespace camera_utils

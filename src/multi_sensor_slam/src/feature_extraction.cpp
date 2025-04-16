
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
        int nFeatures = 200;
        float scaleFactor = 1.2f;
        int nLevels = 8;
        int iniThFAST = 30;
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
    // RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), "Extracted %lu keypoints.", keypoints.size());

} 
}  // namespace camera_utils

#include <cmath>
#include <string>
#include <vector>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <iostream>
//#include <DBoW2/DBoW2>
#include "multi_sensor_slam/feature_matching.hpp"

using namespace camera_utils;
using namespace cv;
using namespace std;
using namespace karto;

/** Match ORB Features */
vector<DMatch> FeatureMatcher::matchFeatures(
    // for binary string descriptors
    const Mat& descriptors1, const Mat& descriptors2) {
    // RCLCPP_INFO(rclcpp::get_logger("FeatureExtraction"), "Matching Features...");
    if (descriptors1.empty() || descriptors2.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("FeatureExtraction"), "BFMatcher: One or both descriptor matrices are empty!");
        return {};  // Return empty vector
    }
    
    BFMatcher matcher(NORM_HAMMING, true);  // Brute-force matcher
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    std::sort(matches.begin(), matches.end(), [](const DMatch &a, const DMatch &b) {
        return a.distance < b.distance;
    });

    // double hamming_threshold = 50.0; // Keep only good matches (adjust this)
    // vector<DMatch> good_matches;

    // for (const auto& match : matches) {
    //     if (match.distance < hamming_threshold) {
    //         good_matches.push_back(match);
    //     }
    // }

    size_t num_to_keep = matches.size() * 0.7; 
    vector<DMatch> best_matches(matches.begin(), matches.begin() + num_to_keep);

    // **Return the better-filtered list**
    return best_matches;
}

vector<DMatch> FeatureMatcher::matchFeaturesFLANN(const Mat& descriptors1, const Mat& descriptors2) {
    // for floating point descriptors such as SURF or SIFT
    if (descriptors1.empty() || descriptors2.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("FeatureMatcher"), "FLANN Matching: One or both descriptor matrices are empty!");
        return {};  // Return empty vector
    }

    Mat desc1, desc2;
    descriptors1.convertTo(desc1, CV_32F);
    descriptors2.convertTo(desc2, CV_32F);

    if (desc1.cols != desc2.cols || desc1.cols != 32) {  // ORB descriptors are always 32 bytes
        RCLCPP_ERROR(rclcpp::get_logger("FeatureMatcher"), 
                        "FLANN Error: Descriptor column sizes do not match or are incorrect! Desc1: %d x %d, Desc2: %d x %d",
                        desc1.rows, desc1.cols, desc2.rows, desc2.cols);
        return {};
    }

    if (desc1.type() != CV_32F || desc2.type() != CV_32F) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureMatcher"), 
                        "FLANN Error: Descriptor types are not CV_32F after conversion! Desc1 Type: %d, Desc2 Type: %d",
                        desc1.type(), desc2.type());
        return {};
    }
    Ptr<flann::IndexParams> indexParams = makePtr<flann::LshIndexParams>(12, 20, 2);
    Ptr<flann::SearchParams> searchParams = makePtr<flann::SearchParams>(50);
    FlannBasedMatcher matcher(indexParams, searchParams);

    vector<vector<DMatch>> knnMatches;
    try {
        matcher.knnMatch(desc1, desc2, knnMatches, 2); // KNN with k=2
    } catch (const cv::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("FeatureMatcher"), 
                     "FLANN Matcher Exception: %s", e.what());
        return {};
    }

    // Apply Lowe's ratio test
    vector<DMatch> good_matches;
    for (const auto& match : knnMatches) {
        if (match.size() >= 2 && match[0].distance < 0.75 * match[1].distance) {
            good_matches.push_back(match[0]);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("FeatureMatcher"), "FLANN Matching found %lu good matches.", good_matches.size());
    return good_matches;

}

vector<DMatch> FeatureMatcher::filterMatchesWithFundamentalMatrix(const vector<DMatch>& matches,
                                                  const vector<KeyPoint>& keypoints1,
                                                  const vector<KeyPoint>& keypoints2) {
    vector<Point2f> points1, points2;
    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }

    // Compute fundamental matrix using RANSAC
    vector<uchar> mask;
    Mat F = findFundamentalMat(points1, points2, FM_RANSAC, 3.0, 0.99, mask);

    vector<DMatch> filtered_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (mask[i]) {
            filtered_matches.push_back(matches[i]);
        }
    }

    return filtered_matches;
}

/** Filter Matches Using RANSAC (PROSAC) */
vector<DMatch> FeatureMatcher::filterMatchesWithRANSAC(
    const vector<DMatch>& matches, 
    const vector<KeyPoint>& keypoints1, 
    const vector<KeyPoint>& keypoints2) {

    if (matches.empty()) return {}; // Return empty if no matches

    vector<Point2f> points1, points2;
    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }

    Mat mask;
    findFundamentalMat(points1, points2, FM_RANSAC, 3, 0.99, mask);

    vector<DMatch> filtered_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (mask.at<uchar>(i)) {
            filtered_matches.push_back(matches[i]);
        }
    }

    return filtered_matches;
}

bool FeatureMatcher::computeRelativePose(const vector<DMatch>& matches, 
    const vector<KeyPoint>& keypoints1, 
    const vector<KeyPoint>& keypoints2, 
    Pose2 & visualPose) {
        
    vector<Point2f> points1, points2;
    for (const auto& match : matches) {
    points1.push_back(keypoints1[match.queryIdx].pt);
    points2.push_back(keypoints2[match.trainIdx].pt);
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) <<  910.1874389648438,  0.0,                623.305908203125,  // fx,  0, cx
                        0.0,                910.4207763671875, 378.3083190917969,  //  0, fy, cy
                        0.0,                 0.0,                1.0);  //  0,  0,  1
    Mat E = findEssentialMat(points1, points2, K, RANSAC, 0.999, 1.0 ); // confidence, pixel error
    Mat R, t;
    recoverPose(E, points1, points2, R, t);

    double theta = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    visualPose = Pose2(t.at<double>(0, 0), t.at<double>(1, 0), theta);

    return true; // Success
    }
    
// ORBBoWMatcher::ORBBoWMatcher(float nn_ratio, int hamming_thresh)
//     : nn_ratio_(nn_ratio), hamming_thresh_(hamming_thresh) {}

// int ORBBoWMatcher::descriptorDistance(const cv::Mat& a, const cv::Mat& b) const {
//     const int* pa = a.ptr<int32_t>();
//     const int* pb = b.ptr<int32_t>();
//     int dist = 0;
//     for (int i = 0; i < 8; i++, pa++, pb++) {
//         unsigned int v = *pa ^ *pb;
//         v = v - ((v >> 1) & 0x55555555);
//         v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
//         dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
//     }
//     return dist;
// }

// std::vector<cv::DMatch> ORBBoWMatcher::match(const Keyframe& kf1, const Keyframe& kf2) const {
//     std::vector<cv::DMatch> matches;

//     auto it1 = kf1.feat_vec.begin();
//     auto it2 = kf2.feat_vec.begin();

//     while (it1 != kf1.feat_vec.end() && it2 != kf2.feat_vec.end()) {
//         if (it1->first == it2->first) {
//             const auto& idxs1 = it1->second;
//             const auto& idxs2 = it2->second;

//             for (unsigned int i1 : idxs1) {
//                 const cv::Mat& desc1 = kf1.descriptors.row(i1);
//                 int best_dist = 256, second_best = 256, best_idx2 = -1;

//                 for (unsigned int i2 : idxs2) {
//                     const cv::Mat& desc2 = kf2.descriptors.row(i2);
//                     int dist = descriptorDistance(desc1, desc2);

//                     if (dist < best_dist) {
//                         second_best = best_dist;
//                         best_dist = dist;
//                         best_idx2 = i2;
//                     } else if (dist < second_best) {
//                         second_best = dist;
//                     }
//                 }

//                 if (best_dist < hamming_thresh_ &&
//                     static_cast<float>(best_dist) < nn_ratio_ * static_cast<float>(second_best)) {
//                     matches.emplace_back(i1, best_idx2, static_cast<float>(best_dist));
//                 }
//             }

//             ++it1;
//             ++it2;
//         } else if (it1->first < it2->first) {
//             it1 = kf1.feat_vec.lower_bound(it2->first);
//         } else {
//             it2 = kf2.feat_vec.lower_bound(it1->first);
//         }
//     }

//     return matches;
// }

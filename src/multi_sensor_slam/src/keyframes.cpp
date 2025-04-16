
#include <cmath>
#include <string>
#include <vector>
#include <memory>

#include "multi_sensor_slam/keyframes.hpp"

using namespace cv;
using namespace std;
using namespace camera_utils;
using namespace karto;

KeyframeHolder::KeyframeHolder() 
{

}
KeyframeHolder::~KeyframeHolder() 
{

}
std::vector<Keyframe> KeyframeHolder::getAllKeyframes() const {
    return keyframes_;
}
void KeyframeHolder::addKeyframe(const Keyframe& keyframe) {
    keyframes_.push_back(keyframe);
}

void KeyframeHolder::addKeyframeImage(const cv::Mat& keyframe_image) {
    keyframe_images_.push_back(keyframe_image);
}

cv::Mat& KeyframeHolder::getKeyframeImage(int keyframe_index) {
    if (keyframe_index < 0 || keyframe_index >= (int)keyframe_images_.size()) {
        throw std::out_of_range("KeyframeHolder: Keyframe index out of range");
    }
    return keyframe_images_.at(keyframe_index);
}

const Keyframe& KeyframeHolder::getKeyframe(int id) const {
    if (id < 0 || id >= (int)keyframes_.size()) {
        throw std::out_of_range("KeyframeHolder: Keyframe index out of range");
    }
    return keyframes_.at(id);
}

void KeyframeHolder::clear() {
    keyframes_.clear();
}


// keyframe_database.hpp
#pragma once

#include <vector>
#include <list>
#include <mutex>
#include <memory>
#include <unordered_set>
#include <opencv2/core.hpp>
#include <DBoW2/DBoW2/FORB.h>
#include <DBoW2/DBoW2/TemplatedVocabulary.h>
#include <DBoW2/DBoW2/BowVector.h>
#include "multi_sensor_slam/keyframes.hpp"  // assumes Keyframe struct is defined here

namespace camera_utils {

class KeyframeDatabase {
public:
    explicit KeyframeDatabase(std::shared_ptr<ORBVocabulary> vocab);

    void add(const Keyframe& kf);
    void clear();

    std::vector<const Keyframe*> detectLoopCandidates(const Keyframe& query, float minScore = 0.015f);

private:
    std::shared_ptr<ORBVocabulary> vocab_;
    std::vector<std::list<const Keyframe*>> inverted_file_;
    mutable std::mutex mutex_;
};

} // namespace vision

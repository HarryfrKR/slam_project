// keyframe_database.cpp
#include "multi_sensor_slam/keyframe_database.h"
#include <algorithm>
#include <iostream>

namespace camera_utils {

KeyframeDatabase::KeyframeDatabase(std::shared_ptr<ORBVocabulary> vocab)
    : vocab_(std::move(vocab)) {
    inverted_file_.resize(vocab_->size());
}

void KeyframeDatabase::add(const Keyframe& kf) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& word_entry : kf.feat_vec) {
        inverted_file_[word_entry.first].push_back(&kf);
    }
}

void KeyframeDatabase::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& entry : inverted_file_) {
        entry.clear();
    }
}

std::vector<const Keyframe*> KeyframeDatabase::detectLoopCandidates(const Keyframe& query, float minScore) {
    std::unordered_set<const Keyframe*> unique_candidates;
    std::vector<const Keyframe*> loop_candidates;

    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto& word_entry : query.feat_vec) {
        for (const auto* kf_ptr : inverted_file_[word_entry.first]) {
            unique_candidates.insert(kf_ptr);
        }
    }

    for (const auto* candidate : unique_candidates) {
        float score = vocab_->score(query.feat_vec, candidate->feat_vec);
        if (score > minScore) {
            loop_candidates.push_back(candidate);
        }
    }

    return loop_candidates;
}

} // namespace vision

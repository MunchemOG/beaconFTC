// Bag of binary words for place recognition: a hierarchical k-majority vocabulary tree over
// ORB descriptors (our own implementation and our own trained vocabulary; no ORB-SLAM/DBoW2
// vocabulary file), TF-IDF weighted, L1-scored, plus an inverted-index keyframe database.
#pragma once

#include <cstdint>
#include <map>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace beacon {

using BowVector = std::map<int, double>;               // word id -> weight (L1-normalized)
using FeatureVector = std::map<int, std::vector<int>>; // node id at a coarser level -> features

class Vocabulary {
public:
    bool empty() const { return nodes_.empty(); }
    int words() const { return (int)wordNodes_.size(); }

    /** Trains a k^L tree on descriptors (rows of 32 bytes) grouped by image (for IDF). */
    void train(const std::vector<cv::Mat>& perImage, int k, int levels, uint32_t seed = 1);

    bool save(const std::string& path) const;
    bool load(const std::string& path);

    /** BoW vector and the feature vector at `levelsUp` above the leaves. */
    void transform(const cv::Mat& desc, BowVector& bow, FeatureVector& fv, int levelsUp = 2) const;

    /** DBoW2-style L1 score in [0, 1]. */
    static double score(const BowVector& a, const BowVector& b);

private:
    struct Node {
        int parent = -1;
        std::vector<int> children;
        uint8_t desc[32] = {0};
        double weight = 0;
        int wordId = -1;
        int level = 0;
    };
    int lookup(const uint8_t* d, int& nodeAtLevel, int level) const;
    void build(int node, const std::vector<const uint8_t*>& descs, int level, uint32_t& rng);

    int k_ = 10, levels_ = 4;
    std::vector<Node> nodes_;
    std::vector<int> wordNodes_;
};

/** Inverted index over keyframe ids. */
class KeyFrameDatabase {
public:
    void add(int kfId, const BowVector& bow);
    void erase(int kfId, const BowVector& bow);
    /** Keyframes sharing words with `bow`, scored; excludes ids in `exclude`. */
    std::vector<std::pair<double, int>> query(const BowVector& bow, const std::vector<int>& exclude,
                                              double minScore, const std::map<int, const BowVector*>& bows,
                                              double minCommonFrac = 0.8) const;
    void clear() { index_.clear(); }

private:
    std::map<int, std::vector<int>> index_;  // word -> kf ids
};

}  // namespace beacon

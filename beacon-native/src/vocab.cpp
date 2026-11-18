#include "vocab.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>

#include "features.h"

namespace beacon {

namespace {

uint32_t nextRand(uint32_t& s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

// Bitwise majority of a set of descriptors (the binary "mean").
void majority(const std::vector<const uint8_t*>& ds, uint8_t* out) {
    int counts[256] = {0};
    for (const uint8_t* d : ds)
        for (int b = 0; b < 256; ++b)
            if (d[b >> 3] & (1 << (b & 7))) counts[b]++;
    std::fill(out, out + 32, 0);
    for (int b = 0; b < 256; ++b)
        if (2 * counts[b] > (int)ds.size()) out[b >> 3] |= (uint8_t)(1 << (b & 7));
}

}  // namespace

void Vocabulary::build(int node, const std::vector<const uint8_t*>& descs, int level, uint32_t& rng) {
    if (level == levels_ || (int)descs.size() <= k_) {
        nodes_[node].wordId = (int)wordNodes_.size();
        wordNodes_.push_back(node);
        return;
    }
    // k-means++ seeding, then a few k-majority iterations.
    std::vector<std::vector<uint8_t>> centres;
    const uint8_t* first = descs[nextRand(rng) % descs.size()];
    centres.push_back(std::vector<uint8_t>(first, first + 32));
    std::vector<int> minD(descs.size(), 1 << 30);
    while ((int)centres.size() < k_) {
        double total = 0;
        for (size_t i = 0; i < descs.size(); ++i) {
            int d = orbDistance(descs[i], centres.back().data());
            minD[i] = std::min(minD[i], d);
            total += (double)minD[i] * minD[i];
        }
        if (total <= 0) break;
        double r = (nextRand(rng) / 4294967296.0) * total, acc = 0;
        size_t pick = descs.size() - 1;
        for (size_t i = 0; i < descs.size(); ++i) {
            acc += (double)minD[i] * minD[i];
            if (acc >= r) {
                pick = i;
                break;
            }
        }
        centres.push_back(std::vector<uint8_t>(descs[pick], descs[pick] + 32));
    }
    int kk = (int)centres.size();
    std::vector<int> assign(descs.size(), -1);
    for (int it = 0; it < 8; ++it) {
        bool changed = false;
        for (size_t i = 0; i < descs.size(); ++i) {
            int best = 0, bd = 1 << 30;
            for (int c = 0; c < kk; ++c) {
                int d = orbDistance(descs[i], centres[c].data());
                if (d < bd) {
                    bd = d;
                    best = c;
                }
            }
            if (assign[i] != best) {
                assign[i] = best;
                changed = true;
            }
        }
        if (!changed) break;
        for (int c = 0; c < kk; ++c) {
            std::vector<const uint8_t*> members;
            for (size_t i = 0; i < descs.size(); ++i)
                if (assign[i] == c) members.push_back(descs[i]);
            if (!members.empty()) majority(members, centres[c].data());
        }
    }
    for (int c = 0; c < kk; ++c) {
        std::vector<const uint8_t*> members;
        for (size_t i = 0; i < descs.size(); ++i)
            if (assign[i] == c) members.push_back(descs[i]);
        if (members.empty()) continue;
        Node child;
        child.parent = node;
        child.level = level + 1;
        std::copy(centres[c].begin(), centres[c].end(), child.desc);
        int id = (int)nodes_.size();
        nodes_.push_back(child);
        nodes_[node].children.push_back(id);
        build(id, members, level + 1, rng);
    }
}

void Vocabulary::train(const std::vector<cv::Mat>& perImage, int k, int levels, uint32_t seed) {
    k_ = k;
    levels_ = levels;
    nodes_.clear();
    wordNodes_.clear();
    nodes_.push_back(Node());
    std::vector<const uint8_t*> all;
    for (const cv::Mat& m : perImage)
        for (int r = 0; r < m.rows; ++r) all.push_back(m.ptr<uint8_t>(r));
    uint32_t rng = seed * 2654435761u + 1;
    build(0, all, 0, rng);
    // IDF: log(N / n_i), n_i = images containing word i.
    std::vector<int> count(wordNodes_.size(), 0);
    for (const cv::Mat& m : perImage) {
        std::set<int> seen;
        for (int r = 0; r < m.rows; ++r) {
            int dummy;
            seen.insert(lookup(m.ptr<uint8_t>(r), dummy, 0));
        }
        for (int w : seen) count[w]++;
    }
    for (size_t w = 0; w < wordNodes_.size(); ++w)
        nodes_[wordNodes_[w]].weight = count[w] > 0 ? std::log((double)perImage.size() / count[w]) : 0.0;
}

int Vocabulary::lookup(const uint8_t* d, int& nodeAtLevel, int level) const {
    int n = 0;
    nodeAtLevel = 0;
    while (!nodes_[n].children.empty()) {
        int best = nodes_[n].children[0], bd = 1 << 30;
        for (int c : nodes_[n].children) {
            int dist = orbDistance(d, nodes_[c].desc);
            if (dist < bd) {
                bd = dist;
                best = c;
            }
        }
        n = best;
        if (nodes_[n].level == level) nodeAtLevel = n;
    }
    return nodes_[n].wordId;
}

void Vocabulary::transform(const cv::Mat& desc, BowVector& bow, FeatureVector& fv, int levelsUp) const {
    bow.clear();
    fv.clear();
    if (nodes_.empty()) {  // no vocabulary: one node holding every feature (brute force)
        for (int r = 0; r < desc.rows; ++r) fv[0].push_back(r);
        return;
    }
    int level = std::max(1, levels_ - levelsUp);
    for (int r = 0; r < desc.rows; ++r) {
        int node;
        int w = lookup(desc.ptr<uint8_t>(r), node, level);
        double weight = nodes_[wordNodes_[w]].weight;
        if (weight > 0) bow[w] += weight;
        fv[node].push_back(r);
    }
    double sum = 0;
    for (auto& e : bow) sum += std::abs(e.second);
    if (sum > 0)
        for (auto& e : bow) e.second /= sum;
}

double Vocabulary::score(const BowVector& a, const BowVector& b) {
    // 1 - 0.5 * |a - b|_1 for L1-normalized vectors = 0.5 * sum over shared words of
    // (|a| + |b| - |a - b|).
    double s = 0;
    auto ia = a.begin();
    auto ib = b.begin();
    while (ia != a.end() && ib != b.end()) {
        if (ia->first < ib->first) ++ia;
        else if (ib->first < ia->first) ++ib;
        else {
            s += std::abs(ia->second) + std::abs(ib->second) - std::abs(ia->second - ib->second);
            ++ia;
            ++ib;
        }
    }
    return 0.5 * s;
}

bool Vocabulary::save(const std::string& path) const {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const char magic[8] = {'B', 'E', 'A', 'C', 'V', 'O', 'C', '1'};
    std::fwrite(magic, 1, 8, f);
    int32_t hdr[3] = {k_, levels_, (int32_t)nodes_.size()};
    std::fwrite(hdr, sizeof hdr, 1, f);
    for (const Node& n : nodes_) {
        int32_t v[3] = {n.parent, n.wordId, n.level};
        std::fwrite(v, sizeof v, 1, f);
        std::fwrite(n.desc, 1, 32, f);
        std::fwrite(&n.weight, sizeof n.weight, 1, f);
    }
    std::fclose(f);
    return true;
}

bool Vocabulary::load(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    char magic[8];
    int32_t hdr[3];
    bool ok = std::fread(magic, 1, 8, f) == 8 && std::equal(magic, magic + 8, "BEACVOC1") &&
              std::fread(hdr, sizeof hdr, 1, f) == 1 && hdr[2] > 0 && hdr[2] < 10000000;
    if (!ok) {
        std::fclose(f);
        return false;
    }
    k_ = hdr[0];
    levels_ = hdr[1];
    nodes_.assign(hdr[2], Node());
    wordNodes_.clear();
    for (int i = 0; i < hdr[2] && ok; ++i) {
        int32_t v[3];
        ok = std::fread(v, sizeof v, 1, f) == 1 && std::fread(nodes_[i].desc, 1, 32, f) == 32 &&
             std::fread(&nodes_[i].weight, sizeof(double), 1, f) == 1;
        nodes_[i].parent = v[0];
        nodes_[i].wordId = v[1];
        nodes_[i].level = v[2];
        if (ok && v[0] >= 0 && v[0] < i) nodes_[v[0]].children.push_back(i);
    }
    std::fclose(f);
    if (!ok) {
        nodes_.clear();
        return false;
    }
    int maxWord = -1;
    for (const Node& n : nodes_) maxWord = std::max(maxWord, n.wordId);
    wordNodes_.assign(maxWord + 1, 0);
    for (int i = 0; i < (int)nodes_.size(); ++i)
        if (nodes_[i].wordId >= 0) wordNodes_[nodes_[i].wordId] = i;
    return true;
}

void KeyFrameDatabase::add(int kf, const BowVector& bow) {
    for (const auto& w : bow) index_[w.first].push_back(kf);
}

void KeyFrameDatabase::erase(int kf, const BowVector& bow) {
    for (const auto& w : bow) {
        auto it = index_.find(w.first);
        if (it == index_.end()) continue;
        auto& v = it->second;
        v.erase(std::remove(v.begin(), v.end(), kf), v.end());
    }
}

std::vector<std::pair<double, int>> KeyFrameDatabase::query(const BowVector& bow, const std::vector<int>& exclude,
                                                            double minScore,
                                                            const std::map<int, const BowVector*>& bows) const {
    std::map<int, int> common;
    for (const auto& w : bow) {
        auto it = index_.find(w.first);
        if (it == index_.end()) continue;
        for (int kf : it->second) common[kf]++;
    }
    std::set<int> ex(exclude.begin(), exclude.end());
    int maxCommon = 0;
    for (const auto& c : common)
        if (!ex.count(c.first)) maxCommon = std::max(maxCommon, c.second);
    std::vector<std::pair<double, int>> out;
    int minCommon = (int)(0.8 * maxCommon);
    for (const auto& c : common) {
        if (ex.count(c.first) || c.second < minCommon) continue;
        auto b = bows.find(c.first);
        if (b == bows.end()) continue;
        double s = Vocabulary::score(bow, *b->second);
        if (s >= minScore) out.push_back({s, c.first});
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    return out;
}

}  // namespace beacon

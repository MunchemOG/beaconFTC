#include "map.h"

#include <algorithm>
#include <cmath>

namespace beacon {

int KeyFrame::trackedPoints(int minObs, const Map& m) const {
    int n = 0;
    for (int id : mps) {
        if (id < 0) continue;
        MapPoint* p = m.mp(id);
        if (p && !p->bad && (int)p->obs.size() >= minObs) ++n;
    }
    return n;
}

KeyFrame* Map::newKeyFrame() {
    auto k = std::make_unique<KeyFrame>();
    k->id = nextKf_++;
    KeyFrame* raw = k.get();
    kfs_[raw->id] = std::move(k);
    if (origin < 0) origin = raw->id;
    return raw;
}

MapPoint* Map::newMapPoint(const Vec3& pos, int refKf) {
    auto p = std::make_unique<MapPoint>();
    p->id = nextMp_++;
    p->pos = pos;
    p->refKf = refKf;
    p->firstKf = refKf;
    MapPoint* raw = p.get();
    mps_[raw->id] = std::move(p);
    return raw;
}

KeyFrame* Map::kf(int id) const {
    auto it = kfs_.find(id);
    return it == kfs_.end() ? nullptr : it->second.get();
}

MapPoint* Map::mp(int id) const {
    if (id < 0) return nullptr;
    auto it = mps_.find(id);
    return it == mps_.end() ? nullptr : it->second.get();
}

std::vector<KeyFrame*> Map::keyFrames() const {
    std::vector<KeyFrame*> out;
    for (const auto& e : kfs_)
        if (!e.second->bad) out.push_back(e.second.get());
    std::sort(out.begin(), out.end(), [](KeyFrame* a, KeyFrame* b) { return a->id < b->id; });
    return out;
}

std::vector<MapPoint*> Map::mapPoints() const {
    std::vector<MapPoint*> out;
    for (const auto& e : mps_)
        if (!e.second->bad) out.push_back(e.second.get());
    std::sort(out.begin(), out.end(), [](MapPoint* a, MapPoint* b) { return a->id < b->id; });
    return out;
}

size_t Map::numKeyFrames() const {
    size_t n = 0;
    for (const auto& e : kfs_)
        if (!e.second->bad) ++n;
    return n;
}

void Map::addObservation(MapPoint* p, KeyFrame* k, int idx) {
    if (p->obs.count(k->id)) return;
    p->obs[k->id] = idx;
    k->mps[idx] = p->id;
}

void Map::eraseObservation(MapPoint* p, KeyFrame* k) {
    auto it = p->obs.find(k->id);
    if (it == p->obs.end()) return;
    if (it->second >= 0 && it->second < (int)k->mps.size() && k->mps[it->second] == p->id) k->mps[it->second] = -1;
    p->obs.erase(it);
    if (p->refKf == k->id && !p->obs.empty()) p->refKf = p->obs.begin()->first;
    if (p->obs.size() <= 2) setBad(p);
}

void Map::setBad(MapPoint* p) {
    if (p->bad) return;
    p->bad = true;
    for (const auto& o : p->obs) {
        KeyFrame* k = kf(o.first);
        if (k && o.second < (int)k->mps.size() && k->mps[o.second] == p->id) k->mps[o.second] = -1;
    }
    p->obs.clear();
}

void Map::replace(MapPoint* p, MapPoint* q) {
    if (p->id == q->id || p->bad) return;
    std::map<int, int> obs = p->obs;
    p->obs.clear();
    p->bad = true;
    p->replacedBy = q->id;
    for (const auto& o : obs) {
        KeyFrame* k = kf(o.first);
        if (!k) continue;
        if (!q->obs.count(k->id)) {
            k->mps[o.second] = q->id;
            q->obs[k->id] = o.second;
        } else {
            k->mps[o.second] = -1;
        }
    }
    q->found += p->found;
    q->visible += p->visible;
    computeDescriptor(q);
}

void Map::updateConnections(KeyFrame* k) {
    std::map<int, int> counter;
    for (int id : k->mps) {
        MapPoint* p = mp(id);
        if (!p || p->bad) continue;
        for (const auto& o : p->obs)
            if (o.first != k->id) counter[o.first]++;
    }
    if (counter.empty()) return;
    int maxW = 0, maxK = -1;
    std::vector<std::pair<int, int>> pairs;
    for (const auto& c : counter) {
        KeyFrame* other = kf(c.first);
        if (!other || other->bad) continue;
        if (c.second > maxW) {
            maxW = c.second;
            maxK = c.first;
        }
        if (c.second >= 15) {
            pairs.push_back({c.second, c.first});
            other->conn[k->id] = c.second;
            std::vector<std::pair<int, int>> op;
            for (const auto& e : other->conn) op.push_back({e.second, e.first});
            std::sort(op.rbegin(), op.rend());
            other->ordered.clear();
            for (const auto& e : op) other->ordered.push_back(e.second);
        }
    }
    if (pairs.empty() && maxK >= 0) {
        pairs.push_back({maxW, maxK});
        KeyFrame* other = kf(maxK);
        other->conn[k->id] = maxW;
        other->ordered.insert(other->ordered.begin(), k->id);
    }
    std::sort(pairs.rbegin(), pairs.rend());
    k->conn.clear();
    k->ordered.clear();
    for (const auto& e : pairs) {
        k->conn[e.second] = e.first;
        k->ordered.push_back(e.second);
    }
    // Spanning tree: the first time a keyframe connects, its strongest neighbour is the parent.
    if (k->parent < 0 && k->id != origin && !k->ordered.empty()) {
        k->parent = k->ordered[0];
        kf(k->parent)->children.insert(k->id);
    }
}

void Map::computeDescriptor(MapPoint* p) {
    std::vector<const uint8_t*> ds;
    for (const auto& o : p->obs) {
        KeyFrame* k = kf(o.first);
        if (k && !k->bad) ds.push_back(k->f.desc.ptr<uint8_t>(o.second));
    }
    if (ds.empty()) return;
    // The descriptor with the least median distance to the others.
    size_t n = ds.size();
    int best = 0, bestMed = 1 << 30;
    std::vector<int> row(n);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) row[j] = orbDistance(ds[i], ds[j]);
        std::nth_element(row.begin(), row.begin() + (n - 1) / 2, row.end());
        int med = row[(n - 1) / 2];
        if (med < bestMed) {
            bestMed = med;
            best = (int)i;
        }
    }
    p->desc = cv::Mat(1, 32, CV_8U);
    std::copy(ds[best], ds[best] + 32, p->desc.ptr<uint8_t>());
}

void Map::updateNormalAndDepth(MapPoint* p) {
    if (p->obs.empty()) return;
    KeyFrame* ref = kf(p->refKf);
    if (!ref || ref->bad) {
        ref = kf(p->obs.begin()->first);
        p->refKf = ref->id;
    }
    Vec3 n = Vec3::Zero();
    int cnt = 0;
    for (const auto& o : p->obs) {
        KeyFrame* k = kf(o.first);
        if (!k) continue;
        n += (p->pos - k->centre()).normalized();
        ++cnt;
    }
    p->normal = n / std::max(1, cnt);
    auto it = p->obs.find(ref->id);
    if (it == p->obs.end()) return;
    double dist = (p->pos - ref->centre()).norm();
    int level = ref->f.kps[it->second].octave;
    float sf = ref->f.scaleFactors[level];
    int nLevels = (int)ref->f.scaleFactors.size();
    p->maxDist = (float)dist * sf;
    p->minDist = p->maxDist / ref->f.scaleFactors[nLevels - 1];
}

std::vector<KeyFrame*> Map::bestCovisibles(KeyFrame* k, int n) const {
    std::vector<KeyFrame*> out;
    for (int id : k->ordered) {
        KeyFrame* o = kf(id);
        if (o && !o->bad) out.push_back(o);
        if ((int)out.size() >= n) break;
    }
    return out;
}

std::vector<KeyFrame*> Map::covisiblesByWeight(KeyFrame* k, int minW) const {
    std::vector<KeyFrame*> out;
    for (int id : k->ordered) {
        auto it = k->conn.find(id);
        if (it == k->conn.end() || it->second < minW) continue;
        KeyFrame* o = kf(id);
        if (o && !o->bad) out.push_back(o);
    }
    return out;
}

std::vector<MapPoint*> Map::pointsOf(KeyFrame* k) const {
    std::vector<MapPoint*> out;
    for (int id : k->mps) {
        MapPoint* p = mp(id);
        if (p && !p->bad) out.push_back(p);
    }
    return out;
}

void Map::setBad(KeyFrame* k) {
    if (k->id == origin || k->bad) return;
    for (const auto& c : k->conn) {
        KeyFrame* o = kf(c.first);
        if (!o) continue;
        o->conn.erase(k->id);
        o->ordered.erase(std::remove(o->ordered.begin(), o->ordered.end(), k->id), o->ordered.end());
    }
    for (int id : k->mps) {
        MapPoint* p = mp(id);
        if (p && !p->bad) eraseObservation(p, k);
    }
    // Re-attach children to the spanning tree: each child picks the covisible keyframe among
    // the parent candidates (k's parent and already re-attached children) with most weight.
    std::set<int> candidates;
    if (k->parent >= 0) candidates.insert(k->parent);
    std::set<int> children = k->children;
    while (!children.empty()) {
        int bestChild = -1, bestParent = -1, bestW = -1;
        for (int c : children) {
            KeyFrame* ch = kf(c);
            if (!ch || ch->bad) continue;
            for (const auto& e : ch->conn)
                if (candidates.count(e.first) && e.second > bestW) {
                    bestW = e.second;
                    bestChild = c;
                    bestParent = e.first;
                }
        }
        if (bestChild < 0) break;
        KeyFrame* ch = kf(bestChild);
        ch->parent = bestParent;
        kf(bestParent)->children.insert(bestChild);
        candidates.insert(bestChild);
        children.erase(bestChild);
    }
    for (int c : children) {  // no covisible link: attach to k's parent
        KeyFrame* ch = kf(c);
        if (ch && k->parent >= 0) {
            ch->parent = k->parent;
            kf(k->parent)->children.insert(c);
        }
    }
    if (k->parent >= 0) kf(k->parent)->children.erase(k->id);
    k->conn.clear();
    k->ordered.clear();
    k->bad = true;
    db.erase(k->id, k->bow);
}

}  // namespace beacon

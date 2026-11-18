#include "matcher.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace beacon {

namespace {

constexpr int kHistBins = 30;

int bin(float rotDiff) {
    float r = rotDiff;
    while (r < 0) r += 360.f;
    while (r >= 360.f) r -= 360.f;
    int b = (int)std::round(r * kHistBins / 360.f);
    return b == kHistBins ? 0 : b;
}

bool inBounds(const Features& f, float u, float v) {
    return u >= f.minX && v >= f.minY && u < f.minX + Features::kCols * f.cellW &&
           v < f.minY + Features::kRows * f.cellH;
}

int predictLevel(const Features& f, float maxDist, float dist) {
    int n = (int)f.scaleFactors.size();
    float ratio = maxDist / dist;
    int l = (int)std::ceil(std::log(ratio) / std::log(n > 1 ? f.scaleFactors[1] : 1.2f));
    return std::max(0, std::min(n - 1, l));
}

}  // namespace

void rotationConsistency(const std::vector<float>& rotDiffs, const std::vector<int>& idx, std::vector<int>& reject) {
    std::vector<std::vector<int>> hist(kHistBins);
    for (size_t i = 0; i < rotDiffs.size(); ++i) hist[bin(rotDiffs[i])].push_back(idx[i]);
    int m[3] = {-1, -1, -1};
    size_t c[3] = {0, 0, 0};
    for (int b = 0; b < kHistBins; ++b) {
        size_t s = hist[b].size();
        if (s > c[0]) {
            c[2] = c[1]; m[2] = m[1];
            c[1] = c[0]; m[1] = m[0];
            c[0] = s; m[0] = b;
        } else if (s > c[1]) {
            c[2] = c[1]; m[2] = m[1];
            c[1] = s; m[1] = b;
        } else if (s > c[2]) {
            c[2] = s; m[2] = b;
        }
    }
    if (c[1] < 0.1 * c[0]) m[1] = m[2] = -1;
    else if (c[2] < 0.1 * c[0]) m[2] = -1;
    for (int b = 0; b < kHistBins; ++b) {
        if (b == m[0] || b == m[1] || b == m[2]) continue;
        for (int i : hist[b]) reject.push_back(i);
    }
}

bool Matcher::inFrustum(MapPoint* p, const SE3& tcw, const Features& f, float viewCosLimit, Projected& out) const {
    Vec3 pc = tcw * p->pos;
    float u, v;
    if (!cam_.project(pc, u, v) || !inBounds(f, u, v)) return false;
    Vec3 ow = -(tcw.linear().transpose() * tcw.translation());
    Vec3 po = p->pos - ow;
    float dist = (float)po.norm();
    if (dist < 0.8f * p->minDist || dist > 1.2f * p->maxDist) return false;
    float viewCos = (float)(po.dot(p->normal) / dist);
    if (viewCos < viewCosLimit) return false;
    out = Projected{p, u, v, predictLevel(f, p->maxDist, dist), viewCos};
    return true;
}

int Matcher::searchByProjection(Frame& F, const std::vector<Projected>& pts, float th) const {
    int n = 0;
    for (const Projected& pr : pts) {
        MapPoint* p = pr.p;
        if (p->bad) continue;
        float r = th * (pr.viewCos > 0.998f ? 2.5f : 4.0f) * F.f.scaleFactors[pr.level];
        std::vector<int> cand = F.f.inArea(pr.u, pr.v, r, pr.level - 1, pr.level);
        int best = 256, second = 256, bestIdx = -1, bestLevel = -1, secondLevel = -1;
        for (int i : cand) {
            if (F.mps[i] >= 0) {
                MapPoint* q = map_.mp(F.mps[i]);
                if (q && !q->obs.empty()) continue;
            }
            int d = orbDistance(p->desc.ptr<uint8_t>(), F.f.desc.ptr<uint8_t>(i));
            if (d < best) {
                second = best;
                secondLevel = bestLevel;
                best = d;
                bestLevel = F.f.kps[i].octave;
                bestIdx = i;
            } else if (d < second) {
                second = d;
                secondLevel = F.f.kps[i].octave;
            }
        }
        if (best <= kHigh) {
            if (bestLevel == secondLevel && best > 0.8f * second) continue;
            F.mps[bestIdx] = p->id;
            F.outlier[bestIdx] = 0;
            ++n;
        }
    }
    return n;
}

int Matcher::searchByProjection(Frame& F, const Frame& last, float th) const {
    int n = 0;
    std::vector<float> rot;
    std::vector<int> idx;
    for (size_t i = 0; i < last.mps.size(); ++i) {
        if (last.mps[i] < 0 || last.outlier[i]) continue;
        MapPoint* p = map_.mp(last.mps[i]);
        if (!p || p->bad) continue;
        Vec3 pc = F.tcw * p->pos;
        float u, v;
        if (!cam_.project(pc, u, v) || !inBounds(F.f, u, v)) continue;
        int oct = last.f.kps[i].octave;
        float r = th * F.f.scaleFactors[oct];
        std::vector<int> cand = F.f.inArea(u, v, r, std::max(0, oct - 1), oct + 1);
        int best = 256, bestIdx = -1;
        for (int j : cand) {
            if (F.mps[j] >= 0) {
                MapPoint* q = map_.mp(F.mps[j]);
                if (q && !q->obs.empty()) continue;
            }
            int d = orbDistance(p->desc.ptr<uint8_t>(), F.f.desc.ptr<uint8_t>(j));
            if (d < best) {
                best = d;
                bestIdx = j;
            }
        }
        if (best <= kHigh) {
            F.mps[bestIdx] = p->id;
            F.outlier[bestIdx] = 0;
            rot.push_back(last.f.kps[i].angle - F.f.kps[bestIdx].angle);
            idx.push_back(bestIdx);
            ++n;
        }
    }
    std::vector<int> rej;
    rotationConsistency(rot, idx, rej);
    for (int j : rej) {
        F.mps[j] = -1;
        --n;
    }
    return n;
}

int Matcher::searchByProjection(Frame& F, KeyFrame* kf, const std::vector<int>& found, float th, int orbDist) const {
    int n = 0;
    std::vector<float> rot;
    std::vector<int> idx;
    std::vector<char> already(1, 0);
    for (int id : kf->mps) {
        MapPoint* p = map_.mp(id);
        if (!p || p->bad || std::find(found.begin(), found.end(), id) != found.end()) continue;
        Projected pr;
        if (!inFrustum(p, F.tcw, F.f, -1.0f, pr)) continue;
        float r = th * F.f.scaleFactors[pr.level];
        std::vector<int> cand = F.f.inArea(pr.u, pr.v, r, pr.level - 1, pr.level + 1);
        int best = 256, bestIdx = -1;
        for (int j : cand) {
            if (F.mps[j] >= 0) continue;
            int d = orbDistance(p->desc.ptr<uint8_t>(), F.f.desc.ptr<uint8_t>(j));
            if (d < best) {
                best = d;
                bestIdx = j;
            }
        }
        if (best <= orbDist) {
            F.mps[bestIdx] = p->id;
            rot.push_back(kf->f.kps[p->obs.count(kf->id) ? p->obs.at(kf->id) : 0].angle - F.f.kps[bestIdx].angle);
            idx.push_back(bestIdx);
            ++n;
        }
    }
    std::vector<int> rej;
    rotationConsistency(rot, idx, rej);
    for (int j : rej) {
        F.mps[j] = -1;
        --n;
    }
    return n;
}

int Matcher::searchByBoW(KeyFrame* kf, Frame& F, std::vector<int>& out) const {
    out.assign(F.f.size(), -1);
    int n = 0;
    std::vector<float> rot;
    std::vector<int> idx;
    auto ik = kf->fv.begin();
    auto iff = F.fv.begin();
    while (ik != kf->fv.end() && iff != F.fv.end()) {
        if (ik->first < iff->first) {
            ++ik;
            continue;
        }
        if (iff->first < ik->first) {
            ++iff;
            continue;
        }
        for (int i : ik->second) {
            MapPoint* p = map_.mp(kf->mps[i]);
            if (!p || p->bad) continue;
            const uint8_t* d1 = kf->f.desc.ptr<uint8_t>(i);
            int best = 256, second = 256, bestIdx = -1;
            for (int j : iff->second) {
                if (out[j] >= 0) continue;
                int d = orbDistance(d1, F.f.desc.ptr<uint8_t>(j));
                if (d < best) {
                    second = best;
                    best = d;
                    bestIdx = j;
                } else if (d < second) {
                    second = d;
                }
            }
            if (best <= kLow && best < 0.7f * second) {
                out[bestIdx] = p->id;
                rot.push_back(kf->f.kps[i].angle - F.f.kps[bestIdx].angle);
                idx.push_back(bestIdx);
                ++n;
            }
        }
        ++ik;
        ++iff;
    }
    std::vector<int> rej;
    rotationConsistency(rot, idx, rej);
    for (int j : rej) {
        out[j] = -1;
        --n;
    }
    return n;
}

int Matcher::searchByBoW(KeyFrame* k1, KeyFrame* k2, std::vector<int>& out) const {
    out.assign(k1->f.size(), -1);
    std::vector<char> used(k2->f.size(), 0);
    int n = 0;
    std::vector<float> rot;
    std::vector<int> idx;
    auto i1 = k1->fv.begin();
    auto i2 = k2->fv.begin();
    while (i1 != k1->fv.end() && i2 != k2->fv.end()) {
        if (i1->first < i2->first) {
            ++i1;
            continue;
        }
        if (i2->first < i1->first) {
            ++i2;
            continue;
        }
        for (int a : i1->second) {
            MapPoint* p1 = map_.mp(k1->mps[a]);
            if (!p1 || p1->bad) continue;
            int best = 256, second = 256, bestIdx = -1;
            for (int b : i2->second) {
                if (used[b]) continue;
                MapPoint* p2 = map_.mp(k2->mps[b]);
                if (!p2 || p2->bad) continue;
                int d = orbDistance(k1->f.desc.ptr<uint8_t>(a), k2->f.desc.ptr<uint8_t>(b));
                if (d < best) {
                    second = best;
                    best = d;
                    bestIdx = b;
                } else if (d < second) {
                    second = d;
                }
            }
            if (best < kLow && best < 0.75f * second) {
                out[a] = k2->mps[bestIdx];
                used[bestIdx] = 1;
                rot.push_back(k1->f.kps[a].angle - k2->f.kps[bestIdx].angle);
                idx.push_back(a);
                ++n;
            }
        }
        ++i1;
        ++i2;
    }
    std::vector<int> rej;
    rotationConsistency(rot, idx, rej);
    for (int a : rej) {
        out[a] = -1;
        --n;
    }
    return n;
}

int Matcher::searchForInitialization(const Features& f1, const Features& f2, std::vector<cv::Point2f>& prev,
                                     std::vector<int>& m12, int window) const {
    m12.assign(f1.size(), -1);
    std::vector<int> dist21(f2.size(), std::numeric_limits<int>::max());
    std::vector<int> m21(f2.size(), -1);
    std::vector<float> rot;
    std::vector<int> idx;
    int n = 0;
    for (size_t i1 = 0; i1 < f1.size(); ++i1) {
        if (f1.kps[i1].octave > 0) continue;
        std::vector<int> cand = f2.inArea(prev[i1].x, prev[i1].y, (float)window, 0, 0);
        int best = 256, second = 256, bestIdx = -1;
        for (int i2 : cand) {
            int d = orbDistance(f1.desc.ptr<uint8_t>((int)i1), f2.desc.ptr<uint8_t>(i2));
            if (dist21[i2] <= d) continue;
            if (d < best) {
                second = best;
                best = d;
                bestIdx = i2;
            } else if (d < second) {
                second = d;
            }
        }
        if (best <= kLow && best < 0.9f * second) {
            if (m21[bestIdx] >= 0) {
                m12[m21[bestIdx]] = -1;
                --n;
            }
            m12[i1] = bestIdx;
            m21[bestIdx] = (int)i1;
            dist21[bestIdx] = best;
            ++n;
        }
    }
    for (size_t i1 = 0; i1 < f1.size(); ++i1) {
        if (m12[i1] < 0) continue;
        rot.push_back(f1.kps[i1].angle - f2.kps[m12[i1]].angle);
        idx.push_back((int)i1);
    }
    std::vector<int> rej;
    rotationConsistency(rot, idx, rej);
    for (int i1 : rej) {
        m12[i1] = -1;
        --n;
    }
    for (size_t i1 = 0; i1 < f1.size(); ++i1)
        if (m12[i1] >= 0) prev[i1] = f2.un[m12[i1]];
    return n;
}

int Matcher::searchForTriangulation(KeyFrame* k1, KeyFrame* k2, std::vector<std::pair<int, int>>& pairs) const {
    pairs.clear();
    Mat3 K;
    K << cam_.fx, 0, cam_.cx, 0, cam_.fy, cam_.cy, 0, 0, 1;
    Mat3 r1 = k1->tcw.linear(), r2 = k2->tcw.linear();
    Vec3 t1 = k1->tcw.translation(), t2 = k2->tcw.translation();
    Mat3 r12 = r1 * r2.transpose();
    Vec3 t12 = -r12 * t2 + t1;
    Mat3 tx;
    tx << 0, -t12.z(), t12.y(), t12.z(), 0, -t12.x(), -t12.y(), t12.x(), 0;
    Mat3 F12 = K.transpose().inverse() * tx * r12 * K.inverse();
    Vec3 c1in2 = k2->tcw * k1->centre();
    float eu = (float)(cam_.fx * c1in2.x() / c1in2.z() + cam_.cx);
    float ev = (float)(cam_.fy * c1in2.y() / c1in2.z() + cam_.cy);
    std::vector<char> used2(k2->f.size(), 0);
    std::vector<int> m12(k1->f.size(), -1);
    auto i1 = k1->fv.begin();
    auto i2 = k2->fv.begin();
    while (i1 != k1->fv.end() && i2 != k2->fv.end()) {
        if (i1->first < i2->first) {
            ++i1;
            continue;
        }
        if (i2->first < i1->first) {
            ++i2;
            continue;
        }
        for (int a : i1->second) {
            if (k1->mps[a] >= 0 && map_.mp(k1->mps[a]) && !map_.mp(k1->mps[a])->bad) continue;
            const cv::Point2f& x1 = k1->f.un[a];
            Eigen::Vector3d l = Eigen::Vector3d(x1.x, x1.y, 1).transpose() * F12;
            double nl = l.x() * l.x() + l.y() * l.y();
            if (nl < 1e-12) continue;
            int best = kLow, bestIdx = -1;
            for (int b : i2->second) {
                if (used2[b]) continue;
                if (k2->mps[b] >= 0 && map_.mp(k2->mps[b]) && !map_.mp(k2->mps[b])->bad) continue;
                int d = orbDistance(k1->f.desc.ptr<uint8_t>(a), k2->f.desc.ptr<uint8_t>(b));
                if (d > best) continue;
                const cv::Point2f& x2 = k2->f.un[b];
                float sf = k2->f.scaleFactors[k2->f.kps[b].octave];
                float dx = eu - x2.x, dy = ev - x2.y;
                if (dx * dx + dy * dy < 100 * sf) continue;
                double num = l.x() * x2.x + l.y() * x2.y + l.z();
                if (num * num / nl > 3.84 * sf * sf) continue;
                best = d;
                bestIdx = b;
            }
            if (bestIdx >= 0) {
                used2[bestIdx] = 1;
                m12[a] = bestIdx;
            }
        }
        ++i1;
        ++i2;
    }
    std::vector<float> rot;
    std::vector<int> idx;
    for (size_t a = 0; a < m12.size(); ++a) {
        if (m12[a] < 0) continue;
        rot.push_back(k1->f.kps[a].angle - k2->f.kps[m12[a]].angle);
        idx.push_back((int)a);
    }
    std::vector<int> rej;
    rotationConsistency(rot, idx, rej);
    for (int a : rej) m12[a] = -1;
    for (size_t a = 0; a < m12.size(); ++a)
        if (m12[a] >= 0) pairs.push_back({(int)a, m12[a]});
    return (int)pairs.size();
}

int Matcher::fuse(KeyFrame* kf, const std::vector<MapPoint*>& pts, float th, Map& map) const {
    int n = 0;
    Vec3 ow = kf->centre();
    for (MapPoint* p : pts) {
        if (!p || p->bad || p->obs.count(kf->id)) continue;
        Vec3 pc = kf->tcw * p->pos;
        float u, v;
        if (!cam_.project(pc, u, v) || !inBounds(kf->f, u, v)) continue;
        Vec3 po = p->pos - ow;
        float dist = (float)po.norm();
        if (dist < 0.8f * p->minDist || dist > 1.2f * p->maxDist) continue;
        if (po.dot(p->normal) < 0.5 * dist) continue;
        int level = predictLevel(kf->f, p->maxDist, dist);
        float r = th * kf->f.scaleFactors[level];
        std::vector<int> cand = kf->f.inArea(u, v, r, level - 1, level);
        int best = 256, bestIdx = -1;
        for (int i : cand) {
            int kl = kf->f.kps[i].octave;
            float ex = u - kf->f.un[i].x, ey = v - kf->f.un[i].y;
            if ((ex * ex + ey * ey) * kf->f.invLevelSigma2[kl] > 5.99f) continue;
            int d = orbDistance(p->desc.ptr<uint8_t>(), kf->f.desc.ptr<uint8_t>(i));
            if (d < best) {
                best = d;
                bestIdx = i;
            }
        }
        if (best > kLow) continue;
        MapPoint* q = map.mp(kf->mps[bestIdx]);
        if (q && !q->bad) {
            if (q->obs.size() > p->obs.size()) map.replace(p, q);
            else map.replace(q, p);
        } else {
            map.addObservation(p, kf, bestIdx);
        }
        ++n;
    }
    return n;
}

int Matcher::searchByProjectionSim(KeyFrame* kf, const Mat3& sR, const Vec3& t, double s,
                                   const std::vector<MapPoint*>& pts, std::vector<int>& matched, int th) const {
    int n = 0;
    Mat3 R = sR / s;
    Vec3 tc = t / s;  // rigid part (camera centre is scale-free)
    Vec3 ow = -R.transpose() * tc;
    for (MapPoint* p : pts) {
        if (!p || p->bad || std::find(matched.begin(), matched.end(), p->id) != matched.end()) continue;
        Vec3 pc = R * p->pos + tc;
        float u, v;
        if (!cam_.project(pc, u, v) || !inBounds(kf->f, u, v)) continue;
        Vec3 po = p->pos - ow;
        float dist = (float)po.norm();
        if (dist < 0.8f * p->minDist || dist > 1.2f * p->maxDist) continue;
        if (po.dot(p->normal) < 0.5 * dist) continue;
        int level = predictLevel(kf->f, p->maxDist, dist);
        float r = (float)th * kf->f.scaleFactors[level];
        std::vector<int> cand = kf->f.inArea(u, v, r, level - 1, level);
        int best = 256, bestIdx = -1;
        for (int i : cand) {
            if (matched[i] >= 0) continue;
            int d = orbDistance(p->desc.ptr<uint8_t>(), kf->f.desc.ptr<uint8_t>(i));
            if (d < best) {
                best = d;
                bestIdx = i;
            }
        }
        if (best <= kLow) {
            matched[bestIdx] = p->id;
            ++n;
        }
    }
    return n;
}

}  // namespace beacon

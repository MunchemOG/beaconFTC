#include "params.h"

#include <cmath>
#include <functional>
#include <map>
#include <nlohmann/json.hpp>

namespace beacon {

namespace {

using json = nlohmann::json;

bool readDouble(const json& v, double& out, const std::string& key, std::string& err) {
    if (!v.is_number() || !std::isfinite(v.get<double>())) {
        err = "params: \"" + key + "\" must be a finite number";
        return false;
    }
    out = v.get<double>();
    return true;
}

bool readInt(const json& v, int& out, const std::string& key, std::string& err) {
    if (!v.is_number_integer()) {
        err = "params: \"" + key + "\" must be an integer";
        return false;
    }
    out = v.get<int>();
    return true;
}

bool readBool(const json& v, bool& out, const std::string& key, std::string& err) {
    if (!v.is_boolean()) {
        err = "params: \"" + key + "\" must be true or false";
        return false;
    }
    out = v.get<bool>();
    return true;
}

}  // namespace

bool Params::parse(const char* text, Params& p, std::string& err) {
    p = Params();
    if (text == nullptr || text[0] == '\0') return true;
    json doc = json::parse(text, nullptr, /*allow_exceptions=*/false, /*ignore_comments=*/true);
    if (doc.is_discarded() || !doc.is_object()) {
        err = "params: not a JSON object";
        return false;
    }
    using Reader = std::function<bool(const json&, const std::string&)>;
    std::string* e = &err;
    auto d = [e](double& f) {
        double* pf = &f;
        return Reader([e, pf](const json& v, const std::string& k) { return readDouble(v, *pf, k, *e); });
    };
    auto i = [e](int& f) {
        int* pf = &f;
        return Reader([e, pf](const json& v, const std::string& k) { return readInt(v, *pf, k, *e); });
    };
    auto b = [e](bool& f) {
        bool* pf = &f;
        return Reader([e, pf](const json& v, const std::string& k) { return readBool(v, *pf, k, *e); });
    };
    std::map<std::string, Reader> readers = {
        {"synchronous", b(p.synchronous)},
        {"lostAfterS", d(p.lostAfterS)},
        {"odomMaxExtrapolateS", d(p.odomMaxExtrapolateS)},
        {"odomHistoryS", d(p.odomHistoryS)},
        {"startSigmaXY", d(p.startSigmaXY)},
        {"startSigmaHeading", d(p.startSigmaHeading)},
        {"driftPerMetre", d(p.driftPerMetre)},
        {"driftHeadingPerRad", d(p.driftHeadingPerRad)},
        {"driftHeadingPerS", d(p.driftHeadingPerS)},
        {"noOdomSigmaXY", d(p.noOdomSigmaXY)},
        {"noOdomSigmaHeading", d(p.noOdomSigmaHeading)},
        {"orbFeatures", i(p.orbFeatures)},
        {"orbScaleFactor", d(p.orbScaleFactor)},
        {"orbLevels", i(p.orbLevels)},
        {"orbFastThreshold", i(p.orbFastThreshold)},
        {"orbMinFastThreshold", i(p.orbMinFastThreshold)},
        {"edgeSampleStepPx", d(p.edgeSampleStepPx)},
        {"edgeSearchMaxPx", d(p.edgeSearchMaxPx)},
        {"edgeSearchMinPx", d(p.edgeSearchMinPx)},
        {"edgeMinGradient", d(p.edgeMinGradient)},
        {"edgeHuberPx", d(p.edgeHuberPx)},
        {"edgeSigmaPx", d(p.edgeSigmaPx)},
        {"edgeCorrelationInflation", d(p.edgeCorrelationInflation)},
        {"edgeMaxRangeM", d(p.edgeMaxRangeM)},
        {"gateMaxOmegaRadS", d(p.gateMaxOmegaRadS)},
        {"gateMaxSpeedMps", d(p.gateMaxSpeedMps)},
        {"gateMinInliers", i(p.gateMinInliers)},
        {"gateMinInlierRatio", d(p.gateMinInlierRatio)},
        {"gateMaxSigmaXY", d(p.gateMaxSigmaXY)},
        {"gateMaxSigmaHeading", d(p.gateMaxSigmaHeading)},
        {"gateMaxChi2", d(p.gateMaxChi2)},
        {"usePoints", b(p.usePoints)},
        {"pointMaxRangeM", d(p.pointMaxRangeM)},
        {"pointSurfaceTolM", d(p.pointSurfaceTolM)},
        {"exploreInitMinMatches", i(p.exploreInitMinMatches)},
        {"exploreInitMinParallaxDeg", d(p.exploreInitMinParallaxDeg)},
        {"exploreLoopClosing", b(p.exploreLoopClosing)},
        {"exploreLocalWindow", i(p.exploreLocalWindow)},
        {"exploreGlobalBA", b(p.exploreGlobalBA)},
        {"exploreMaxFramesBetweenKf", i(p.exploreMaxFramesBetweenKf)},
        {"exploreMinFramesBetweenKf", i(p.exploreMinFramesBetweenKf)},
    };
    for (auto it = doc.begin(); it != doc.end(); ++it) {
        const std::string& key = it.key();
        if (key == "mode") {
            if (it->is_string() && it->get<std::string>() == "field") {
                p.mode = Mode::Field;
            } else if (it->is_string() && it->get<std::string>() == "explore") {
                p.mode = Mode::Explore;
            } else {
                err = "params: \"mode\" must be \"field\" or \"explore\"";
                return false;
            }
            continue;
        }
        if (key == "vocabularyPath") {
            if (!it->is_string()) {
                err = "params: \"vocabularyPath\" must be a string";
                return false;
            }
            p.vocabularyPath = it->get<std::string>();
            continue;
        }
        auto r = readers.find(key);
        if (r == readers.end()) {
            err = "params: unknown key \"" + key + "\"";
            return false;
        }
        if (!r->second(*it, key)) return false;
    }
    if (p.orbFeatures < 50 || p.orbLevels < 1 || p.orbScaleFactor <= 1.0) {
        err = "params: ORB settings out of range";
        return false;
    }
    if (p.lostAfterS <= 0 || p.odomHistoryS <= 0) {
        err = "params: lostAfterS and odomHistoryS must be > 0";
        return false;
    }
    return true;
}

}  // namespace beacon

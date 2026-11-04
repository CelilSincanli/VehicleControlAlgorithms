#include "path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

AdaptivePurePursuitFileConfig LoadAdaptivePurePursuitConfig(const std::string& path) {
    AdaptivePurePursuitFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    // Every value is clamped into the slider range (tuning_limits.hpp); values
    // that were not numbers or out of range are listed in cfg.warnings.
    ConfigReader r(src);
    r.Float("min_lookahead",  cfg.min_lookahead, tuning::kAPPMinLookahead);
    r.Float("max_lookahead",  cfg.max_lookahead, tuning::kAPPMaxLookahead);
    r.Float("speed_gain",     cfg.speed_gain, tuning::kAPPSpeedGain);
    r.Float("curvature_gain", cfg.curvature_gain, tuning::kAPPCurvatureGain);
    r.Float("max_speed",      cfg.max_speed_mps, tuning::kMaxSpeed);
    r.Int  ("search_window",  cfg.search_window, tuning::kSearchWindow);
    cfg.warnings = r.warnings();
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

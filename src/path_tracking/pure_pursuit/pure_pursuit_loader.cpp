#include "path_tracking/pure_pursuit/pure_pursuit_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

PurePursuitFileConfig LoadPurePursuitConfig(const std::string& path) {
    PurePursuitFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    // Every value is clamped into the slider range (tuning_limits.hpp); values
    // that were not numbers or out of range are listed in cfg.warnings.
    ConfigReader r(src);
    r.Float("lookahead_distance", cfg.lookahead_distance, tuning::kPPLookaheadDist);
    r.Float("lookahead_gain",     cfg.lookahead_gain, tuning::kPPLookaheadGain);
    r.Float("max_speed",          cfg.max_speed_mps, tuning::kMaxSpeed);
    r.Int  ("search_window",      cfg.search_window, tuning::kSearchWindow);
    cfg.warnings = r.warnings();
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

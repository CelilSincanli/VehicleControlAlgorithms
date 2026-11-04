#include "path_tracking/stanley/stanley_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

StanleyFileConfig LoadStanleyConfig(const std::string& path) {
    StanleyFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    // Every value is clamped into the slider range (tuning_limits.hpp); values
    // that were not numbers or out of range are listed in cfg.warnings.
    ConfigReader r(src);
    r.Float("stanley_gain",  cfg.stanley_gain, tuning::kStanleyGain);
    r.Float("min_speed",     cfg.min_speed, tuning::kStanleyMinSpeed);
    r.Float("max_speed",     cfg.max_speed_mps, tuning::kMaxSpeed);
    r.Float("max_delta",     cfg.max_delta, tuning::kMaxDelta);
    r.Int  ("search_window", cfg.search_window, tuning::kSearchWindow);
    cfg.warnings = r.warnings();
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

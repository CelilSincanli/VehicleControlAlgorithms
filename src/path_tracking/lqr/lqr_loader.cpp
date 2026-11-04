#include "path_tracking/lqr/lqr_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

LqrFileConfig LoadLqrConfig(const std::string& path) {
    LqrFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    // Every value is clamped into the slider range (tuning_limits.hpp); values
    // that were not numbers or out of range are listed in cfg.warnings.
    ConfigReader r(src);
    r.Float("q0",              cfg.q0, tuning::kLQRWeight);
    r.Float("q1",              cfg.q1, tuning::kLQRWeight);
    r.Float("q2",              cfg.q2, tuning::kLQRWeight);
    r.Float("q3",              cfg.q3, tuning::kLQRWeight);
    r.Float("q4",              cfg.q4, tuning::kLQRWeight);
    r.Float("q5",              cfg.q5, tuning::kLQRWeight);
    r.Float("r_steering",      cfg.r_steering, tuning::kLQRWeight);
    r.Float("r_acceleration",  cfg.r_acceleration, tuning::kLQRWeight);
    r.Float("r_scale",         cfg.r_scale, tuning::kLQRWeight);
    r.Float("max_speed",       cfg.max_speed, tuning::kMaxSpeed);
    r.Int  ("dare_iterations", cfg.dare_iterations, tuning::kLQRDareIterations);
    r.Float("dare_threshold",  cfg.dare_threshold, tuning::kLQRDareThreshold);
    r.Int  ("search_window",   cfg.search_window, tuning::kSearchWindow);
    cfg.warnings = r.warnings();
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

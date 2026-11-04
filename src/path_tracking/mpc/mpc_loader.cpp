#include "path_tracking/mpc/mpc_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

MpcFileConfig LoadMpcConfig(const std::string& path) {
    MpcFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    // Every value is clamped into the slider range (tuning_limits.hpp); values
    // that were not numbers or out of range are listed in cfg.warnings.
    ConfigReader r(src);
    r.Int  ("horizon_N",     cfg.horizon_N, tuning::kMPCHorizon);
    r.Int  ("iterations",    cfg.iterations, tuning::kMPCIterations);
    r.Float("rollout_dt",    cfg.rollout_dt, tuning::kMPCRolloutDt);
    r.Float("max_speed",     cfg.max_speed_mps, tuning::kMaxSpeed);
    r.Float("w_lat",         cfg.w_lat, tuning::kMPCWeight);
    r.Float("w_heading",     cfg.w_heading, tuning::kMPCWeight);
    r.Float("w_control",     cfg.w_control, tuning::kMPCWControl);
    r.Float("w_rate",        cfg.w_rate, tuning::kMPCWRate);
    r.Int  ("search_window", cfg.search_window, tuning::kSearchWindow);
    r.Float("max_delta",     cfg.max_delta, tuning::kMaxDelta);
    cfg.warnings = r.warnings();
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

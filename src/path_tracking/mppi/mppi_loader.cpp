#include "path_tracking/mppi/mppi_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

MppiFileConfig LoadMppiConfig(const std::string& path) {
    MppiFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    // Every value is clamped into the slider range (tuning_limits.hpp); values
    // that were not numbers or out of range are listed in cfg.warnings.
    ConfigReader r(src);
    r.Int  ("horizon_T",     cfg.horizon_T, tuning::kMPPIHorizon);
    r.Int  ("num_samples_K", cfg.num_samples_K, tuning::kMPPISamples);
    r.Float("lambda",        cfg.lambda, tuning::kMPPILambda);
    r.Float("sigma_steer",   cfg.sigma_steer, tuning::kMPPISigma);
    r.Float("rollout_dt",    cfg.rollout_dt, tuning::kMPPIRolloutDt);
    r.Float("max_speed",     cfg.max_speed_mps, tuning::kMaxSpeed);
    r.Float("w_lat",         cfg.w_lat, tuning::kMPPIWeight);
    r.Float("w_heading",     cfg.w_heading, tuning::kMPPIWeight);
    r.Int  ("search_window", cfg.search_window, tuning::kSearchWindow);
    r.Float("max_delta",     cfg.max_delta, tuning::kMaxDelta);
    r.Int  ("rng_seed",      cfg.rng_seed, tuning::kMPPISeed);
    cfg.warnings = r.warnings();
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

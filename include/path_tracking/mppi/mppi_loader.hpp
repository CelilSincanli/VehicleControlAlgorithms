#pragma once
#include <string>
#include <vector>

namespace path_tracking {

struct MppiFileConfig {
    int   horizon_T     = 20;
    int   num_samples_K = 128;
    float lambda        = 1.0f;
    float sigma_steer   = 0.2f;
    float rollout_dt    = 0.15f;
    float max_speed_mps = 5.0f;
    float w_lat         = 1.0f;
    float w_heading     = 0.5f;
    int   search_window = 40;
    float max_delta     = 1.0f;
    int   rng_seed      = 42;
    std::vector<std::string> warnings;   // values clamped or not parsable
    bool  loaded        = false;
};

MppiFileConfig LoadMppiConfig(const std::string& path);

} // namespace path_tracking

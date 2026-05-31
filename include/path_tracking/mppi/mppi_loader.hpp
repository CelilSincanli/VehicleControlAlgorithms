#pragma once
#include <string>

namespace path_tracking {

struct MppiFileConfig {
    int   horizon_T     = 15;
    int   num_samples_K = 128;
    float lambda        = 1.0f;
    float sigma_steer   = 0.3f;
    float rollout_dt    = 0.1f;
    float max_speed_mps = 5.0f;
    float w_lat         = 1.0f;
    float w_heading     = 0.5f;
    int   search_window = 40;
    float max_delta     = 1.0f;
    int   rng_seed      = 42;
    bool  loaded        = false;
};

MppiFileConfig LoadMppiConfig(const std::string& path);

} // namespace path_tracking

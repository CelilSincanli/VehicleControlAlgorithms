#pragma once
#include <string>
#include <vector>

namespace path_tracking {

struct MpcFileConfig {
    int   horizon_N     = 15;
    int   iterations    = 30;
    float rollout_dt    = 0.1f;
    float max_speed_mps = 5.0f;
    float w_lat         = 1.0f;
    float w_heading     = 0.5f;
    float w_control     = 0.05f;
    float w_rate        = 0.1f;
    int   search_window = 40;
    float max_delta     = 1.0f;
    std::vector<std::string> warnings;   // values clamped or not parsable
    bool  loaded        = false;
};

MpcFileConfig LoadMpcConfig(const std::string& path);

} // namespace path_tracking

#pragma once
#include <string>

namespace path_tracking {

struct MpcFileConfig {
    int   horizon_N     = 15;
    int   iterations    = 10;
    float learning_rate = 0.08f;
    float fd_step       = 0.01f;
    float rollout_dt    = 0.1f;
    float max_speed_mps = 5.0f;
    float w_lat         = 1.0f;
    float w_heading     = 0.5f;
    float w_control     = 0.05f;
    bool  loaded        = false;
};

MpcFileConfig LoadMpcConfig(const std::string& path);

} // namespace path_tracking

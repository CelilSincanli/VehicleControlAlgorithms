#pragma once
#include <string>

namespace path_tracking {

struct AdaptivePurePursuitFileConfig {
    float min_lookahead  = 1.5f;
    float max_lookahead  = 8.0f;
    float speed_gain     = 0.5f;
    float curvature_gain = 3.0f;
    float max_speed_mps  = 5.0f;
    bool  loaded         = false;
};

AdaptivePurePursuitFileConfig LoadAdaptivePurePursuitConfig(const std::string& path);

} // namespace path_tracking

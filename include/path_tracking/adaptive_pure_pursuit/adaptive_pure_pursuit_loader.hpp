#pragma once
#include <string>
#include <vector>

namespace path_tracking {

struct AdaptivePurePursuitFileConfig {
    float min_lookahead  = 1.5f;
    float max_lookahead  = 8.0f;
    float speed_gain     = 0.5f;
    float curvature_gain = 3.0f;
    float max_speed_mps  = 5.0f;
    int   search_window  = 40;
    std::vector<std::string> warnings;   // values clamped or not parsable
    bool  loaded         = false;
};

AdaptivePurePursuitFileConfig LoadAdaptivePurePursuitConfig(const std::string& path);

} // namespace path_tracking

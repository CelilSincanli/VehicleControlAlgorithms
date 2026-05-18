#pragma once
#include <string>

namespace path_tracking {

struct PurePursuitFileConfig {
    float lookahead_distance = 2.0f;
    float lookahead_gain     = 0.3f;
    float max_speed_mps      = 3.0f;
    bool  loaded             = false;
};

PurePursuitFileConfig LoadPurePursuitConfig(const std::string& path);

} // namespace path_tracking

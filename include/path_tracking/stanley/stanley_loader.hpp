#pragma once
#include <string>

namespace path_tracking {

struct StanleyFileConfig {
    float stanley_gain  = 0.5f;
    float min_speed     = 0.1f;
    float max_speed_mps = 5.0f;
    bool  loaded        = false;
};

StanleyFileConfig LoadStanleyConfig(const std::string& path);

} // namespace path_tracking

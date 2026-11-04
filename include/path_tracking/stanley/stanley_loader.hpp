#pragma once
#include <string>
#include <vector>

namespace path_tracking {

struct StanleyFileConfig {
    float stanley_gain  = 0.5f;
    float min_speed     = 0.1f;
    float max_speed_mps = 5.0f;
    float max_delta     = 1.0f;
    int   search_window = 40;
    std::vector<std::string> warnings;   // values clamped or not parsable
    bool  loaded        = false;
};

StanleyFileConfig LoadStanleyConfig(const std::string& path);

} // namespace path_tracking

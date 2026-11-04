#pragma once
#include <string>
#include <vector>

namespace path_tracking {

struct LqrFileConfig {
    float q0             = 1.0f;
    float q1             = 1.0f;
    float q2             = 1.0f;
    float q3             = 1.0f;
    float q4             = 1.0f;
    float q5             = 1.0f;
    float r_steering     = 1.0f;
    float r_acceleration = 1.0f;
    float r_scale        = 4.0f;
    float max_speed      = 5.0f;
    int   dare_iterations = 150;
    float dare_threshold  = 0.01f;
    int   search_window   = 40;
    std::vector<std::string> warnings;   // values clamped or not parsable
    bool  loaded          = false;
};

LqrFileConfig LoadLqrConfig(const std::string& path);

} // namespace path_tracking

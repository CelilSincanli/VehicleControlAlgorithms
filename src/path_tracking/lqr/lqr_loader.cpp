#include "path_tracking/lqr/lqr_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

LqrFileConfig LoadLqrConfig(const std::string& path) {
    LqrFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    readFloat(src, "q0",              cfg.q0);
    readFloat(src, "q1",              cfg.q1);
    readFloat(src, "q2",              cfg.q2);
    readFloat(src, "q3",              cfg.q3);
    readFloat(src, "q4",              cfg.q4);
    readFloat(src, "r_steering",      cfg.r_steering);
    readFloat(src, "r_acceleration",  cfg.r_acceleration);
    readFloat(src, "r_scale",         cfg.r_scale);
    readFloat(src, "time_step",       cfg.time_step);
    readFloat(src, "max_speed",       cfg.max_speed);
    readInt  (src, "dare_iterations", cfg.dare_iterations);
    readFloat(src, "dare_threshold",  cfg.dare_threshold);
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

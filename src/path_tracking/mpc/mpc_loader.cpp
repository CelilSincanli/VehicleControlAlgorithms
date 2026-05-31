#include "path_tracking/mpc/mpc_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

MpcFileConfig LoadMpcConfig(const std::string& path) {
    MpcFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    readInt  (src, "horizon_N",     cfg.horizon_N);
    readInt  (src, "iterations",    cfg.iterations);
    readFloat(src, "learning_rate", cfg.learning_rate);
    readFloat(src, "fd_step",       cfg.fd_step);
    readFloat(src, "rollout_dt",    cfg.rollout_dt);
    readFloat(src, "max_speed",     cfg.max_speed_mps);
    readFloat(src, "w_lat",         cfg.w_lat);
    readFloat(src, "w_heading",     cfg.w_heading);
    readFloat(src, "w_control",     cfg.w_control);
    readInt  (src, "search_window", cfg.search_window);
    readFloat(src, "max_delta",     cfg.max_delta);
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

#include "path_tracking/mppi/mppi_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

MppiFileConfig LoadMppiConfig(const std::string& path) {
    MppiFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    readInt  (src, "horizon_T",     cfg.horizon_T);
    readInt  (src, "num_samples_K", cfg.num_samples_K);
    readFloat(src, "lambda",        cfg.lambda);
    readFloat(src, "sigma_steer",   cfg.sigma_steer);
    readFloat(src, "rollout_dt",    cfg.rollout_dt);
    readFloat(src, "max_speed",     cfg.max_speed_mps);
    readFloat(src, "w_lat",         cfg.w_lat);
    readFloat(src, "w_heading",     cfg.w_heading);
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

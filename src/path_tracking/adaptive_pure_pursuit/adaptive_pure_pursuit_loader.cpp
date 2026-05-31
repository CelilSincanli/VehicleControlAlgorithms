#include "path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

AdaptivePurePursuitFileConfig LoadAdaptivePurePursuitConfig(const std::string& path) {
    AdaptivePurePursuitFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    readFloat(src, "min_lookahead",  cfg.min_lookahead);
    readFloat(src, "max_lookahead",  cfg.max_lookahead);
    readFloat(src, "speed_gain",     cfg.speed_gain);
    readFloat(src, "curvature_gain", cfg.curvature_gain);
    readFloat(src, "max_speed",      cfg.max_speed_mps);
    readInt  (src, "search_window",  cfg.search_window);
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

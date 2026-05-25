#include "path_tracking/pure_pursuit/pure_pursuit_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

PurePursuitFileConfig LoadPurePursuitConfig(const std::string& path) {
    PurePursuitFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    readFloat(src, "lookahead_distance", cfg.lookahead_distance);
    readFloat(src, "lookahead_gain",     cfg.lookahead_gain);
    readFloat(src, "max_speed",          cfg.max_speed_mps);
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

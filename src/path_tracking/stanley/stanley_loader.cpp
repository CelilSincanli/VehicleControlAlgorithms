#include "path_tracking/stanley/stanley_loader.hpp"
#include "path_tracking/config_reader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

StanleyFileConfig LoadStanleyConfig(const std::string& path) {
    StanleyFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    readFloat(src, "stanley_gain",  cfg.stanley_gain);
    readFloat(src, "min_speed",     cfg.min_speed);
    readFloat(src, "max_speed",     cfg.max_speed_mps);
    readFloat(src, "max_delta",     cfg.max_delta);
    readInt  (src, "search_window", cfg.search_window);
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

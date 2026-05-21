#include "path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit_loader.hpp"
#include <fstream>
#include <string>

namespace path_tracking {

static bool readFloat(const std::string& src, const std::string& key, float& out) {
    std::string search = "\"" + key + "\"";
    auto pos = src.find(search);
    if (pos == std::string::npos) return false;
    pos = src.find(':', pos + search.size());
    if (pos == std::string::npos) return false;
    ++pos;
    while (pos < src.size() && (src[pos] == ' ' || src[pos] == '\t' || src[pos] == '\n')) ++pos;
    try { out = std::stof(src.substr(pos)); return true; }
    catch (...) { return false; }
}

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
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

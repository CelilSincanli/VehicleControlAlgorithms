#include "path_tracking/stanley/stanley_loader.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

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

StanleyFileConfig LoadStanleyConfig(const std::string& path) {
    StanleyFileConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) return cfg;
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    readFloat(src, "stanley_gain", cfg.stanley_gain);
    readFloat(src, "min_speed",    cfg.min_speed);
    readFloat(src, "max_speed",    cfg.max_speed_mps);
    cfg.loaded = true;
    return cfg;
}

} // namespace path_tracking

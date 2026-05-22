#include "path_tracking/lqr/lqr_loader.hpp"
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

static bool readInt(const std::string& src, const std::string& key, int& out) {
    std::string search = "\"" + key + "\"";
    auto pos = src.find(search);
    if (pos == std::string::npos) return false;
    pos = src.find(':', pos + search.size());
    if (pos == std::string::npos) return false;
    ++pos;
    while (pos < src.size() && (src[pos] == ' ' || src[pos] == '\t' || src[pos] == '\n')) ++pos;
    try { out = std::stoi(src.substr(pos)); return true; }
    catch (...) { return false; }
}

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

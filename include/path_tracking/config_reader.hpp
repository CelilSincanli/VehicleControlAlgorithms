#pragma once
#include <string>

namespace path_tracking {

inline bool readFloat(const std::string& src, const std::string& key, float& out) {
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

inline bool readInt(const std::string& src, const std::string& key, int& out) {
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

} // namespace path_tracking

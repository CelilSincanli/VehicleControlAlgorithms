#pragma once
#include "tuning_limits.hpp"
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace path_tracking {

// Finds `"key": <number>` in a flat JSON config. A quoted number ("8.0") is
// accepted too. Returns false when the key is missing or its value is not a
// number; `out` is only written on success.
inline bool readFloat(const std::string& src, const std::string& key, float& out) {
    std::string search = "\"" + key + "\"";
    auto pos = src.find(search);
    if (pos == std::string::npos) return false;
    pos = src.find(':', pos + search.size());
    if (pos == std::string::npos) return false;
    ++pos;
    while (pos < src.size() && (src[pos] == ' ' || src[pos] == '\t' || src[pos] == '\n' || src[pos] == '\r' || src[pos] == '"')) ++pos;
    try { size_t used = 0; float v = std::stof(src.substr(pos), &used); if (used == 0) return false; out = v; return true; }
    catch (...) { return false; }
}

inline bool readInt(const std::string& src, const std::string& key, int& out) {
    float v = 0.0f;
    if (!readFloat(src, key, v) || !std::isfinite(v)) return false;
    out = static_cast<int>(std::lround(v));
    return true;
}

inline bool hasKey(const std::string& src, const std::string& key) {
    return src.find("\"" + key + "\"") != std::string::npos;
}

// ---------------------------------------------------------------------------
// ConfigReader: reads values and clamps them into the shared tuning ranges
// (tuning_limits.hpp), collecting one warning per value that was not a
// number or was out of range. A missing key keeps the code default silently.
// ---------------------------------------------------------------------------
class ConfigReader {
public:
    explicit ConfigReader(const std::string& src) : src_(src) {}

    void Float(const char* key, float& out, const tuning::FloatRange& r) {
        float v = out;
        if (!readFloat(src_, key, v)) {
            if (hasKey(src_, key)) Warn(key, "is not a number, default kept", out);
            return;
        }
        float c = std::isfinite(v) ? std::max(r.lo, std::min(r.hi, v)) : r.lo;
        if (c != v) Warn(key, "out of range", c);
        out = c;
    }

    void Int(const char* key, int& out, const tuning::IntRange& r) {
        int v = out;
        if (!readInt(src_, key, v)) {
            if (hasKey(src_, key)) Warn(key, "is not a number, default kept", static_cast<float>(out));
            return;
        }
        int c = std::max(r.lo, std::min(r.hi, v));
        if (c != v) Warn(key, "out of range", static_cast<float>(c));
        out = c;
    }

    const std::vector<std::string>& warnings() const { return warnings_; }

private:
    const std::string&       src_;
    std::vector<std::string> warnings_;

    void Warn(const char* key, const char* what, float used) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s %s -> %.4g", key, what, used);
        warnings_.emplace_back(buf);
    }
};

} // namespace path_tracking

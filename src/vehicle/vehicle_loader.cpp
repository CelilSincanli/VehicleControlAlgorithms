#include "vehicle/vehicle_loader.hpp"
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cctype>

namespace {

struct JVal {
    enum Type { Null, Num, Str, Arr, Obj } type = Null;
    double                   num  = 0.0;
    std::string              str;
    std::vector<JVal>        arr;
    std::vector<std::string> keys;

    double      asFloat()  const { return num; }
    std::string asString() const { return str; }

    bool has(const std::string& k) const {
        for (const auto& key : keys) if (key == k) return true;
        return false;
    }
    const JVal& at(const std::string& k) const {
        for (size_t i = 0; i < keys.size(); ++i)
            if (keys[i] == k) return arr[i];
        static JVal empty{};
        return empty;
    }
    const JVal& at(size_t i) const { return arr.at(i); }
    size_t      size()       const { return arr.size(); }
};

static void skip_ws(const std::string& s, size_t& p) {
    while (p < s.size() && std::isspace((unsigned char)s[p])) ++p;
}

static JVal parse_val(const std::string& s, size_t& p);

static std::string parse_str(const std::string& s, size_t& p) {
    ++p;
    std::string r;
    while (p < s.size() && s[p] != '"') {
        if (s[p] == '\\') { ++p; }
        if (p < s.size()) r += s[p++];
    }
    if (p < s.size()) ++p;
    return r;
}

static JVal parse_num(const std::string& s, size_t& p) {
    size_t start = p;
    if (p < s.size() && s[p] == '-') ++p;
    while (p < s.size() && (std::isdigit((unsigned char)s[p]) ||
           s[p] == '.' || s[p] == 'e' || s[p] == 'E' ||
           s[p] == '+' || s[p] == '-')) ++p;
    JVal v; v.type = JVal::Num;
    try { v.num = std::stod(s.substr(start, p - start)); }
    catch (...) {}
    return v;
}

static JVal parse_arr(const std::string& s, size_t& p) {
    ++p;
    JVal v; v.type = JVal::Arr;
    skip_ws(s, p);
    while (p < s.size() && s[p] != ']') {
        v.arr.push_back(parse_val(s, p));
        skip_ws(s, p);
        if (p < s.size() && s[p] == ',') ++p;
        skip_ws(s, p);
    }
    if (p < s.size()) ++p;
    return v;
}

static JVal parse_obj(const std::string& s, size_t& p) {
    ++p;
    JVal v; v.type = JVal::Obj;
    skip_ws(s, p);
    while (p < s.size() && s[p] != '}') {
        skip_ws(s, p);
        if (s[p] != '"') break;
        std::string key = parse_str(s, p);
        skip_ws(s, p);
        if (p < s.size() && s[p] == ':') ++p;
        skip_ws(s, p);
        v.keys.push_back(key);
        v.arr.push_back(parse_val(s, p));
        skip_ws(s, p);
        if (p < s.size() && s[p] == ',') ++p;
        skip_ws(s, p);
    }
    if (p < s.size()) ++p;
    return v;
}

static JVal parse_val(const std::string& s, size_t& p) {
    skip_ws(s, p);
    if (p >= s.size()) return {};
    char c = s[p];
    if (c == '"') { JVal v; v.type = JVal::Str; v.str = parse_str(s, p); return v; }
    if (c == '{') return parse_obj(s, p);
    if (c == '[') return parse_arr(s, p);
    if (c == '-' || std::isdigit((unsigned char)c)) return parse_num(s, p);
    if (s.substr(p, 4) == "true")  { p += 4; JVal v; v.type = JVal::Num; v.num = 1; return v; }
    if (s.substr(p, 5) == "false") { p += 5; JVal v; v.type = JVal::Num; v.num = 0; return v; }
    if (s.substr(p, 4) == "null")  { p += 4; return {}; }
    return {};
}

static JVal parse_json(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};
    std::string content((std::istreambuf_iterator<char>(f)), {});
    size_t pos = 0;
    return parse_val(content, pos);
}

} // anonymous namespace

bool VehicleLoader::Load(const std::string& json_path) {
    vehicle_ = VehicleData{};
    JVal root = parse_json(json_path);
    if (root.type != JVal::Obj) return false;

    vehicle_.name = root.at("name").asString();

    vehicle_.mass = (float)root.at("mass").asFloat();
    vehicle_.a    = (float)root.at("a").asFloat();
    vehicle_.b    = (float)root.at("b").asFloat();
    vehicle_.CA   = (float)root.at("CA").asFloat();

    vehicle_.minimum_turning_radius             = (float)root.at("minimum_turning_radius").asFloat();
    vehicle_.max_steering_wheel_angle           = (float)root.at("max_steering_wheel_angle").asFloat();
    vehicle_.min_steering_wheel_angle           = (float)root.at("min_steering_wheel_angle").asFloat();
    vehicle_.max_steering_wheel_rate            = (float)root.at("max_steering_wheel_rate").asFloat();
    vehicle_.steering_wheel_to_tire_angle_ratio = (float)root.at("steering_wheel_to_tire_angle_ratio").asFloat();

    vehicle_.wheelbase      = (float)root.at("wheelbase").asFloat();
    vehicle_.wheel_radius   = (float)root.at("wheel_radius").asFloat();
    vehicle_.wheel_width    = (float)root.at("wheel_width").asFloat();
    vehicle_.wheel_tread    = (float)root.at("wheel_tread").asFloat();
    vehicle_.front_overhang = (float)root.at("front_overhang").asFloat();
    vehicle_.rear_overhang  = (float)root.at("rear_overhang").asFloat();
    vehicle_.left_overhang  = (float)root.at("left_overhang").asFloat();
    vehicle_.right_overhang = (float)root.at("right_overhang").asFloat();
    vehicle_.vehicle_height = (float)root.at("vehicle_height").asFloat();

    vehicle_.loaded = true;
    return true;
}

std::vector<std::string> VehicleLoader::DiscoverVehicles(const std::string& vehicle_dir) {
    std::vector<std::string> result;
    namespace fs = std::filesystem;
    if (!fs::exists(vehicle_dir)) return result;

    for (const auto& entry : fs::directory_iterator(vehicle_dir)) {
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() == ".json")
            result.push_back(entry.path().string());
    }
    std::sort(result.begin(), result.end());
    return result;
}

#include "map/map_loader.hpp"
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cctype>

// ---------------------------------------------------------------------------
// Minimal recursive-descent JSON parser (handles objects, arrays, strings,
// numbers, true/false/null — enough for our two fixed file formats).
// ---------------------------------------------------------------------------
namespace {

// Two parallel vectors instead of unordered_map to avoid incomplete-type error
// when instantiating map<string, JVal> inside the JVal struct itself.
struct JVal {
    enum Type { Null, Num, Str, Arr, Obj } type = Null;
    double                   num  = 0.0;
    std::string              str;
    std::vector<JVal>        arr;
    std::vector<std::string> keys; // parallel to arr when type == Obj

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
    ++p; // consume opening "
    std::string r;
    while (p < s.size() && s[p] != '"') {
        if (s[p] == '\\') { ++p; }
        if (p < s.size()) r += s[p++];
    }
    if (p < s.size()) ++p; // consume closing "
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
    ++p; // consume [
    JVal v; v.type = JVal::Arr;
    skip_ws(s, p);
    while (p < s.size() && s[p] != ']') {
        v.arr.push_back(parse_val(s, p));
        skip_ws(s, p);
        if (p < s.size() && s[p] == ',') ++p;
        skip_ws(s, p);
    }
    if (p < s.size()) ++p; // consume ]
    return v;
}

static JVal parse_obj(const std::string& s, size_t& p) {
    ++p; // consume {
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
    if (p < s.size()) ++p; // consume }
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

// ---------------------------------------------------------------------------
// MapLoader
// ---------------------------------------------------------------------------

bool MapLoader::Load(const std::string& map_dir) {
    map_  = MapData{};
    path_ = PathData{};
    bool ok_map  = ParseMap (map_dir + "/map.json");
    bool ok_path = ParsePath(map_dir + "/path.json");
    return ok_map && ok_path;
}

bool MapLoader::ParseMap(const std::string& json_path) {
    JVal root = parse_json(json_path);
    if (root.type != JVal::Obj) return false;

    map_.name         = root.at("name").asString();
    map_.world_width  = (float)root.at("world").at("width").asFloat();
    map_.world_height = (float)root.at("world").at("height").asFloat();
    map_.start.x      = (float)root.at("start").at("x").asFloat();
    map_.start.y      = (float)root.at("start").at("y").asFloat();
    map_.goal.x       = (float)root.at("goal").at("x").asFloat();
    map_.goal.y       = (float)root.at("goal").at("y").asFloat();

    if (root.has("vehicle_radius"))
        map_.vehicle_radius = (float)root.at("vehicle_radius").asFloat();

    map_.obstacles.clear();
    for (size_t i = 0; i < root.at("obstacles").size(); ++i) {
        const JVal& o = root.at("obstacles").at(i);
        Obstacle obs;
        obs.id     = o.at("id").asString();
        obs.shape  = o.at("shape").asString();
        obs.x      = (float)o.at("x").asFloat();
        obs.y      = (float)o.at("y").asFloat();
        obs.width  = (float)o.at("width").asFloat();
        obs.height = (float)o.at("height").asFloat();
        map_.obstacles.push_back(obs);
    }

    map_.loaded = true;
    return true;
}

bool MapLoader::ParsePath(const std::string& json_path) {
    JVal root = parse_json(json_path);
    if (root.type != JVal::Obj) return false;

    path_.map_name  = root.at("map_name").asString();
    path_.algorithm = root.at("algorithm").asString();

    path_.waypoints.clear();
    for (size_t i = 0; i < root.at("waypoints").size(); ++i) {
        const JVal& wp = root.at("waypoints").at(i);
        Point2D p;
        p.x = (float)wp.at("x").asFloat();
        p.y = (float)wp.at("y").asFloat();
        path_.waypoints.push_back(p);
    }

    path_.loaded = true;
    return true;
}

std::vector<std::string> MapLoader::DiscoverMaps(const std::string& maps_root) {
    std::vector<std::string> result;
    namespace fs = std::filesystem;
    if (!fs::exists(maps_root)) return result;

    for (const auto& entry : fs::directory_iterator(maps_root)) {
        if (!entry.is_directory()) continue;
        if (fs::exists(entry.path() / "map.json") &&
            fs::exists(entry.path() / "path.json")) {
            result.push_back(entry.path().string());
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

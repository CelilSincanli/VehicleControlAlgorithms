#include "map/map_loader.hpp"
#include "common/json_value.hpp"
#include <cmath>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cctype>

namespace {
using JVal = json::Value;

bool IsNumber(const JVal& v) { return v.type == JVal::Num && std::isfinite(v.num); }
} // anonymous namespace

// ---------------------------------------------------------------------------
// MapLoader
// ---------------------------------------------------------------------------

bool MapLoader::Load(const std::string& map_dir) {
    map_   = MapData{};
    path_  = PathData{};
    error_.clear();
    // Both files must be valid; a half-loaded map is never left behind.
    if (!ParseMap(map_dir + "/map.json") || !ParsePath(map_dir + "/path.json")) {
        map_  = MapData{};
        path_ = PathData{};
        return false;
    }
    return true;
}

bool MapLoader::ParseMap(const std::string& json_path) {
    JVal root;
    std::string err;
    if (!json::ParseFile(json_path, root, err)) { error_ = "map.json: " + err; return false; }
    if (root.type != JVal::Obj) { error_ = "map.json: top level is not an object"; return false; }

    const JVal& world = root.at("world");
    const JVal& start = root.at("start");
    const JVal& goal  = root.at("goal");
    if (!IsNumber(world.at("width")) || !IsNumber(world.at("height")) ||
        !IsNumber(start.at("x")) || !IsNumber(start.at("y")) || !IsNumber(goal.at("x")) || !IsNumber(goal.at("y"))) {
        error_ = "map.json: world width/height, start and goal must be numbers";
        return false;
    }
    map_.name         = root.at("name").asString();
    map_.world_width  = (float)world.at("width").asFloat();
    map_.world_height = (float)world.at("height").asFloat();
    map_.start.x      = (float)start.at("x").asFloat();
    map_.start.y      = (float)start.at("y").asFloat();
    map_.goal.x       = (float)goal.at("x").asFloat();
    map_.goal.y       = (float)goal.at("y").asFloat();
    if (map_.world_width <= 0.0f || map_.world_height <= 0.0f) { error_ = "map.json: world size must be positive"; return false; }
    auto inside = [&](const Point2D& p) { return p.x >= 0 && p.y >= 0 && p.x <= map_.world_width && p.y <= map_.world_height; };
    if (!inside(map_.start) || !inside(map_.goal)) { error_ = "map.json: start and goal must lie inside the world"; return false; }

    if (root.has("vehicle_radius"))
        map_.vehicle_radius = (float)root.at("vehicle_radius").asFloat();

    const JVal& obstacles = root.at("obstacles");
    for (size_t i = 0; i < obstacles.size(); ++i) {
        const JVal& o = obstacles.at(i);
        Obstacle obs;
        obs.id     = o.at("id").asString();
        obs.shape  = o.at("shape").asString();
        if (!IsNumber(o.at("x")) || !IsNumber(o.at("y")) || !IsNumber(o.at("width")) || !IsNumber(o.at("height"))) {
            error_ = "map.json: obstacle " + std::to_string(i) + " needs numeric x, y, width, height";
            return false;
        }
        obs.x      = (float)o.at("x").asFloat();
        obs.y      = (float)o.at("y").asFloat();
        obs.width  = (float)o.at("width").asFloat();
        obs.height = (float)o.at("height").asFloat();
        if (obs.width <= 0.0f || obs.height <= 0.0f) {
            error_ = "map.json: obstacle " + std::to_string(i) + " has a non-positive size";
            return false;
        }
        map_.obstacles.push_back(obs);
    }

    map_.loaded = true;
    return true;
}

bool MapLoader::ParsePath(const std::string& json_path) {
    JVal root;
    std::string err;
    if (!json::ParseFile(json_path, root, err)) { error_ = "path.json: " + err; return false; }
    if (root.type != JVal::Obj) { error_ = "path.json: top level is not an object"; return false; }

    path_.map_name  = root.at("map_name").asString();
    path_.algorithm = root.at("algorithm").asString();

    const JVal& wps = root.at("waypoints");
    for (size_t i = 0; i < wps.size(); ++i) {
        const JVal& wp = wps.at(i);
        if (!IsNumber(wp.at("x")) || !IsNumber(wp.at("y"))) {
            error_ = "path.json: waypoint " + std::to_string(i) + " needs numeric x and y";
            return false;
        }
        path_.waypoints.push_back({(float)wp.at("x").asFloat(), (float)wp.at("y").asFloat()});
    }
    if (path_.waypoints.size() < 2) { error_ = "path.json: at least 2 waypoints are required"; return false; }

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

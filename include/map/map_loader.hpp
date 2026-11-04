#pragma once
#include "map_data.hpp"
#include <string>
#include <vector>

class MapLoader {
public:
    bool Load(const std::string& map_dir);

    const MapData&  GetMap()  const { return map_;  }
    const PathData& GetPath() const { return path_; }

    // Why the last Load() returned false (syntax error or invalid content).
    const std::string& GetError() const { return error_; }

    static std::vector<std::string> DiscoverMaps(const std::string& maps_root);

private:
    MapData  map_;
    PathData path_;
    std::string error_;

    bool ParseMap (const std::string& json_path);
    bool ParsePath(const std::string& json_path);
};

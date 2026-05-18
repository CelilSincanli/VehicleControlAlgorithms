#pragma once
#include "map_data.hpp"
#include <string>
#include <vector>

class MapLoader {
public:
    bool Load(const std::string& map_dir);

    const MapData&  GetMap()  const { return map_;  }
    const PathData& GetPath() const { return path_; }

    static std::vector<std::string> DiscoverMaps(const std::string& maps_root);

private:
    MapData  map_;
    PathData path_;

    bool ParseMap (const std::string& json_path);
    bool ParsePath(const std::string& json_path);
};

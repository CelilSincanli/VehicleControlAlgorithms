#pragma once
#include "map_data.hpp"
#include <string>
#include <vector>

class MapLoader {
public:
    // Load map.json + path.json from the given directory.
    // Returns true only if both files parsed successfully.
    bool Load(const std::string& map_dir);

    const MapData&  GetMap()  const { return map_;  }
    const PathData& GetPath() const { return path_; }

    // Scan maps_root for sub-directories that contain both
    // map.json and path.json.  Returns sorted list of absolute paths.
    static std::vector<std::string> DiscoverMaps(const std::string& maps_root);

private:
    MapData  map_;
    PathData path_;

    bool ParseMap (const std::string& json_path);
    bool ParsePath(const std::string& json_path);
};

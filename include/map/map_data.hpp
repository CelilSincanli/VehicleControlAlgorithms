#pragma once
#include <string>
#include <vector>

struct Point2D {
    float x = 0.0f;
    float y = 0.0f;
};

struct Obstacle {
    std::string id;
    std::string shape;  // "rectangle"
    float x      = 0.0f;
    float y      = 0.0f;
    float width  = 0.0f;
    float height = 0.0f;
};

struct MapData {
    std::string name;
    float       world_width    = 10.0f;
    float       world_height   = 5.0f;
    Point2D     start;
    Point2D     goal;
    float       vehicle_radius = 0.2f;
    std::vector<Obstacle> obstacles;
    bool        loaded = false;
};

struct PathData {
    std::string          map_name;
    std::string          algorithm;
    std::vector<Point2D> waypoints;
    bool                 loaded = false;
};

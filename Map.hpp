// Map.hpp
// A class to store the map and perform raycasting

#include <vector>

class Map { 
    public: 
        Map(const std::vector<std::vector<int>>& map, float gridSize);
        float raycast(std::vector<float> position, std::vector<float> direction);
        std::vector<std::vector<int>>& getMap();

    private:
        const std::vector<std::vector<int>>& map_;
        float gridSize_;

};
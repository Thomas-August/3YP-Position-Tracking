// Ray.hpp
// Define a class for raycasting
//
#include <vector>

class Map { 
    public: 
        Map(const std::vector<std::vector<int>>& map);
        float raycast(std::vector<float> position, std::vector<float> direction);
        std::vector<std::vector<int>>& getMap();

    private:
        const std::vector<std::vector<int>>& map_;

};
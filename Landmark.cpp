//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#include <ios>
#include <sstream>
#include <fstream>

#include "Landmark.h"

std::vector<Landmark> Landmark::ReadLandmarksFromFile()
{
    const std::string filename = "../data/map_data.txt";

    std::vector<Landmark> map;

    std::ifstream in_file_map(filename.c_str(), std::ifstream::in);

    if (!in_file_map)
        return map;

    std::string line_map;
    while(getline(in_file_map, line_map))
    {
        std::istringstream iss_map(line_map);

        float x, y;
        int id;

        iss_map >> x >> y >> id;

        map.emplace_back(id, x, y);
    }
    return map;
}

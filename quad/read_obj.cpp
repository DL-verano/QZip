#include "read_obj.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string.h>
bool ReadOBJFile(const std::string &filename, std::vector<std::array<double, 3>> &pts,
                 std::vector<std::vector<int>> &faces) {
    char line[512];
    std::ifstream input(filename);
    while (input.getline(line, 512)) {
        if (strlen(line) < 2)
            continue;
        if (line[0] == '#')
            continue;
        std::stringstream ss(line);
        std::string type;
        if (!(ss >> type)) {
            std::cout << "format error";
            return false;
        }
        if (type == "v") {
            std::array<double, 3> p;
            ss >> p[0];
            ss >> p[1];
            ss >> p[2];
            pts.push_back(p);
        } else if (type == "f") {
            faces.emplace_back();
            auto &f = faces.back();
            auto read_vid = [&ss](int &vid) -> bool {
                std::string s;
                ss >> s;
                if (s.empty())
                    return false;
                auto slash = s.find_first_of("/");
                if (slash == std::string::npos) {
                    std::stringstream sss(s);
                    return bool(sss >> vid);
                } else {
                    std::stringstream sss(s.substr(0, slash));
                    return bool(sss >> vid);
                }
            };
            int value;
            while (read_vid(value))
                f.push_back(value - 1);
        } else if (type == "vn") {
            continue;
        } else {
            std::cout << "format error: unknown type " << type << std::endl;
            return false;
        }
    }
    return true;
}

std::vector<std::vector<int>> ReadFeatureLineFile(const std::string &filename) {
    std::vector<std::vector<int>> features;

    std::ifstream input(filename);
    std::string line;

    std::getline(input, line);
    std::istringstream iss(line);
    int n;
    iss >> n;
    for (int i = 0; i < n; i++) {
        std::getline(input, line);
        std::istringstream ss(line);
        int m;
        ss >> m;
        features.emplace_back();
        auto &feature = features.back();
        std::getline(input, line);
        std::istringstream sss(line);
        for (int j = 0; j < m; j++) {
            int id;
            sss >> id;
            feature.push_back(id);
        }
    }
    return features;
}

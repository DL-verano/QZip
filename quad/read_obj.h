#pragma once
#include <array>
#include <string>
#include <vector>

bool ReadOBJFile(const std::string &filename, std::vector<std::array<double, 3>> &pts,
                 std::vector<std::vector<int>> &faces);

std::vector<std::vector<int>> ReadFeatureLineFile(const std::string &filename);

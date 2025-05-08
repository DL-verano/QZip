#pragma once
#include "meta_types.h"
#include <memory>
#include <vector>

class KdTree {
    class Impl;
    std::unique_ptr<Impl> pImpl;

  public:
    KdTree();
    ~KdTree();
    void set_points(const std::vector<Point_2> &points);
    int query(Point_2 p) const;
};

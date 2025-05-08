#pragma once
#include "meta_types.h"
#include "nanoflann.hpp"
#include <memory>
#include <vector>

namespace KdTreeDetails {

struct PointCloud {
    struct SimplePoint {
        double x, y, z;
    };
    std::vector<SimplePoint> pts;

    size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    double_t kdtree_distance(const double_t *p1, const size_t idx_p2, size_t /*size*/) const {
        const double d0 = p1[0] - pts[idx_p2].x;
        const double d1 = p1[1] - pts[idx_p2].y;
        const double d2 = p1[2] - pts[idx_p2].z;
        return d0 * d0 + d1 * d1 + d2 * d2;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, int dim) const {
        if (dim == 0)
            return pts[idx].x;
        else if (dim == 1)
            return pts[idx].y;
        else
            return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX> bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
};

template <typename Derived> struct PointCloudAdaptor {
    typedef double coord_t;

    const Derived *obj; //!< A const ref to the data set origin

    /// The constructor that sets the data set source
    PointCloudAdaptor(const Derived &obj_) : obj(&obj_) {}

    /// CRTP helper method
    inline const Derived &derived() const { return *obj; }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return derived().size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    double_t kdtree_distance(const double_t *p1, const size_t idx_p2, size_t /*size*/) const {
        const double d0 = p1[0] - derived()[idx_p2].x();
        const double d1 = p1[1] - derived()[idx_p2].y();
        const double d2 = p1[2] - derived()[idx_p2].z();
        return d0 * d0 + d1 * d1 + d2 * d2;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline coord_t kdtree_get_pt(const size_t idx, int dim) const {
        if (dim == 0)
            return derived()[idx].x();
        else if (dim == 1)
            return derived()[idx].y();
        else
            return derived()[idx].z();
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX> bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
};

} // namespace KdTreeDetails

class KdTree3D {
    typedef KdTreeDetails::PointCloudAdaptor<std::vector<Point_3>> PointCloudAdaptor;
    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor>, PointCloudAdaptor, 3> my_kd_tree_t;
    std::unique_ptr<my_kd_tree_t> kdtree;
    std::unique_ptr<PointCloudAdaptor> points_adaptor;

  public:
    KdTree3D() { kdtree = nullptr; }

    void set_points(const std::vector<Point_3> &points, int leaf = 20) {
        kdtree = nullptr;
        points_adaptor = std::make_unique<PointCloudAdaptor>(points);
        kdtree = std::make_unique<my_kd_tree_t>(3, *points_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(leaf /* max leaf */));
        kdtree->buildIndex();
    }

    int query(Point_3 p) const {
        struct Point {
            double x, y, z;
        } query_pt{p.x(), p.y(), p.z()};
        size_t num_results = 1;
        size_t ret_index;
        double out_dist_sqr;
        kdtree->knnSearch(&query_pt.x, num_results, &ret_index, &out_dist_sqr);
        return int(ret_index);
    }

    void query_knn(Point_3 p, int k, std::vector<size_t> &indices) const {
        struct Point {
            double x, y, z;
        } query_pt{p.x(), p.y(), p.z()};
        indices.resize(k);
        std::vector<double> out_dist_sqr(k);
        kdtree->knnSearch(&query_pt.x, k, indices.data(), out_dist_sqr.data());
    }
};
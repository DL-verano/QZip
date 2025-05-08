#include "kd_tree.h"
#include "nanoflann.hpp"

struct PointCloud {
    struct SimplePoint {
        double x, y;
        int index;
    };
    std::vector<SimplePoint> pts;

    size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    double_t kdtree_distance(const double_t *p1, const size_t idx_p2, size_t /*size*/) const {
        const double d0 = p1[0] - pts[idx_p2].x;
        const double d1 = p1[1] - pts[idx_p2].y;
        return d0 * d0 + d1 * d1;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, int dim) const {
        if (dim == 0)
            return pts[idx].x;
        else
            return pts[idx].y;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX> bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
};

class KdTree::Impl {
    PointCloud cloud;
    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2> my_kd_tree_t;
    std::unique_ptr<my_kd_tree_t> kdtree;

  public:
    Impl() { kdtree = nullptr; }
    void set_points(const std::vector<Point_2> &points) {
        kdtree = nullptr;
        cloud.pts.clear();
        for (int k = 0; k < points.size(); k++)
            cloud.pts.push_back(PointCloud::SimplePoint{points[k].x(), points[k].y(), k});
        kdtree = std::make_unique<my_kd_tree_t>(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        kdtree->buildIndex();
    }

    int query(Point_2 p) {
        PointCloud::SimplePoint query_pt{p.x(), p.y(), -1};
        size_t num_results = 1;
        size_t ret_index;
        double out_dist_sqr;
        kdtree->knnSearch(&query_pt.x, num_results, &ret_index, &out_dist_sqr);
        return cloud.pts[ret_index].index;
    }
};

KdTree::KdTree() { pImpl = std::make_unique<Impl>(); }

KdTree::~KdTree() = default;

void KdTree::set_points(const std::vector<Point_2> &points) { pImpl->set_points(points); }

int KdTree::query(Point_2 p) const { return pImpl->query(p); }

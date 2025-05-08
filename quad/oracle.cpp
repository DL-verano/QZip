#include "oracle.h"
#include <random>

void Oracle::random_inside_point(int number, int random_seed, std::back_insert_iterator<std::vector<Point_3>> points) const {
    CGAL::Bbox_3 bbox = bounding_box();
    std::default_random_engine gene(random_seed);
    std::uniform_real_distribution<double> rand01(0.0, 1.0);
    for (int i = 0; i < number; i++) {
        while (true) {
            double x = rand01(gene) * (bbox.xmax() - bbox.xmin()) + bbox.xmin();
            double y = rand01(gene) * (bbox.ymax() - bbox.ymin()) + bbox.ymin();
            double z = rand01(gene) * (bbox.zmax() - bbox.zmin()) + bbox.zmin();
            Point_3 p(x, y, z);
            if (inside(p)) {
                *points++ = p;
                break;
            }
        }
    }
}

#pragma once
#include "meta_types.h"
class Oracle {
  public:
    virtual ~Oracle() = default;
    virtual bool inside(const Point_3 &p) const = 0;
    virtual Vector_3 vector_to_nearest_point_on_boundary(const Point_3 &p) const = 0;
    virtual Vector_3 oriented_segment_intersection(const Point_3& p, const Vector_3& half_segment) const = 0;
    virtual Vector_3 vector_to_nearest_point_and_normal_on_boundary(const Point_3 &p, Vector_3 &normal) const { throw "not implemented"; }
    virtual Vector_3 vector_to_nearest_point_on_sharp_edges(const Point_3 &p) const { throw "not implemented"; }
    virtual Vector_3 vector_to_nearest_point_on_sharp_vertices(const Point_3 &p) const { throw "not implemented"; }
    virtual Vector_3 vector_to_nearest_point_on_border_edges(const Point_3 &p) const { throw "not implemented"; }
    virtual CGAL::Bbox_3 bounding_box() const = 0;
    virtual void random_inside_point(int number, int random_seed, std::back_insert_iterator<std::vector<Point_3>> points) const;

    virtual bool has_sharp_edges() const { return false; }
    virtual bool has_sharp_vertices() const { return false; }
    virtual const std::vector<Point_3> &sharp_edge_samples() const { throw "not implemented"; }
    virtual const std::vector<Kernel::Segment_3> &sharp_edges() const { throw "not implemented"; }
    virtual const std::vector<Point_3> &sharp_vertices() const { throw "not implemented"; }
    virtual const std::vector<std::pair<Kernel::Point_3, Kernel::Vector_3>>& surface_sample_and_normals() const { throw "not implemented"; }

    virtual bool get_ray_intersection(const Kernel::Ray_3 &ray, Kernel::Point_3 &p) const { throw "not implemented"; }

    virtual void render() const {}
};

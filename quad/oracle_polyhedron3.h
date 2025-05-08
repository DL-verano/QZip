#pragma once
#include "oracle.h"
#include "tools/kd_tree_3d.h"
#include <CGAL/HalfedgeDS_halfedge_base.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/Polyhedron_items_3.h>
#include <memory>

#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>

class OraclePolyhedron3 : public Oracle {
  public:
    template <class Refs> struct My_halfedge : public CGAL::HalfedgeDS_halfedge_base<Refs> {};
    template <class Refs> struct My_vertex : public CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point_3> {
        int idx;
        My_vertex() : CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point_3>() {}
        My_vertex(const Point_3 &p) : CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point_3>(p) {}
    };
    // An items type using my face.
    struct My_items : public CGAL::Polyhedron_items_3 {
        template <class Refs, class Traits> struct Halfedge_wrapper { typedef My_halfedge<Refs> Halfedge; };
        template <class Refs, class Traits> struct Vertex_wrapper {
            typedef typename Traits::Point_3 Point;
            typedef My_vertex<Refs> Vertex;
        };
    };
    typedef CGAL::Polyhedron_3<Kernel, My_items> Polyhedron;

  private:
    typedef CGAL::Polyhedral_mesh_domain_3<Polyhedron, Kernel> Mesh_domain;
    Polyhedron poly_;
    Mesh_domain domain_;
    Mesh_domain::Is_in_domain is_in_domain_;

    // AABB Tree
    typedef CGAL::Simple_cartesian<double> AABB_Kernel;

    // boundary facet
    std::vector<Kernel::Triangle_3> boundary_triangles_;

    typedef std::vector<AABB_Kernel::Triangle_3>::iterator Triangle_Iterator;
    typedef CGAL::AABB_triangle_primitive<AABB_Kernel, Triangle_Iterator> Triangle_Primitive;
    typedef CGAL::AABB_traits<AABB_Kernel, Triangle_Primitive> AABB_triangle_traits;
    typedef CGAL::AABB_tree<AABB_triangle_traits> Triangle_AABBTree;
    std::unique_ptr<Triangle_AABBTree> boundary_triangle_aabb_;
    std::vector<AABB_Kernel::Triangle_3> boundary_triangles_for_aabb_;

    // sharp edges
    std::vector<Point_3> sharp_edge_samples_;
    std::vector<Kernel::Segment_3> sharp_edges_;
    bool has_sharp_edges_;

    typedef std::vector<AABB_Kernel::Segment_3>::iterator Segment_Iterator;
    typedef CGAL::AABB_triangle_primitive<AABB_Kernel, Segment_Iterator> Segment_Primitive;
    typedef CGAL::AABB_traits<AABB_Kernel, Segment_Primitive> AABB_segment_traits;
    typedef CGAL::AABB_tree<AABB_segment_traits> Segment_AABBTree;
    std::unique_ptr<Segment_AABBTree> sharp_edge_aabb_tree_;
    std::vector<AABB_Kernel::Segment_3> sharp_edges_for_aabb_;

    // sharp vertices
    std::vector<Kernel::Point_3> sharp_vertices_;
    bool has_sharp_vertices_;
    KdTree3D sharp_vertices_kd_tree_;

    std::vector<std::pair<Kernel::Point_3, Kernel::Vector_3>> surface_samples_;

    std::unique_ptr<Segment_AABBTree> border_edge_aabb_tree_;
    std::vector<AABB_Kernel::Segment_3> border_edges_for_aabb_;

  public:
    OraclePolyhedron3(Polyhedron &poly, const std::vector<std::vector<int>> *features, double sharp_edge_angle,
                      double sharp_edge_turn_angle);

    Vector_3 vector_to_nearest_point_on_boundary(const Point_3 &p) const override {
        CGAL::Cartesian_converter<Kernel, AABB_Kernel> converter;
        CGAL::Cartesian_converter<AABB_Kernel, Kernel> inverser;
        return inverser(boundary_triangle_aabb_->closest_point(converter(p))) - p;
    }

    Vector_3 oriented_segment_intersection(const Point_3 &p, const Vector_3 &half_segment) const override;

    Vector_3 vector_to_nearest_point_and_normal_on_boundary(const Point_3 &p, Vector_3 &normal) const override {
        CGAL::Cartesian_converter<Kernel, AABB_Kernel> converter;
        CGAL::Cartesian_converter<AABB_Kernel, Kernel> inverser;
        auto pri_and_id = boundary_triangle_aabb_->closest_point_and_primitive(converter(p));
        auto pt = pri_and_id.first;
        auto id = pri_and_id.second;
        AABB_Kernel::Triangle_3 t = *id;
        Vector_3 n = inverser(CGAL::normal(t.vertex(0), t.vertex(1), t.vertex(2)));
        normal = n / std::sqrt(n.squared_length());
        return inverser(pt) - p;
    }

    Vector_3 vector_to_nearest_point_on_sharp_edges(const Point_3 &p) const override {
        CGAL::Cartesian_converter<Kernel, AABB_Kernel> converter;
        CGAL::Cartesian_converter<AABB_Kernel, Kernel> inverser;
        return inverser(sharp_edge_aabb_tree_->closest_point(converter(p))) - p;
    }

    Vector_3 vector_to_nearest_point_on_sharp_vertices(const Point_3 &p) const override {
        int i = sharp_vertices_kd_tree_.query(p);
        return sharp_vertices_[i] - p;
    }

    Vector_3 vector_to_nearest_point_on_border_edges(const Point_3 &p) const override {
        CGAL::Cartesian_converter<Kernel, AABB_Kernel> converter;
        CGAL::Cartesian_converter<AABB_Kernel, Kernel> inverser;
        return inverser(border_edge_aabb_tree_->closest_point(converter(p))) - p;
    }

    CGAL::Bbox_3 bounding_box() const override { return domain_.bbox(); }

    bool inside(const Point_3 &p) const override { return bool(is_in_domain_(p)); }

    bool has_sharp_edges() const override { return has_sharp_edges_; }
    const std::vector<Point_3> &sharp_edge_samples() const override { return sharp_edge_samples_; }
    const std::vector<Kernel::Segment_3> &sharp_edges() const override { return sharp_edges_; }

    bool has_sharp_vertices() const override { return has_sharp_vertices_; }
    const std::vector<Kernel::Point_3> &sharp_vertices() const override { return sharp_vertices_; }

    const std::vector<std::pair<Kernel::Point_3, Kernel::Vector_3>> &surface_sample_and_normals() const override {
        return surface_samples_;
    }

    bool get_ray_intersection(const Kernel::Ray_3 &ray, Kernel::Point_3 &p) const override;

    void render() const override;
};

#include "oracle_polyhedron3.h"

#include "tools/gl_tools.h"
#include "tools/materials.h"
#include "tools/scope_timer.h"
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/Triangle_accessor_3.h>
#include <iostream>
#include <random>

OraclePolyhedron3::OraclePolyhedron3(Polyhedron &poly, const std::vector<std::vector<int>> *features,
                                     double sharp_edge_angle, double sharp_edge_turn_angle)
    : poly_(poly), domain_(poly_), is_in_domain_(domain_.is_in_domain_object()), has_sharp_edges_(false),
      has_sharp_vertices_(false) {
    typedef CGAL::Triangle_accessor_3<Polyhedron, Kernel> TA;

    poly_.normalize_border();
    CGAL::Cartesian_converter<Kernel, AABB_Kernel> converter;

    {
        // collect boundary triangles
        auto end = TA().triangles_end(poly_);
        double total_area = 0.;
        for (auto it = TA().triangles_begin(poly_); it != end; ++it) {
            auto t = TA().triangle(it);
            double area = std::sqrt(t.squared_area());
            total_area += area;
            boundary_triangles_.push_back(t);
            boundary_triangles_for_aabb_.push_back(converter(t));
        }
    }
    boundary_triangle_aabb_ = std::make_unique<Triangle_AABBTree>();
    boundary_triangle_aabb_->insert(boundary_triangles_for_aabb_.begin(), boundary_triangles_for_aabb_.end());
    boundary_triangle_aabb_->accelerate_distance_queries();

    int idx = 0;
    for (auto it = poly_.vertices_begin(); it != poly_.vertices_end(); ++it, ++idx) {
        it->idx = idx;
    }
    std::set<std::pair<int, int>> sharp_edges;
    if (features) {
        for (const auto &lines : *features) {
            for (int i = 0; i < lines.size() - 1; i++) {
                int a = lines[i];
                int b = lines[i + 1];
                sharp_edges.insert(std::make_pair(std::min(a, b), std::max(a, b)));
            }
        }
    }

    std::unordered_map<Polyhedron::Vertex *, std::vector<Vector_3>> v_to_seg;

    if (sharp_edge_angle != 0. || features) {
        // collect boundary sharp edges
        int count = 0;
        for (auto it = poly_.edges_begin(); it != poly_.edges_end(); ++it) {
            auto other = it->opposite();

            bool is_sharp_edge = false;
            auto v0 = it->vertex();
            auto v1 = other->vertex();
            Point_3 p0 = v0->point();
            Point_3 q0 = v1->point();

            if (features) {
                int a = std::min(v0->idx, v1->idx);
                int b = std::max(v0->idx, v1->idx);
                if (sharp_edges.find(std::make_pair(a, b)) != sharp_edges.end())
                    is_sharp_edge = true;
            } else {
                if (it->is_border_edge()) {
                    is_sharp_edge = true;
                } else {
                    Point_3 p1 = it->next()->vertex()->point();
                    Point_3 p2 = it->next()->next()->vertex()->point();

                    Point_3 q1 = other->next()->vertex()->point();
                    Point_3 q2 = other->next()->next()->vertex()->point();

                    Vector_3 v1 = CGAL::cross_product(p1 - p0, p2 - p1);
                    Vector_3 v2 = CGAL::cross_product(q1 - q0, q2 - q1);

                    v1 = v1 / std::sqrt(v1.squared_length());
                    v2 = v2 / std::sqrt(v2.squared_length());
                    double cosine = CGAL::scalar_product(v1, v2);
                    double degree = 180. - std::acos(cosine) / 3.14159265358979323 * 180.;
                    is_sharp_edge = degree < sharp_edge_angle;
                }
            }
            if (is_sharp_edge) {
                count++;
                Kernel::Segment_3 s(it->vertex()->point(), other->vertex()->point());
                sharp_edges_.push_back(s);
                sharp_edges_for_aabb_.push_back(converter(s));
                v_to_seg[&*it->vertex()].push_back(q0 - p0);
                v_to_seg[&*other->vertex()].push_back(p0 - q0);
            }
        }
        if (count > 0)
            has_sharp_edges_ = true;
        std::cout << count << " sharp edges detected" << std::endl;

        if (has_sharp_edges_) {
            count = 0;
            const double dx = 0.01;
            for (auto &e : sharp_edges_) {
                double l = std::sqrt(e.squared_length());
                int n = std::max(int(std::ceil(l / dx)), 1);
                for (int i = 0; i < n; i++) {
                    sharp_edge_samples_.push_back(e.source() + (e.target() - e.source()) / n * (i + 0.5));
                }
                count += n;
            }
        }
        std::cout << count << " sharp samples created" << std::endl;

        sharp_edge_aabb_tree_ = std::make_unique<Segment_AABBTree>();
        sharp_edge_aabb_tree_->insert(sharp_edges_for_aabb_.begin(), sharp_edges_for_aabb_.end());
        sharp_edge_aabb_tree_->accelerate_distance_queries();

        for (auto it = poly_.vertices_begin(); it != poly_.vertices_end(); ++it) {
            const int num_edges = v_to_seg.find(&*it) == v_to_seg.end() ? 0 : v_to_seg.at(&*it).size();
            bool is_sharp_vertex = num_edges > 0 && num_edges != 2;
            if (num_edges == 2) {
                Vector_3 v1 = v_to_seg.at(&*it).front();
                Vector_3 v2 = v_to_seg.at(&*it).back();
                v1 /= std::sqrt(v1.squared_length());
                v2 /= std::sqrt(v2.squared_length());
                if (std::acos(v1 * v2) / 3.1415 * 180 < sharp_edge_turn_angle) {
                    is_sharp_vertex = true;
                }
            }
            if (is_sharp_vertex) {
                sharp_vertices_.push_back(it->point());
                has_sharp_vertices_ = true;
            }
        }
        if (has_sharp_vertices_) {
            sharp_vertices_kd_tree_.set_points(sharp_vertices_);
        }
    }

    for (auto it = poly_.border_halfedges_begin(); it != poly_.halfedges_end(); ++it) {
        auto other = it->opposite();
        Kernel::Segment_3 s(it->vertex()->point(), other->vertex()->point());
        border_edges_for_aabb_.push_back(converter(s));
        border_edge_aabb_tree_ = std::make_unique<Segment_AABBTree>();
        border_edge_aabb_tree_->insert(border_edges_for_aabb_.begin(), border_edges_for_aabb_.end());
        border_edge_aabb_tree_->accelerate_distance_queries();
    }

    double total_area = 0.;
    std::default_random_engine gen(123);
    {
        auto end = TA().triangles_end(poly_);
        for (auto it = TA().triangles_begin(poly_); it != end; ++it) {
            auto t = TA().triangle(it);
            double area = std::sqrt(t.squared_area());
            total_area += area;
        }
    }
    const int face_count = poly_.size_of_facets() * 30;
    const double density = double(face_count) / total_area;

    double residual = 0.0;
    std::uniform_real_distribution<double> uni(0., 1.);

    {
        auto random_uniform_point_3d = [&uni, &gen](const Kernel::Triangle_3 &t) {
            typedef Kernel::Point_3 Point;
            typedef Kernel::Vector_3 Vector;

            Point a = t.vertex(0);
            Point b = t.vertex(1);
            Point c = t.vertex(2);

            // edge vectors
            Vector ab = b - a;
            Vector ac = c - a;

            // pick random value in triangle (0,0), (0,1), (1,0)
            double u = uni(gen);
            double v = uni(gen);
            if (u + v > 1.0) // flip over diag if needed
            {
                u = 1.0 - u;
                v = 1.0 - v;
            }
            Point p = a + u * ab + v * ac;
            return p;
        };

        auto end = TA().triangles_end(poly_);
        for (auto it = TA().triangles_begin(poly_); it != end; ++it) {
            Kernel::Triangle_3 t = TA().triangle(it);
            double area = std::sqrt(t.squared_area());

            double nb_samples_d = area * density + residual;
            auto nb_samples = static_cast<unsigned int>(std::round(nb_samples_d));

            for (int i = 0; i < nb_samples; i++) {
                const Vector_3 normal = CGAL::normal(t.vertex(0), t.vertex(1), t.vertex(2));
                surface_samples_.emplace_back(random_uniform_point_3d(t), normal / std::sqrt(normal.squared_length()));
            }

            residual = nb_samples_d - double(nb_samples);
        }
    }
}

Vector_3 OraclePolyhedron3::oriented_segment_intersection(const Point_3 &p, const Vector_3 &half_segment) const {
    CGAL::Cartesian_converter<Kernel, AABB_Kernel> converter;
    CGAL::Cartesian_converter<AABB_Kernel, Kernel> inverser;
    typedef Triangle_AABBTree::Intersection_and_primitive_id<AABB_Kernel::Segment_3>::Type Seg_intersection;
    std::vector<Seg_intersection> inters;
    typedef AABB_Kernel::Point_3 Pt3;
    const Kernel::Segment_3 s(p - half_segment, p + half_segment);
    boundary_triangle_aabb_->all_intersections(converter(s), std::back_inserter(inters));
    Vector_3 result = CGAL::NULL_VECTOR;
    double min_dist = std::numeric_limits<double>::max();
    const Vector_3 dir = half_segment / std::sqrt(half_segment.squared_length());
    for (const Seg_intersection &inter : inters) {
        const Pt3 *pt = boost::get<Pt3>(&inter.first);
        auto id = inter.second;
        AABB_Kernel::Triangle_3 t = *id;
        Vector_3 normal = inverser(CGAL::normal(t.vertex(0), t.vertex(1), t.vertex(2)));
        normal /= std::sqrt(normal.squared_length());
        if (normal * dir > 0.707 /*45 degree*/) {
            const auto pt_inv = inverser(*pt);
            const double dist = CGAL::squared_distance(pt_inv, p);
            if (dist < min_dist) {
                min_dist = dist;
                result = pt_inv - p;
            }
        }
    }
    return result;
}

bool OraclePolyhedron3::get_ray_intersection(const Kernel::Ray_3 &ray, Kernel::Point_3 &p) const {
    CGAL::Cartesian_converter<Kernel, AABB_Kernel> converter;
    CGAL::Cartesian_converter<AABB_Kernel, Kernel> inverser;
    typedef boost::optional<Triangle_AABBTree::Intersection_and_primitive_id<AABB_Kernel::Ray_3>::Type>
        Ray_intersection;
    typedef AABB_Kernel::Point_3 Pt3;
    Ray_intersection inter = boundary_triangle_aabb_->first_intersection(converter(ray));
    if (inter) {
        if (boost::get<Pt3>(&(inter->first))) {
            const Pt3 *pt3 = boost::get<Pt3>(&(inter->first));
            p = inverser(*pt3);
            return true;
        }
    }
    return false;
}

void OraclePolyhedron3::render() const {
    gl_set_material(Material::Ruby);
    ::glDisable(GL_COLOR_MATERIAL);
    ::glEnable(GL_LIGHTING);

    if (!sharp_edges_.empty()) {
        ::glEnable(GL_POLYGON_OFFSET_FILL); // do not overlap lines
        ::glPolygonOffset(1., 1.);
    }

    std::vector<std::pair<int, double>> tri_depth;
    {
        ScopeTimer timer("Collect triangles with z order");

        GLdouble model_view[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
        GLdouble projection[16];
        glGetDoublev(GL_PROJECTION_MATRIX, projection);
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        for (int i = 0; i < boundary_triangles_.size(); i++) {
            Point_3 center = CGAL::centroid(boundary_triangles_[i]);
            GLdouble x, y, z;
            // get window coords based on 3D coordinates
            gluProject(center.x(), center.y(), center.z(), model_view, projection, viewport, &x, &y, &z);
            tri_depth.emplace_back(i, -z);
        }
    }
    {
        ScopeTimer timer("Sort triangles with z order");
        std::sort(tri_depth.begin(), tri_depth.end(),
                  [](std::pair<int, double> &a, std::pair<int, double> &b) -> bool { return a.second < b.second; });
    }

    ::glBegin(GL_TRIANGLES);
    for (const auto &pair : tri_depth) {
        const auto &t = boundary_triangles_[pair.first];
        gl_shaded_triangle(t[0], t[1], t[2]);
        gl_shaded_triangle(t[1], t[0], t[2]);
    }
    ::glEnd();

    ::glDisable(GL_LIGHTING);

    {
        ::glColor3f(1.0, 0.0, 0.0);
        ::glLineWidth(2.0);
        ::glBegin(GL_LINES);
        for (auto &e : sharp_edges_) {
            ::glVertex3d(e.source().x(), e.source().y(), e.source().z());
            ::glVertex3d(e.target().x(), e.target().y(), e.target().z());
        }
        ::glEnd();
    }
    ::glDisable(GL_POLYGON_OFFSET_FILL); // do not overlap lines

    ::glColor3f(1.0, 0.0, 0.0);
    ::glPointSize(8.0);
    ::glBegin(GL_POINTS);
    for (auto &p : sharp_vertices_) {
        gl_vertex(p);
    }
    ::glEnd();

    ::glColor3f(0.0, 0.0, 0.0);
    ::glPointSize(1.0);
    ::glBegin(GL_POINTS);
    for (auto &p : surface_samples_) {
        gl_vertex(p.first);
    }
    ::glEnd();
}

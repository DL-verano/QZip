#pragma once

#include "quad/oracle.h"
#include "quad/quad.h"
#include <CGAL/Polyhedron_3.h>
#include <QObject>
class Scene3DSurface : public QObject {
    Q_OBJECT

  signals:
    void need_repaint();
    void need_save_snapshot(const QString &filename);
    void scene_sphere_update(double x, double y, double z, double r);

  private:
    typedef Kernel::FT FT;
    typedef CGAL::Bbox_3 Bbox;
    typedef Kernel::Point_3 Point_3;
    typedef Kernel::Plane_3 Plane;
    typedef Kernel::Sphere_3 Sphere;
    typedef Kernel::Vector_3 Vector_3;
    // typedef Kernel::Segment_3 Segment_3;
    typedef Kernel::Triangle_3 Triangle_3;
    typedef Kernel::Tetrahedron_3 Tetrahedron;
    typedef CGAL::Polyhedron_3<Kernel> SimplePolyhedron;

    Ramp ramp;
    Quad2d3d quad;

    bool m_view_oracle = true;
    bool m_view_oracle_sharp_edge_sample = false;
    bool m_view_quads = true;
    bool m_auto_recenter_when_load = true;

    std::shared_ptr<Oracle> oracle;

  public: // life cycle
    Scene3DSurface();
    virtual ~Scene3DSurface();

    bool open_surface_mesh(const QString &filename, const QString &feature_line_filename,
                           double sharp_edge_threshold_degree, double sharp_edge_turn_threshold_degree);
    bool open_triangle_mesh(const QString &filename);
    bool open_quad_off_mesh(const QString &filename);
    void open_quad_mesh(const QString &filename);
    void save_quad_mesh(const QString &filename);
    void save_quad_mesh_to_obj(const QString &filename);
    void save_quad_mesh_sharp_edge_candidate(const QString &filename);
    void open_quad_mesh_sharp_edge_candidate(const QString &filename);

    void set_view_oracle(bool value) { m_view_oracle = value; }
    void set_view_oracle_sharp_edge_sample(bool value) { m_view_oracle_sharp_edge_sample = value; }
    void set_view_quad(bool value) { m_view_quads = value; }
    void set_auto_recenter_when_load(bool value) { m_auto_recenter_when_load = value; }

    void auto_tool(const std::string &oracle_folder, const std::string &result_folder, const std::string &mesh_name);

    Quad2d3d &get_quad() { return quad; }

    // rendering
    void render() const;
    bool get_intersection(Kernel::Ray_3 ray, Kernel::Point_3 &center);
};
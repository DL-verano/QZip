#include "scene.h"

#include <QTextStream>
#include <fstream>
#include <iostream>

#include <CGAL/make_mesh_3.h>

#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

#include "console_color.h"
#include "quad/oracle_polyhedron3.h"
#include "quad/read_obj.h"
#include "tools/gl_tools.h"

#include <QDir>
#include <QFile>

Scene3DSurface::Scene3DSurface() {
    connect(&quad, SIGNAL(need_repaint()), this, SIGNAL(need_repaint()));
    quad.set_oracle(oracle);
}

Scene3DSurface::~Scene3DSurface() {}

// A modifier creating a triangle with the incremental builder.
template <class HDS> class TriangleMeshBuilder : public CGAL::Modifier_base<HDS> {
  public:
    std::vector<std::array<double, 3>> pts;
    std::vector<std::vector<int>> faces;
    TriangleMeshBuilder() {}
    void operator()(HDS &hds) {
        // Postcondition: hds is a valid polyhedral surface.
        CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
        B.begin_surface(pts.size(), faces.size());
        typedef typename HDS::Vertex Vertex;
        typedef typename Vertex::Point Point;
        for (const auto &p : pts)
            B.add_vertex(Point(p[0], p[1], p[2]));

        for (const auto &f : faces) {
            B.begin_facet();
            for (int vi : f)
                B.add_vertex_to_facet(vi);
            B.end_facet();
        }
        B.end_surface();
    }
};

bool Scene3DSurface::open_surface_mesh(const QString &filename, const QString &feature_line_filename,
                                       double sharp_edge_threshold_degree, double sharp_edge_turn_threshold_degree) {
    typedef OraclePolyhedron3::Polyhedron Polyhedron;
    typedef Polyhedron::HalfedgeDS HalfedgeDS;

    Polyhedron polyhedron;
    if (filename.endsWith(".off", Qt::CaseInsensitive)) {
        std::ifstream input;
        input.open(filename.toUtf8().constData());
        input >> polyhedron;
    } else if (filename.endsWith(".obj")) {
        TriangleMeshBuilder<HalfedgeDS> builder;
        if (!ReadOBJFile(filename.toStdString(), builder.pts, builder.faces))
            return false;
        polyhedron.delegate(builder);
    } else {
        std::cout << "unknown format" << std::endl;
        return false;
    }

    std::vector<std::vector<int>> features;
    if (!feature_line_filename.isEmpty()) {
        features = ReadFeatureLineFile(feature_line_filename.toStdString());
    }

    CGAL::Bbox_3 b;
    for (auto it = polyhedron.vertices_begin(); it != polyhedron.vertices_end(); ++it) {
        const auto p = it->point();
        b += CGAL::Bbox_3(p.x(), p.y(), p.z(), p.x(), p.y(), p.z());
    }
    const double x = (b.xmin() + b.xmax()) / 2.;
    const double y = (b.ymin() + b.ymax()) / 2.;
    const double z = (b.zmin() + b.zmax()) / 2.;
    const double dx = x - b.xmin();
    const double dy = y - b.ymin();
    const double dz = z - b.zmin();
    const double r = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (m_auto_recenter_when_load)
        emit scene_sphere_update(x, y, z, r);

    oracle = std::make_shared<OraclePolyhedron3>(polyhedron, feature_line_filename.isEmpty() ? nullptr : &features,
                                                 sharp_edge_threshold_degree, sharp_edge_turn_threshold_degree);
    quad.set_oracle(oracle);

    return true;
}

bool Scene3DSurface::open_triangle_mesh(const QString &filename) {
    if (!filename.contains(".off", Qt::CaseInsensitive))
        return false;
    std::ifstream input;
    input.open(filename.toUtf8().constData());

    SimplePolyhedron poly;
    input >> poly;
    typedef SimplePolyhedron::Vertex_handle Vh;
    typedef SimplePolyhedron::Face_handle Fh;

    std::map<Vh, Quad2d3d::Vertex_handle> vh_to_handle;
    std::map<Fh, Quad2d3d::Vertex_handle> fh_to_handle;
    std::map<std::pair<Vh, Vh>, Quad2d3d::Vertex_handle> edge_to_handle;
    std::list<Quad2d3d::Vertex> vertices;
    std::list<Quad2d3d::Quad> quads;
    int vid_count = 0;
    int qid_count = 0;
    for (auto vi = poly.vertices_begin(); vi != poly.vertices_end(); ++vi) {
        Quad2d3d::Vertex vq;
        vq.id = vid_count++;
        vq.point = Point_3(vi->point().x(), vi->point().y(), vi->point().z());
        vertices.push_back(vq);
        vh_to_handle[vi] = --vertices.end();
    }
    auto make_edge = [](const Vh &v1, const Vh &v2) {
        if (v1 < v2)
            return std::make_pair(v1, v2);
        return std::make_pair(v2, v1);
    };
    auto get_vhs = [&](const SimplePolyhedron::Facet &f) {
        SimplePolyhedron::Halfedge_around_facet_const_circulator h, hend;
        h = hend = f.facet_begin();
        std::vector<SimplePolyhedron::Vertex_handle> vhs;
        CGAL_For_all(h, hend) {
            SimplePolyhedron::Halfedge hedge = *h;
            SimplePolyhedron::Vertex_handle v = hedge.vertex();
            vhs.push_back(v);
        }
        if (vhs.size() != 3) {
            std::cout << "non triangle found" << std::endl;
            throw;
        }
        return vhs;
    };
    for (auto fi = poly.facets_begin(); fi != poly.facets_end(); ++fi) {
        const auto vhs = get_vhs(*fi);

        Quad2d3d::Vertex vq;
        const Point_3 c = CGAL::centroid(vhs[0]->point(), vhs[1]->point(), vhs[2]->point());
        vq.id = vid_count++;
        vq.point = Point_3(c.x(), c.y(), c.z());
        vertices.push_back(vq);
        fh_to_handle[fi] = --vertices.end();
        for (int k = 0; k < 3; k++) {
            const int k1 = (k + 1) % 3;
            const int k2 = (k + 2) % 3;
            const auto edge = make_edge(vhs[k1], vhs[k2]);
            if (edge_to_handle.count(edge) == 0) {
                Quad2d3d::Vertex v;
                const Point_3 c = CGAL::midpoint(vhs[k1]->point(), vhs[k2]->point());
                v.id = vid_count++;
                v.point = Point_3(c.x(), c.y(), c.z());
                vertices.push_back(v);
                edge_to_handle[edge] = --vertices.end();
            }
        }
    }
    for (auto fi = poly.facets_begin(); fi != poly.facets_end(); ++fi) {
        const auto vhs = get_vhs(*fi);
        for (int k = 0; k < 3; k++) {
            const int k1 = (k + 1) % 3;
            const int k2 = (k + 2) % 3;
            const auto vid = vhs[k];
            const auto vid1 = vhs[k1];
            const auto vid2 = vhs[k2];

            Quad2d3d::Quad quad;
            quad.id = qid_count++;
            quad.corners[0] = fh_to_handle.at(fi);
            quad.corners[1] = edge_to_handle.at(make_edge(vid, vid2));
            quad.corners[2] = vh_to_handle.at(vid);
            quad.corners[3] = edge_to_handle.at(make_edge(vid, vid1));
            quads.push_back(quad);
        }
    }

    quad.construct_from_vertices_quads(std::move(vertices), std::move(quads));
    double x, y, z, r;
    quad.get_bounding_sphere(x, y, z, r);
    if (m_auto_recenter_when_load)
        emit scene_sphere_update(x, y, z, r);

    return true;
}

bool Scene3DSurface::open_quad_off_mesh(const QString &filename) {
    if (!filename.contains(".off", Qt::CaseInsensitive))
        return false;
    std::ifstream input;
    input.open(filename.toUtf8().constData());

    SimplePolyhedron poly;
    input >> poly;

    std::list<Quad2d3d::Vertex> vertices;
    std::list<Quad2d3d::Quad> quads;

    std::vector<Quad2d3d::Vertex_handle> v_handles;
    typedef SimplePolyhedron::Vertex_handle Vh;
    std::map<Vh, int> v_map;
    int vid_count = 0;
    int qid_count = 0;
    for (auto vi = poly.vertices_begin(); vi != poly.vertices_end(); ++vi) {
        v_map[vi] = vertices.size();
        vertices.emplace_back();
        auto &v = vertices.back();
        v.id = vid_count++;
        v.point = vi->point();
        auto it = vertices.end();
        --it;
        v_handles.push_back(it);
    }
    std::vector<SimplePolyhedron::Vertex_handle> vhs;
    for (auto fi = poly.facets_begin(); fi != poly.facets_end(); ++fi) {
        SimplePolyhedron::Facet f = *fi; // STL pair
        SimplePolyhedron::Halfedge_around_facet_const_circulator h, hend;
        h = hend = f.facet_begin();
        vhs.clear();
        CGAL_For_all(h, hend) {
            SimplePolyhedron::Halfedge hedge = *h;
            SimplePolyhedron::Vertex_handle v = hedge.vertex();
            vhs.push_back(v);
        }
        if (vhs.size() != 4) {
            std::cout << "non quad found" << std::endl;
            throw;
        }
        quads.emplace_back();
        auto &q = quads.back();
        q.id = qid_count++;
        for (int i = 0; i < 4; i++) {
            const auto &v = vhs[i];
            const int idx = v_map.at(v);
            q.corners.at(i) = v_handles.at(idx);
        }
    }

    quad.construct_from_vertices_quads(std::move(vertices), std::move(quads));

    double x, y, z, r;
    quad.get_bounding_sphere(x, y, z, r);
    if (m_auto_recenter_when_load)
        emit scene_sphere_update(x, y, z, r);
    return true;
}

void Scene3DSurface::open_quad_mesh(const QString &filename) {
    quad.load_quad(filename);
    double x, y, z, r;
    quad.get_bounding_sphere(x, y, z, r);
    if (m_auto_recenter_when_load)
        emit scene_sphere_update(x, y, z, r);
}

void Scene3DSurface::save_quad_mesh(const QString &filename) { quad.save_quad(filename); }
void Scene3DSurface::save_quad_mesh_to_obj(const QString &filename) { quad.save_quad_to_obj(filename); }

void Scene3DSurface::save_quad_mesh_sharp_edge_candidate(const QString &filename) {
    quad.save_quad_sharp_edge_candidate(filename);
}

void Scene3DSurface::open_quad_mesh_sharp_edge_candidate(const QString &filename) {
    quad.load_quad_sharp_edge_candidate(filename);
}

void Scene3DSurface::auto_tool(const std::string &oracle_folder, const std::string &result_folder,
                               const std::string &mesh_name) {
    std::cout << green << "Mesh '" << mesh_name << "' ..." << white << std::endl;
    std::cout << std::endl;

    const QString mesh_name_q = mesh_name.c_str();
    const QString oracle_f = oracle_folder.c_str();
    const QString result_f = result_folder.c_str();
    QDir(result_f).mkdir(mesh_name_q);

    const QString done_file = result_f + mesh_name_q + "/done";
    if (QFile(done_file).exists()) {
        std::cout << "already done, skip this mesh" << std::endl;
        return;
    }

    const QString mesh_folder = result_f + mesh_name_q + "/";
    const QString tri_file = mesh_name_q + "-tri.obj";
    const QString quad_file = mesh_name_q + "-quad.obj";

    QFile::copy(oracle_f + tri_file, mesh_folder + tri_file);
    QFile::copy(oracle_f + quad_file, mesh_folder + quad_file);
    open_surface_mesh(mesh_folder + tri_file, "", 0, 0);
    quad.load_quad(mesh_folder + quad_file);
    std::ofstream log(mesh_folder.toStdString() + "info.txt");
    log << quad.get_statistics() << std::endl;

    quad.qzip_and_laplacian(5);
    quad.save_quad(mesh_folder + "5.quad");
    quad.save_quad_to_obj(mesh_folder + "5.obj");
    log << "5 iters";
    log << quad.get_statistics() << std::endl;

    quad.qzip_and_laplacian(5);
    quad.save_quad(mesh_folder + "10.quad");
    quad.save_quad_to_obj(mesh_folder + "10.obj");
    log << "10 iters";
    log << quad.get_statistics() << std::endl;
    std::ofstream done(done_file.toStdString());
}

void Scene3DSurface::render() const {
    ::glDisable(GL_LIGHTING);
    if (m_view_oracle_sharp_edge_sample && oracle) {
        const auto &samples = oracle->sharp_edge_samples();
        glPointSize(2.0f);
        glColor3ub(20, 100, 20);
        glBegin(GL_POINTS);
        for (const Point_3 &p : samples) {
            glVertex3d(p.x(), p.y(), p.z());
        }
        glEnd();
    }
    if (m_view_quads)
        quad.gl_draw(ramp);
    if (m_view_oracle && oracle)
        oracle->render();
}

bool Scene3DSurface::get_intersection(Kernel::Ray_3 ray, Kernel::Point_3 &center) {
    return oracle && oracle->get_ray_intersection(ray, center);
}

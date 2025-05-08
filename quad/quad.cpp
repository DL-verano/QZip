#include "quad.h"
#include "../tools/kd_tree_3d.h"
#include "../tools/scope_timer.h"
#include "include_opengl.h"
#include "read_obj.h"
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <QFileInfo>
#include <fstream>
#include <random>
#include <set>

typedef Quad2d3d::Vertex_handle V;
typedef Quad2d3d::Vertex_handle_const cV;

int Quad2d3d::Vertex::flag_counter = 0;
int Quad2d3d::Vertex::id_counter = 0;
int Quad2d3d::Quad::id_counter = 0;

// basic structure

Vector_3 Quad2d3d::Quad::calc_normal() const {
    const Point_3 &p0 = corners[0]->point;
    const Point_3 &p1 = corners[1]->point;
    const Point_3 &p2 = corners[2]->point;
    const Point_3 &p3 = corners[3]->point;
    const Vector_3 n012 = CGAL::normal(p0, p1, p2);
    const Vector_3 n023 = CGAL::normal(p0, p2, p3);
    const Vector_3 n123 = CGAL::normal(p1, p2, p3);
    const Vector_3 n130 = CGAL::normal(p1, p3, p0);

    const Vector_3 normal = safe_normalize(n012) + safe_normalize(n023) + safe_normalize(n123) + safe_normalize(n130);

    return safe_normalize(normal);
}

double Quad2d3d::Quad::area() const {
    const Point_3 &p0 = corners[0]->point;
    const Point_3 &p1 = corners[1]->point;
    const Point_3 &p2 = corners[2]->point;
    const Point_3 &p3 = corners[3]->point;
    const Point_3 &m = CGAL::midpoint(CGAL::midpoint(p0, p1), CGAL::midpoint(p2, p3));
    return std::sqrt(CGAL::squared_area(m, p0, p1)) + std::sqrt(CGAL::squared_area(m, p1, p2)) +
           std::sqrt(CGAL::squared_area(m, p2, p3)) + std::sqrt(CGAL::squared_area(m, p3, p0));
}

double Quad2d3d::Quad::angle(int k) const {
    const auto v = corners.at(k);
    const auto v_pre = corners[(k + 1) % 4];
    const auto v_next = corners[(k + 3) % 4];
    Vector_3 va = v_pre->point - v->point;
    Vector_3 vc = v_next->point - v->point;
    return std::acos((va / std::sqrt(va.squared_length())) * (vc / std::sqrt(vc.squared_length())));
}

void Quad2d3d::clear() {
    vertices.clear();
    quads.clear();
    vid_map.clear();
    compass_path.clear();
    simple_path.clear();
}

void Quad2d3d::construct_from_vertices_quads(std::list<Vertex> &&vertices_, std::list<Quad> &&quads_,
                                             const std::vector<int> *frozen_quad_ids) {
    this->clear();
    vertices = std::move(vertices_);
    quads = std::move(quads_);

    Vertex::id_counter = 0;
    Quad::id_counter = 0;

    std::unordered_map<Vertex *, std::vector<Quad_handle>> v_to_quads;
    const std::set<int> frozen_quad_set(begin(*frozen_quad_ids), end(*frozen_quad_ids));

    // update links between items
    for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
        if (frozen_quad_set.find(qit->id) != frozen_quad_set.end()) {
            qit->is_frozen = true;
        }
        for (int i = 0; i < 4; i++) {
            qit->corners.at(i)->quad = qit;
            v_to_quads[&*qit->corners.at(i)].push_back(qit);
        }
        Quad::id_counter = std::max(Quad::id_counter, qit->id + 1);
    }
    for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
        for (int i = 0; i < 4; i++) {
            const auto vit = qit->corners.at(i);
            const auto vit_1 = qit->corners[(i + 1) % 4];
            const auto &nbs = v_to_quads.at(&*vit);
            qit->neighbors.at(i) = quads.end();
            qit->sharp_edge_neighbors.at(i) = quads.end();
            for (const Quad_handle &nb : nbs) {
                if (nb->has(vit_1) && nb != qit) {
                    qit->neighbors.at(i) = nb;
                    break;
                }
            }
        }
    }
    for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
        vit->on_boundary = vit->on_boundary_or_sharp_edge = is_boundary_or_on_sharp_edge(vit);
        if (vit->on_boundary_or_sharp_edge)
            rotate_vertex_quad_to_boundary(vit);
        Vertex::id_counter = std::max(Vertex::id_counter, vit->id + 1);
    }

    for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
        update_vertex_valence(vit);
    }

    vid_map.clear();
    for (auto vit = vertices.begin(); vit != vertices.end(); ++vit)
        vid_map[vit->id] = vit;

    // checks
    check_valid();
    update_current_average_side_length();
}

std::vector<Point_3> Quad2d3d::dump_vertices() const {
    std::vector<Point_3> vs;
    for (const auto &v : vertices)
        vs.push_back(v.point);

    return vs;
}

void Quad2d3d::check_valid() const {
    for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
        for (int i = 0; i < 4; i++) {
            const auto vit_0 = qit->corners.at(i);
            const auto vit_1 = qit->corners[(i + 1) % 4];
            const auto nb = qit->neighbors.at(i);
            if (nb != quads.end()) {
                const int k0 = nb->index(vit_0);
                const int k1 = nb->index(vit_1);
                if (k0 != (k1 + 1) % 4)
                    throw;
                if (qit != nb->neighbors.at(k1))
                    throw;
            }
            const auto sharp_nb = qit->sharp_edge_neighbors.at(i);
            if (nb != quads.end() && sharp_nb != quads.end())
                throw;
            if (sharp_nb != quads.end()) {
                if (!sharp_nb->has_sharp_neighbor(qit))
                    throw;
                if (vit_0->pre_vid_over_sharp_edge < 0 && vit_0->next_vid_over_sharp_edge < 0)
                    throw;
                if (vit_1->pre_vid_over_sharp_edge < 0 && vit_1->next_vid_over_sharp_edge < 0)
                    throw;
            }
        }
    }
    for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
        if (vit->on_boundary_or_sharp_edge != is_boundary_or_on_sharp_edge(vit))
            throw;
        if (vit->on_boundary != is_boundary(vit))
            throw;
        if (vit->on_boundary_or_sharp_edge)
            if (rotate_quad_cw(vit->quad, vit) != quads.end())
                throw;
        if (vid_map.at(vit->id) != vit)
            throw;
        if (!vit->on_boundary_or_sharp_edge && vit->is_sharp_vertex)
            throw;
        if (vit->on_boundary_or_sharp_edge && !vit->on_boundary) {
            if (vit->pre_vid_over_sharp_edge < 0 && vit->next_vid_over_sharp_edge < 0)
                throw;
            if (vit->pre_vid_over_sharp_edge >= 0) {
                vid_map.at(vit->pre_vid_over_sharp_edge);
            }
            if (vit->next_vid_over_sharp_edge >= 0) {
                vid_map.at(vit->next_vid_over_sharp_edge);
            }
        }
    }
}

void Quad2d3d::map_sharp_feature_vertex(std::map<int, int> &feature_vid_map, std::set<int> &redundant_vids) const {
    // map sharp vertices sharing the same point to the same vertex
    std::set<int> done_vids;
    for (auto v = vertices.begin(); v != vertices.end(); ++v) {
        if (done_vids.find(v->id) != done_vids.end())
            continue;
        if (v->pre_vid_over_sharp_edge < 0 && v->next_vid_over_sharp_edge < 0 || v->next_vid_over_sharp_edge == v->id) {
            feature_vid_map[v->id] = v->id;
            continue;
        }
        auto cur_v = v;
        while (cur_v->next_vid_over_sharp_edge >= 0) {
            cur_v = vid_map.at(cur_v->next_vid_over_sharp_edge);
            if (cur_v == v)
                break;
        }
        if (cur_v->next_vid_over_sharp_edge < 0) {
            if (v->pre_vid_over_sharp_edge >= 0)
                continue;
            // v is beginning
        } else {
            // it's a ring
        }
        cur_v = v;
        std::vector<Vertex_handle_const> v_ring;
        while (cur_v->next_vid_over_sharp_edge >= 0) {
            if (cur_v != v)
                redundant_vids.insert(cur_v->id);
            v_ring.push_back(cur_v);
            auto ok = done_vids.insert(cur_v->id);
            if (!ok.second)
                throw;
            cur_v = vid_map.at(cur_v->next_vid_over_sharp_edge);
            if (cur_v == v)
                break;
        }
        for (const auto &nbv : v_ring)
            feature_vid_map[nbv->id] = v->id;
    }
}

void Quad2d3d::save_quad(const QString &filename) const {
    std::ofstream output(filename.toStdString());
    std::cout << "save quad..." << std::endl;

    output << "3d ";
    int frozen_quad_num = 0;
    for (const Quad &q : quads) {
        if (q.is_frozen)
            frozen_quad_num++;
    }

    std::map<int, int> feature_vid_map;
    std::set<int> redundant_vids;
    map_sharp_feature_vertex(feature_vid_map, redundant_vids);

    const int vertices_count = vertices.size() - redundant_vids.size();
    std::vector<int> sharp_vids;
    for (auto v = vertices.begin(); v != vertices.end(); ++v) {
        if (v->is_sharp_vertex && redundant_vids.find(v->id) == redundant_vids.end())
            sharp_vids.push_back(v->id);
    }

    std::vector<std::pair<Quad_handle_const, int>> sharp_edges;
    for (auto q = quads.begin(); q != quads.end(); ++q) {
        for (int i = 0; i < 4; i++) {
            if (q->sharp_edge_neighbors[i] != quads.end()) {
                const auto nb = q->sharp_edge_neighbors[i];
                if (q->id < nb->id)
                    sharp_edges.emplace_back(q, i);
            }
        }
    }

    output << vertices_count << " vertices " << quads.size() << " quads";
    output << " " << frozen_quad_num << " frozen quads";
    output << " " << sharp_vids.size() << " sharp vertices";
    output << " " << sharp_edges.size() << " sharp edges";
    output << "\n";

    output << "#vertices: (id, x, y, z)\n";
    for (const Vertex &v : vertices) {
        if (redundant_vids.find(v.id) != redundant_vids.end())
            continue;
        output << v.id << "\t" << v.point.x() << "\t" << v.point.y() << "\t" << v.point.z();
        output << "\n";
    }
    output << "#quads: (id, v0, v1, v2, v3):\n";
    for (const Quad &q : quads) {
        output << q.id;
        for (int i = 0; i < 4; i++) {
            int vid = q.corners.at(i)->id;
            vid = feature_vid_map.at(vid);
            output << "\t" << vid;
        }
        output << "\n";
    }

    output << "#frozen quad ids:\n";
    for (const Quad &q : quads) {
        if (q.is_frozen)
            output << q.id << "\n";
    }
    output << "#sharp vertices id\n";
    for (int id : sharp_vids) {
        output << id << "\n";
    }
    output << "#sharp edges, pair of integer: (quad_id, 0-3 side)\n";
    for (const auto &edge : sharp_edges) {
        output << edge.first->id << " " << edge.second << "\n";
    }
    output.close();
    std::cout << "done" << std::endl;
}

void Quad2d3d::save_quad_to_obj(const QString &filename) const {
    std::ofstream output(filename.toStdString());
    std::cout << "save quad..." << std::endl;

    std::map<int, int> feature_vid_map;
    std::set<int> redundant_vids;
    map_sharp_feature_vertex(feature_vid_map, redundant_vids);

    std::map<int, int> vid_to_idx;

    int count = 1;
    for (const Vertex &v : vertices) {
        if (redundant_vids.find(v.id) != redundant_vids.end())
            continue;
        output << "v " << v.point.x() << " " << v.point.y() << " " << v.point.z();
        output << "\n";
        vid_to_idx[v.id] = count++;
    }

    for (const Quad &q : quads) {
        output << "f";
        for (int i = 0; i < 4; i++) {
            int vid = q.corners.at(i)->id;
            vid = feature_vid_map.at(vid);
            const int idx = vid_to_idx.at(vid);
            output << " " << idx;
        }
        output << "\n";
    }
}

void Quad2d3d::load_quad(const QString &filename) {
    std::list<Vertex> vertices_input;
    std::list<Quad> quads_input;
    std::vector<int> frozen_quad_ids;
    std::vector<int> sharp_vids;
    std::vector<std::pair<int, int>> sharp_edges;
    if (QFileInfo(filename).suffix().compare("obj", Qt::CaseInsensitive) == 0) {
        std::vector<std::array<double, 3>> pts;
        std::vector<std::vector<int>> faces;
        if (!ReadOBJFile(filename.toStdString(), pts, faces))
            return;
        for (const auto &f : faces)
            if (f.size() != 4) {
                std::cout << "Error in loading OBJ file as quad, non quad face exists" << std::endl;
                return;
            }
        std::vector<V> vid_to_handle;

        for (int i = 0; i < pts.size(); i++) {
            Vertex v;
            v.id = i;
            v.point = Point_3(pts[i][0], pts[i][1], pts[i][2]);
            vertices_input.push_back(v);
            vid_to_handle.push_back(--vertices_input.end());
        }
        for (int i = 0; i < faces.size(); i++) {
            Quad quad;
            quad.id = i;
            for (int k = 0; k < 4; k++)
                quad.corners[k] = vid_to_handle.at(faces[i][k]);
            quads_input.push_back(quad);
        }
    } else if (QFileInfo(filename).suffix().compare("quad", Qt::CaseInsensitive) == 0) {
        char line[512];
        std::ifstream input(filename.toStdString());
        std::cout << "read quad..." << std::endl;
        input.getline(line, 512);
        int nv, nq, n_frozen_q = 0, n_sharp_vertex = 0, n_sharp_edge = 0;
        int nums = sscanf(line, "3d %d vertices %d quads %d frozen quads %d sharp vertices %d sharp edges", &nv, &nq,
                          &n_frozen_q, &n_sharp_vertex, &n_sharp_edge);
        if (nums != 2 && nums != 3 && nums != 4 && nums != 5) {
            std::cout << "error reading quad" << std::endl;
            return;
        }
        auto getline = [&]() -> bool {
            while (true) {
                if (!input.getline(line, sizeof(line))) {
                    std::cout << "error when reading" << std::endl;
                    return false;
                }
                if (strlen(line) > 0 && line[0] == '#') {
                    continue;
                } else {
                    return true;
                }
            }
        };
        std::map<int, V> vid_to_handle;
        for (int i = 0; i < nv; i++) {
            if (!getline()) {
                std::cout << "error when reading" << std::endl;
                return;
            }
            int vid;
            double x, y, z;
            if (sscanf(line, "%d\t%lf\t%lf\t%lf", &vid, &x, &y, &z) == 4) {
                Vertex v;
                v.id = vid;
                v.point = Point_3(x, y, z);
                vertices_input.push_back(v);
                vid_to_handle[v.id] = --vertices_input.end();
            } else {
                std::cout << "error when reading" << std::endl;
                return;
            }
        }

        for (int i = 0; i < nq; i++) {
            if (!getline()) {
                std::cout << "error when reading" << std::endl;
                return;
            }
            int qid;
            int vids[4];
            if (sscanf(line, "%d\t%d\t%d\t%d\t%d", &qid, &vids[0], &vids[1], &vids[2], &vids[3]) == 5) {
                Quad quad;
                quad.id = qid;
                for (int k = 0; k < 4; k++)
                    quad.corners[k] = vid_to_handle.at(vids[k]);
                quads_input.push_back(quad);
            } else {
                std::cout << "error when reading" << std::endl;
                return;
            }
        }
        for (int i = 0; i < n_frozen_q; i++) {
            if (!getline()) {
                std::cout << "error when reading" << std::endl;
                return;
            }
            int qid;
            if (sscanf(line, "%d", &qid) == 1) {
                frozen_quad_ids.push_back(qid);
            } else {
                std::cout << "error when reading" << std::endl;
                return;
            }
        }
        for (int i = 0; i < n_sharp_vertex; i++) {
            if (!getline()) {
                std::cout << "error when reading" << std::endl;
                return;
            }
            int vid;
            if (sscanf(line, "%d", &vid) == 1) {
                sharp_vids.push_back(vid);
            } else {
                std::cout << "error when reading" << std::endl;
                return;
            }
        }
        for (int i = 0; i < n_sharp_edge; i++) {
            if (!getline()) {
                std::cout << "error when reading" << std::endl;
                return;
            }
            int qid, k;
            if (sscanf(line, "%d %d", &qid, &k) == 2) {
                sharp_edges.emplace_back(qid, k);
            } else {
                std::cout << "error when reading" << std::endl;
                return;
            }
        }
        input.close();
    }

    construct_from_vertices_quads(std::move(vertices_input), std::move(quads_input), &frozen_quad_ids);

    if (!sharp_edges.empty()) {
        std::map<int, Quad_handle> qid_map;
        for (auto q = quads.begin(); q != quads.end(); ++q) {
            qid_map[q->id] = q;
        }
        std::vector<std::pair<Quad_handle, int>> edges;
        for (const auto &edge : sharp_edges) {
            edges.emplace_back(qid_map.at(edge.first), edge.second);
        }
        split_given_sharp_edges(edges, 0);
        for (int vid : sharp_vids)
            vid_map.at(vid)->is_sharp_vertex = true;
    }

    std::cout << "done" << std::endl;
}

void Quad2d3d::save_quad_sharp_edge_candidate(const QString &filename) const {
    std::ofstream output(filename.toStdString());
    std::cout << "save quad sharp edge candidate..." << std::endl;
    for (auto q = quads.begin(); q != quads.end(); ++q) {
        for (int k = 0; k < 4; k++)
            if (q->is_sharp_edge_candidate[k])
                output << q->id << " " << k << std::endl;
    }
    output.close();
    std::cout << "done" << std::endl;
}

void Quad2d3d::load_quad_sharp_edge_candidate(const QString &filename) {
    std::cout << "load quad sharp edge candidate..." << std::endl;
    char line[512];
    std::ifstream input(filename.toStdString());
    std::map<int, Quad_handle> qmap;
    for (auto q = quads.begin(); q != quads.end(); ++q) {
        qmap[q->id] = q;
        for (int k = 0; k < 4; k++)
            q->is_sharp_edge_candidate[k] = false;
    }
    while (input.getline(line, 512)) {
        int qid, k;
        if (sscanf(line, "%d %d", &qid, &k) == 2) {
            qmap.at(qid)->is_sharp_edge_candidate[k] = true;
        } else {
            std::cout << "skip invalid line: " << line << std::endl;
        }
    }
    std::cout << "done" << std::endl;
}

bool Quad2d3d::is_boundary_or_on_sharp_edge(Vertex_handle_const v) const {
    auto qit = v->quad;
    do {
        const int k = qit->index(v);
        if (qit->neighbors.at(k) == quads.end() || qit->neighbors.at((k + 3) % 4) == quads.end())
            return true;
        qit = rotate_quad_ccw(qit, v);
    } while (qit != v->quad);
    return false;
}

bool Quad2d3d::is_boundary(Vertex_handle_const v) const {
    if (v->next_vid_over_sharp_edge >= 0 || v->pre_vid_over_sharp_edge >= 0) {
        auto cur_v = v;
        while (cur_v->next_vid_over_sharp_edge >= 0 && cur_v->next_vid_over_sharp_edge != v->id) {
            cur_v = vid_map.at(cur_v->next_vid_over_sharp_edge);
        }
        return cur_v->next_vid_over_sharp_edge != v->id;
    }
    return is_boundary_or_on_sharp_edge(v);
}

void Quad2d3d::rotate_vertex_quad_to_boundary(Vertex_handle v) {
    auto qit = v->quad;
    do {
        const int k = qit->index(v);
        auto q_next = qit->neighbors.at(k);
        if (q_next == quads.end()) {
            v->quad = qit;
            return;
        }
        qit = q_next;
    } while (qit != v->quad);
}

Quad2d3d::Quad_handle_const Quad2d3d::get_quad_from_edge_ccw(Vertex_handle_const v0, Vertex_handle_const v1) const {
    auto qit = v0->quad;
    do {
        const int k = qit->index(v0);
        if (qit->corners[(k + 1) % 4] == v1)
            return qit;
        qit = rotate_quad_ccw(qit, v0);
    } while (qit != v0->quad && qit != quads.end());
    return quads.end();
}

Quad2d3d::Quad_handle Quad2d3d::get_quad_from_edge_ccw(Vertex_handle v0, Vertex_handle v1) {
    auto qit = v0->quad;
    do {
        const int k = qit->index(v0);
        if (qit->corners[(k + 1) % 4] == v1)
            return qit;
        qit = rotate_quad_ccw(qit, v0);
    } while (qit != v0->quad && qit != quads.end());
    return quads.end();
}

bool Quad2d3d::locate_vertex(const Point_3 &p, Vertex_handle &v) {
    double min_sqd = std::numeric_limits<double>::max();
    for (auto vit = begin(vertices); vit != end(vertices); ++vit) {
        const double sqd = CGAL::squared_distance(vit->point, p);
        if (sqd < min_sqd) {
            v = vit;
            min_sqd = sqd;
        }
    }
    return true;
}

std::string Quad2d3d::get_statistics() const {
    char buf[8192];
    sprintf(buf, "%d quads, %d vertices\n", quads.size(), vertices.size());
    std::map<int, int> valences;
    for (auto v = begin(vertices); v != end(vertices); ++v) {
        valences[effective_valence(v)]++;
    }
    int valence_2 = 0;
    int valence_3 = 0;
    int valence_5 = 0;
    int valence_6 = 0;
    int valence_7_and_higher = 0;
    for (const auto &pair : valences) {
        if (pair.first == 2) {
            valence_2 = pair.second;
        } else if (pair.first == 3) {
            valence_3 = pair.second;
        } else if (pair.first == 5) {
            valence_5 = pair.second;
        } else if (pair.first == 6) {
            valence_6 = pair.second;
        } else if (pair.first >= 7) {
            valence_7_and_higher = pair.second;
        }
    }
    sprintf(buf + strlen(buf), "Singular vertices:\n");
    sprintf(buf + strlen(buf), " v2: %6d (%4.1f%%)\n", valence_2, double(valence_2) / vertices.size() * 100);
    sprintf(buf + strlen(buf), " v3: %6d (%4.1f%%)\n", valence_3, double(valence_3) / vertices.size() * 100);
    sprintf(buf + strlen(buf), " v5: %6d (%4.1f%%)\n", valence_5, double(valence_5) / vertices.size() * 100);
    sprintf(buf + strlen(buf), " v6: %6d (%4.1f%%)\n", valence_6, double(valence_6) / vertices.size() * 100);
    sprintf(buf + strlen(buf), ">v6: %6d (%4.1f%%)\n", valence_7_and_higher,
            double(valence_7_and_higher) / vertices.size() * 100);
    return buf;
}

void Quad2d3d::print_statistics() const { std::cout << get_statistics() << std::endl; }

void Quad2d3d::get_bounding_sphere(double &x, double &y, double &z, double &r) const {
    double xmin = std::numeric_limits<double>::max();
    double ymin = std::numeric_limits<double>::max();
    double zmin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::lowest();
    double ymax = std::numeric_limits<double>::lowest();
    double zmax = std::numeric_limits<double>::lowest();
    for (auto vit = begin(vertices); vit != end(vertices); ++vit) {
        const auto &p = vit->point;
        xmin = std::min(xmin, p.x());
        ymin = std::min(ymin, p.y());
        zmin = std::min(zmin, p.z());
        xmax = std::max(xmax, p.x());
        ymax = std::max(ymax, p.y());
        zmax = std::max(zmax, p.z());
    }
    x = (xmin + xmax) / 2.;
    y = (ymin + ymax) / 2.;
    z = (zmin + zmax) / 2.;
    r = std::sqrt((x - xmin) * (x - xmin) + (y - ymin) * (y - ymin) + (z - zmin) * (z - zmin));
}

void Quad2d3d::highlight_vertex_id(int id) {
    if (highlighted_vertex_id >= 0)
        highlighted_vertex_id = -1;
    else
        highlighted_vertex_id = id;
}

void Quad2d3d::highlight_quad_id(int id) {
    if (highlighted_quad_id >= 0)
        highlighted_quad_id = -1;
    else
        highlighted_quad_id = id;
}

void Quad2d3d::calculate_quad_stretch(Quad_handle q) {
    const Point_3 m01 = CGAL::midpoint(q->corners[0]->point, q->corners[1]->point);
    const Point_3 m12 = CGAL::midpoint(q->corners[1]->point, q->corners[2]->point);
    const Point_3 m23 = CGAL::midpoint(q->corners[2]->point, q->corners[3]->point);
    const Point_3 m30 = CGAL::midpoint(q->corners[3]->point, q->corners[0]->point);

    const Vector_3 H = m12 - m30;
    const Vector_3 V = m23 - m01;
    const Vector_3 L02 = q->corners[2]->point - q->corners[0]->point;
    const Vector_3 L13 = q->corners[3]->point - q->corners[1]->point;

    const double lH = std::sqrt(H.squared_length());
    const double lV = std::sqrt(V.squared_length());
    const double l02 = std::sqrt(L02.squared_length());
    const double l13 = std::sqrt(L13.squared_length());

    const double maxHV = std::max(lH / lV, lV / lH);
    const double maxDiag = std::max(l02 / l13, l13 / l02);

    if (maxHV > maxDiag) {
        if (lH > long_quad_threshold * lV) {
            q->stretch_direction = 1;
            q->stretch_ratio = lH / lV;
        } else if (lV > long_quad_threshold * lH) {
            q->stretch_direction = 2;
            q->stretch_ratio = lV / lH;
        } else {
            q->stretch_direction = 0;
        }
    } else {
        if (l02 > long_quad_threshold * l13) {
            q->stretch_direction = 3;
            q->stretch_ratio = l02 / l13;
        } else if (l13 > long_quad_threshold * l02) {
            q->stretch_direction = 4;
            q->stretch_ratio = l13 / l02;
        } else {
            q->stretch_direction = 0;
        }
    }
}

void Quad2d3d::update_all_quad_stretch() {
    for (auto it = quads.begin(); it != quads.end(); ++it)
        it->stretch_direction = 0;
    if (!use_shape_score)
        return;
    for (auto it = quads.begin(); it != quads.end(); ++it)
        calculate_quad_stretch(it);
}

void Quad2d3d::calculate_quad_size_ratio(Quad_handle q) {
    const double q_area = q->area();
    const double q_side_length = std::sqrt(q_area);

    if (q_side_length > current_average_side_length * size_ratio_threshold ||
        q_side_length < current_average_side_length / size_ratio_threshold) {
        q->problematic_size = true;
        q->size_ratio = q_side_length / current_average_side_length;
    }
}

void Quad2d3d::update_current_average_side_length() {
    double area_sum = 0.;
    for (auto it = quads.begin(); it != quads.end(); ++it) {
        area_sum += it->area();
    }
    current_average_side_length = std::sqrt(area_sum / quads.size());
}

void Quad2d3d::update_all_quad_size_ratio() {
    for (auto it = quads.begin(); it != quads.end(); ++it)
        it->problematic_size = false;
    if (!use_size_score) {
        return;
    }
    update_current_average_side_length();
    for (auto it = quads.begin(); it != quads.end(); ++it) {
        calculate_quad_size_ratio(it);
    }
}

double Quad2d3d::calc_shape_score(double quad_stretch_ratio) const {
    return std::pow((quad_stretch_ratio - 1.0) * shape_score_strength, shape_score_gamma);
}

double Quad2d3d::calc_size_score(double quad_size_ratio) const {
    if (quad_size_ratio < 1.0)
        quad_size_ratio = 1.0 / quad_size_ratio;
    return std::pow((quad_size_ratio - 1.0) * size_score_strength, size_score_gamma);
}

void Quad2d3d::guess_quad_sharp_feature_from_oracle() {
    double length_sum = 0.;
    int length_count = 0;
    for (auto q = quads.begin(); q != quads.end(); ++q) {
        for (int k = 0; k < 4; k++) {
            if (q->neighbors[k] != quads.end() && q->id > q->neighbors[k]->id)
                continue;
            length_sum += std::sqrt(CGAL::squared_distance(q->corners[k]->point, q->corners[(k + 1) % 4]->point));
            length_count++;
        }
    }
    const double ave_length = length_sum / length_count;

    const std::vector<Point_3> &samples = oracle->sharp_edge_samples();

    KdTree3D v_tree;
    std::vector<Point_3> all_v_pts;
    std::vector<Vertex_handle> vhs;
    for (auto v = vertices.begin(); v != vertices.end(); ++v) {
        all_v_pts.push_back(v->point);
        vhs.push_back(v);
    }
    v_tree.set_points(all_v_pts);
    std::unordered_set<Vertex_handle, Vertex_handle_hash> sharp_vs;
    {
        std::vector<int> ids(samples.size());
#pragma omp parallel for
        for (int a = 0; a < samples.size(); a++) {
            const Point_3 &s = samples[a];
            int idx = v_tree.query(s);
            if (std::sqrt(CGAL::squared_distance(all_v_pts[idx], s)) < 0.3 * ave_length) {
                ids[a] = idx;
            } else {
                ids[a] = -1;
            }
        }
        const std::set<int> indices(begin(ids), end(ids));
        for (int i : indices) {
            if (i >= 0)
                sharp_vs.insert(vhs[i]);
        }
    }

    KdTree3D tree;
    std::vector<Point_3> all_pts;
    std::vector<std::pair<Quad_handle, int>> edges;
    for (auto q = quads.begin(); q != quads.end(); ++q) {
        for (int k = 0; k < 4; k++) {
            q->is_sharp_edge_candidate[k] = false;
            if (q->neighbors[k] != quads.end() && q->id > q->neighbors[k]->id)
                continue;
            const auto v0 = q->corners[k];
            const auto v1 = q->corners[(k + 1) % 4];
            if (sharp_vs.find(v0) == sharp_vs.end() || sharp_vs.find(v1) == sharp_vs.end())
                continue;
            const Point_3 m = CGAL::midpoint(v0->point, v1->point);
            all_pts.push_back(m);
            edges.emplace_back(q, k);
        }
    }
    tree.set_points(all_pts);

    std::vector<std::pair<Quad_handle, int>> sharp_edges;
    {
        std::vector<int> ids(samples.size());
#pragma omp parallel for
        for (int a = 0; a < samples.size(); a++) {
            const Point_3 &s = samples[a];
            int idx = tree.query(s);
            if (std::sqrt(CGAL::squared_distance(all_pts[idx], s)) < 0.3 * ave_length) {
                ids[a] = idx;
            } else {
                ids[a] = -1;
            }
        }
        const std::set<int> indices(begin(ids), end(ids));
        for (int i : indices) {
            if (i >= 0)
                sharp_edges.push_back(edges[i]);
        }
    }
    for (const auto &e : sharp_edges) {
        e.first->is_sharp_edge_candidate[e.second] = true;
    }
}

void Quad2d3d::confirm_sharp_feature_guess(double sharp_edge_turn_angle) {
    std::vector<std::pair<Quad_handle, int>> edges;
    for (auto q = quads.begin(); q != quads.end(); ++q) {
        for (int k = 0; k < 4; k++) {
            if (q->is_sharp_edge_candidate[k]) {
                q->is_sharp_edge_candidate[k] = false;
                edges.emplace_back(q, k);
            }
        }
    }
    split_given_sharp_edges(edges, sharp_edge_turn_angle);
}

void Quad2d3d::split_given_sharp_edges(const std::vector<std::pair<Quad_handle, int>> &edges,
                                       double sharp_edge_turn_angle) {
    for (auto &pair : edges) {
        auto q = pair.first;
        int k = pair.second;
        const auto nb = q->neighbors.at(k);
        if (nb == quads.end())
            continue;

        q->sharp_edge_neighbors.at(k) = q->neighbors.at(k);
        q->neighbors.at(k) = quads.end();
        const auto &vk = q->corners[k];

        const int k_nb = (nb->index(vk) + 3) % 4;
        nb->sharp_edge_neighbors.at(k_nb) = nb->neighbors.at(k_nb);
        nb->neighbors.at(k_nb) = quads.end();
    }
    split_vertices_by_sharp_edges(sharp_edge_turn_angle);
    check_valid();
}

void Quad2d3d::mark_sharp_vertex_by_angle(bool use_sharp_vertex, double sharp_angle_degree,
                                          double sharp_edge_turn_angle) {
    check_valid();
    if (!use_sharp_vertex)
        return;
    std::vector<std::pair<Quad_handle, int>> edges;
    for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
        const Vector_3 normal = qit->calc_normal();
        for (int k = 0; k < 4; k++) {
            const auto nb = qit->neighbors.at(k);
            if (nb == quads.end())
                continue;
            if (nb->id < qit->id)
                continue;
            const Vector_3 nb_n = nb->calc_normal();
            const double angle =
                180. - std::acos(std::max(-1.0, std::min(1.0, normal * nb_n))) * 180. / (std::atan(1.) * 4);
            if (angle < sharp_angle_degree) {
                edges.emplace_back(qit, k);
            }
        }
    }
    split_given_sharp_edges(edges, sharp_edge_turn_angle);
}

void Quad2d3d::change_edge_feature_type(Quad_handle q, const Point_3 &intersection) {
    Point_3 mids[4];
    for (int k = 0; k < 4; k++)
        mids[k] = CGAL::midpoint(q->corners[k]->point, q->corners[(k + 1) % 4]->point);

    double min_d = std::numeric_limits<double>::max();
    int min_k = -1;
    for (int k = 0; k < 4; k++) {
        double d = CGAL::squared_distance(mids[k], intersection);
        if (d < min_d) {
            min_d = d;
            min_k = k;
        }
    }
    const auto nb = q->neighbors[min_k];

    if (nb != quads.end() && q->id > nb->id) {
        const int mirror_k = nb->index(q);
        q = nb;
        min_k = mirror_k;
    }
    q->is_sharp_edge_candidate.at(min_k) = !q->is_sharp_edge_candidate.at(min_k);
}

std::pair<Quad2d3d::Vertex_handle, Quad2d3d::Quad_handle> Quad2d3d::pick(Kernel::Ray_3 &ray, Point_3 &intersection) {
    typedef CGAL::Simple_cartesian<double> AABB_Kernel;
    typedef std::vector<AABB_Kernel::Triangle_3>::iterator Triangle_Iterator;
    typedef CGAL::AABB_triangle_primitive<AABB_Kernel, Triangle_Iterator> Triangle_Primitive;
    typedef CGAL::AABB_traits<AABB_Kernel, Triangle_Primitive> AABB_triangle_traits;
    typedef CGAL::AABB_tree<AABB_triangle_traits> Triangle_AABBTree;
    std::vector<AABB_Kernel::Triangle_3> triangles_aabb;

    CGAL::Cartesian_converter<Kernel, AABB_Kernel> converter;
    std::vector<Quad_handle> idx_to_quad;

    // collect boundary triangles
    for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
        AABB_Kernel::Triangle_3 t0(converter(qit->corners[0]->point), converter(qit->corners[1]->point),
                                   converter(qit->corners[2]->point));
        AABB_Kernel::Triangle_3 t1(converter(qit->corners[0]->point), converter(qit->corners[2]->point),
                                   converter(qit->corners[3]->point));
        triangles_aabb.push_back(t0);
        triangles_aabb.push_back(t1);
        idx_to_quad.push_back(qit);
        idx_to_quad.push_back(qit);
    }

    Triangle_AABBTree aabb;
    aabb.insert(triangles_aabb.begin(), triangles_aabb.end());
    aabb.accelerate_distance_queries();

    CGAL::Cartesian_converter<AABB_Kernel, Kernel> inverser;
    typedef boost::optional<Triangle_AABBTree::Intersection_and_primitive_id<AABB_Kernel::Ray_3>::Type>
        Ray_intersection;
    typedef AABB_Kernel::Point_3 Pt3;
    Ray_intersection inter = aabb.first_intersection(converter(ray));
    Point_3 p;
    Quad_handle q;
    if (inter && boost::get<Pt3>(&(inter->first))) {
        const Pt3 *pt3 = boost::get<Pt3>(&(inter->first));
        const Triangle_Iterator tit = inter->second;
        p = inverser(*pt3);
        q = idx_to_quad.at(int(tit - triangles_aabb.begin()));
    } else {
        return std::make_pair(vertices.end(), quads.end());
    }
    double max_d2 = std::numeric_limits<double>::max();
    Vertex_handle v;
    for (int k = 0; k < 4; k++) {
        const double d2 = CGAL::squared_distance(q->corners[k]->point, p);
        if (d2 < max_d2) {
            max_d2 = d2;
            v = q->corners[k];
        }
    }
    intersection = p;
    return std::make_pair(v, q);
}

void Quad2d3d::froze_quad(const Quad_handle &q) { q->is_frozen = !q->is_frozen; }

void Quad2d3d::isolate_frozen_quad() {
    for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
        if (!qit->is_frozen)
            continue;
        for (int k = 0; k < 4; k++) {
            const auto nb = qit->neighbors.at(k);
            if (nb == quads.end())
                continue;
            if (nb->is_frozen)
                continue;
            qit->sharp_edge_neighbors.at(k) = qit->neighbors.at(k);
            qit->neighbors.at(k) = quads.end();
            const int k_nb = nb->index(qit);
            nb->sharp_edge_neighbors.at(k_nb) = nb->neighbors.at(k_nb);
            nb->neighbors.at(k_nb) = quads.end();
        }
    }
    split_vertices_by_sharp_edges(0.);
    for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
        if (!qit->is_frozen)
            continue;
        for (int k = 0; k < 4; k++) {
            auto v = qit->corners[k];
            v->is_frozen = true;
            if (v->next_vid_over_sharp_edge >= 0) {
                auto v_next = v;
                while (v_next->next_vid_over_sharp_edge >= 0 && v_next->next_vid_over_sharp_edge != v->id) {
                    v_next = vid_map.at(v_next->next_vid_over_sharp_edge);
                    v_next->is_frozen = true;
                }
            }
            if (v->pre_vid_over_sharp_edge >= 0) {
                auto v_pre = v;
                while (v_pre->pre_vid_over_sharp_edge >= 0 && v_pre->pre_vid_over_sharp_edge != v->id) {
                    v_pre = vid_map.at(v_pre->pre_vid_over_sharp_edge);
                    v_pre->is_frozen = true;
                }
            }
        }
    }
    check_valid();
}

int Quad2d3d::effective_valence(const Vertex_handle_const &v) const {
    if (v->on_boundary_or_sharp_edge) {
        const int boundary_preferred_valence = calc_vertex_gaussian_curvature_valence(v);
        return 4 + v->valence - boundary_preferred_valence;
    } else {
        return v->valence;
    }
}

void Quad2d3d::split_vertices_by_sharp_edges(double sharp_edge_turn_angle) {
    for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
        rotate_vertex_quad_to_boundary(vit);
    }
    const int size = vertices.size();
    int count = 0;
    auto vit = vertices.begin();
    for (; count < size; ++vit, ++count) {
        auto q = vit->quad;
        auto init_q = q;
        auto cur_v = vit;
        while (q != quads.end()) {
            const int k = q->index(cur_v);
            auto next = q->neighbors[(k + 3) % 4];
            if (next == init_q) {
                break;
            }
            if (next == quads.end()) {
                next = q->sharp_edge_neighbors[(k + 3) % 4];
                if (next == quads.end())
                    break;
                if (next == init_q) {
                    cur_v->next_vid_over_sharp_edge = vit->id;
                    vit->pre_vid_over_sharp_edge = cur_v->id;
                    break;
                }
                vertices.emplace_back();
                auto new_v = vertices.end();
                --new_v;
                new_v->id = Vertex::id_counter++;
                new_v->quad = next;
                new_v->point = cur_v->point;
                cur_v->next_vid_over_sharp_edge = new_v->id;
                new_v->pre_vid_over_sharp_edge = cur_v->id;
                vid_map[new_v->id] = new_v;
                cur_v = new_v;
            }
            q = next;
            q->corners.at(q->index(vit)) = cur_v;
        }
    }
    for (vit = vertices.begin(); vit != vertices.end(); ++vit) {
        update_vertex_valence(vit);
        vit->on_boundary_or_sharp_edge = is_boundary_or_on_sharp_edge(vit);
        vit->on_boundary = is_boundary(vit);
    }

    for (auto v = vertices.begin(); v != vertices.end(); ++v) {
        if (!v->on_boundary_or_sharp_edge) {
            v->is_sharp_vertex = false;
            continue;
        }
        if (v->pre_vid_over_sharp_edge != v->next_vid_over_sharp_edge) {
            v->is_sharp_vertex = true;
            continue;
        }
        const auto q = v->quad;
        const V v_first = q->corners.at((q->index(v) + 1) % 4);
        auto q_last = q;
        do {
            int k = q_last->index(v);
            int k_next = (k + 3) % 4;
            auto q_next = q_last->neighbors[k_next];
            if (q_next == quads.end()) {
                break;
            }
            q_last = q_next;
        } while (true);
        const V v_last = q_last->corners.at((q_last->index(v) + 3) % 4);
        Vector_3 v1 = v_first->point - v->point;
        Vector_3 v2 = v_last->point - v->point;
        v1 /= std::sqrt(v1.squared_length());
        v2 /= std::sqrt(v2.squared_length());
        if (std::acos(std::max(-1., std::min(1., v1 * v2))) / 3.1415 * 180 < sharp_edge_turn_angle)
            v->is_sharp_vertex = true;
    }
}

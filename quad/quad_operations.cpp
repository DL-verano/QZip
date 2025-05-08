#define _USE_MATH_DEFINES
#include "../include/console_color.h"
#include "quad.h"
#include <cmath>
#include <queue>
#include <random>
#include <set>
#include <thread>

typedef Quad2d3d::Vertex_handle V;

//#pragma optimize("", off)

bool Quad2d3d::collapse_quad(const Collapse &collapse, bool tolerate_bad_case) {
    auto quad = collapse.quad;
    int k_nb_mirror[4] = {-1, -1, -1, -1};
    for (int i = 0; i < 4; i++) {
        auto nb = quad->neighbors.at(i);
        if (nb == quads.end())
            continue;
        const int k_v0_nb = nb->index(quad->corners.at(i));
        k_nb_mirror[i] = (k_v0_nb + 3) % 4;
    }
    int k_sharp_nb_mirror[4] = {-1, -1, -1, -1};
    for (int i = 0; i < 4; i++) {
        auto nb = quad->sharp_edge_neighbors.at(i);
        if (nb == quads.end())
            continue;
        bool found = false;
        for (int k = 0; k < 4; k++) {
            const auto v = nb->corners[k];
            if (v->next_vid_over_sharp_edge < 0)
                continue;
            const auto v_next = vid_map.at(v->next_vid_over_sharp_edge);
            if (v_next == quad->corners.at(i)) {
                found = true;
                k_sharp_nb_mirror[i] = (k + 3) % 4;
                break;
            }
        }
        if (!found)
            throw;
    }

    const int k_from = collapse.k_from;
    const int k_to = collapse.k_to;
    if (k_from != (k_to + 2) % 4)
        throw;
    const int k_right = (k_from + 1) % 4;
    const int k_left = (k_to + 1) % 4;

    const auto v_from = quad->corners.at(k_from);
    const auto v_to = quad->corners.at(k_to);
    const auto v_left = quad->corners.at(k_left);
    const auto v_right = quad->corners.at(k_right);

    if (quad->neighbors.at(k_from) == quad->neighbors.at(k_right)) {
        if (tolerate_bad_case)
            throw;
        return false;
    }

    if (quad->neighbors.at(k_to) == quad->neighbors.at(k_left)) {
        if (tolerate_bad_case)
            throw;
        return false;
    }

    if (v_from->on_boundary_or_sharp_edge && v_to->on_boundary_or_sharp_edge)
        throw;

    if (!tolerate_bad_case && (v_from->on_boundary_or_sharp_edge || v_to->on_boundary_or_sharp_edge) &&
        (v_left->on_boundary_or_sharp_edge && v_right->on_boundary_or_sharp_edge)) {
        // in this case, collapse will create a quad that is flat on boundary or sharp feature
        std::cout << "Collapse will lead to flat edge at boundary/feature, abandon" << std::endl;
        return false;
    }

    {
        // map all v_from corner in quads to v_to
        std::vector<std::pair<Quad_handle, int>> quad_k;
        auto qit = v_from->quad;
        do {
            if (qit != quad)
                quad_k.emplace_back(qit, qit->index(v_from));
            qit = rotate_quad_ccw(qit, v_from);
        } while (qit != quads.end() && qit != v_from->quad);
        for (auto &pair : quad_k) {
            pair.first->corners.at(pair.second) = v_to;
            v_to->quad = pair.first;
        }
    }

    for (const auto &pair :
         std::vector<std::pair<int, int>>{{k_from, k_right}, {k_right, k_from}, {k_to, k_left}, {k_left, k_to}}) {
        const int k1 = pair.first;
        const int k2 = pair.second;
        const auto q = quad->neighbors.at(k1);
        if (q == quads.end())
            continue;
        q->neighbors.at(k_nb_mirror[k1]) = quad->neighbors.at(k2);
        const auto v = k1 == k_right || k2 == k_right ? v_right : v_left;
        v->quad = q;
        v_to->quad = q;
    }

    for (const auto &pair :
         std::vector<std::pair<int, int>>{{k_from, k_right}, {k_right, k_from}, {k_to, k_left}, {k_left, k_to}}) {
        const int k1 = pair.first;
        const int k2 = pair.second;
        const auto q = quad->sharp_edge_neighbors[k1];
        if (q == quads.end())
            continue;
        q->sharp_edge_neighbors.at(k_sharp_nb_mirror[k1]) = quad->neighbors.at(k2);
        quad->neighbors.at(k2)->sharp_edge_neighbors.at(k_nb_mirror[k2]) = q;
    }

    if (collapse.has_point_original) {
        v_to->point = collapse.point_original;
    } else {
        if (v_from->on_boundary_or_sharp_edge || v_to->on_boundary_or_sharp_edge) {
            if (v_from->on_boundary_or_sharp_edge && v_to->on_boundary_or_sharp_edge)
                throw;
            if (v_from->on_boundary_or_sharp_edge)
                v_to->point = v_from->point;
        } else {
            v_to->point = CGAL::midpoint(CGAL::midpoint(v_left->point, v_right->point),
                                         CGAL::midpoint(v_from->point, v_to->point));
        }
    }
    if (v_from->pre_vid_over_sharp_edge >= 0) {
        auto v_pre = vid_map.at(v_from->pre_vid_over_sharp_edge);
        v_pre->next_vid_over_sharp_edge = v_to->id;
        v_to->pre_vid_over_sharp_edge = v_pre->id;
    }
    if (v_from->next_vid_over_sharp_edge >= 0) {
        auto v_next = vid_map.at(v_from->next_vid_over_sharp_edge);
        v_next->pre_vid_over_sharp_edge = v_to->id;
        v_to->next_vid_over_sharp_edge = v_next->id;
    }
    if (v_from->is_sharp_vertex)
        v_to->is_sharp_vertex = true;

    quads.erase(quad);
    if (vid_map.erase(v_from->id) == 0)
        throw;
    vertices.erase(v_from);

    for (const auto v : {v_to, v_left, v_right}) {
        v->on_boundary_or_sharp_edge = is_boundary_or_on_sharp_edge(v);
        v->on_boundary = is_boundary(v);
        if (v->on_boundary_or_sharp_edge)
            rotate_vertex_quad_to_boundary(v);
    }
    for (const auto v : {v_to, v_left, v_right}) {
        update_vertex_valence(v);
    }
    return true;
}

Quad2d3d::Vertex_handle Quad2d3d::split_edge(Split split, bool is_left) {
    quads.push_back(Quad{});
    auto qit = --quads.end();
    vertices.push_back(Vertex{});
    auto new_vit = --vertices.end();

    if (is_left) {
        std::swap(split.vertex_back_id, split.vertex_heading_id);
    }

    const V vertex_heading = vid_map.at(split.vertex_heading_id);
    const V vertex = vid_map.at(split.vertex_id);
    const V vertex_back = vid_map.at(split.vertex_back_id);

    const Vector_3 normal = calc_vertex_normal(vertex);

    qit->corners[0] = new_vit;
    qit->corners[1] = vertex_heading;
    qit->corners[2] = vertex;
    qit->corners[3] = vertex_back;

    for (int i = 0; i < 4; i++)
        qit->sharp_edge_neighbors.at(i) = quads.end();

    qit->neighbors[0] = get_quad_from_edge_ccw(vertex_heading, vertex);
    qit->neighbors[1] = get_quad_from_edge_ccw(vertex, vertex_heading);
    qit->neighbors[2] = get_quad_from_edge_ccw(vertex_back, vertex);
    qit->neighbors[3] = get_quad_from_edge_ccw(vertex, vertex_back);

    for (const auto &pair :
         std::vector<std::pair<int, Vertex_handle>>{{0, vertex_heading}, {1, vertex}, {2, vertex_back}, {3, vertex}}) {
        const auto q = qit->neighbors.at(pair.first);
        if (q == quads.end())
            continue;
        const auto v = pair.second;
        const int k_mirror = q->index(v);
        q->neighbors.at(k_mirror) = qit;
        const auto q_sharp = q->sharp_edge_neighbors.at(k_mirror);
        if (q_sharp != quads.end()) {
            q->sharp_edge_neighbors.at(k_mirror) = quads.end();
            int k_oppo;
            switch (pair.first) {
            case 0:
                k_oppo = 1;
                break;
            case 1:
                k_oppo = 0;
                break;
            case 2:
                k_oppo = 3;
                break;
            case 3:
                k_oppo = 2;
                break;
            }
            qit->sharp_edge_neighbors.at(k_oppo) = q_sharp;

            bool found = false;
            int k_pivot = (k_oppo + 1) % 4;
            if (k_pivot == 0)
                k_pivot = 2;
            const V v_pivot = qit->corners[k_pivot];
            for (int k = 0; k < 4; k++) {
                const auto v = q_sharp->corners[k];
                if (v->next_vid_over_sharp_edge == v_pivot->id || v->pre_vid_over_sharp_edge == v_pivot->id) {
                    found = true;
                    q_sharp->sharp_edge_neighbors.at(k) = qit;
                    break;
                }
            }
            if (!found)
                throw;
        }
    }

    {
        std::unordered_set<Quad_handle, Quad_handle_hash> qs;
        const auto q0 = qit->neighbors[0];
        const auto q3 = qit->neighbors[3];
        auto q0_run = q0;
        while (q0_run != quads.end() && q0_run != q3) {
            qs.insert(q0_run);
            q0_run = rotate_quad_cw(q0_run, vertex);
        }
        auto q3_run = q3;
        while (q3_run != quads.end() && q3_run != q0) {
            qs.insert(q3_run);
            q3_run = rotate_quad_ccw(q3_run, vertex);
        }
        if (q0 != quads.end())
            qs.insert(q0);
        for (auto &q : qs)
            q->corners.at(q->index(vertex)) = new_vit;
    }

    new_vit->quad = qit;
    vertex->quad = qit;

    for (const auto v : {new_vit, vertex, vertex_back, vertex_heading}) {
        v->on_boundary_or_sharp_edge = is_boundary_or_on_sharp_edge(v);
        v->on_boundary = is_boundary(v);
        if (v->on_boundary_or_sharp_edge)
            rotate_vertex_quad_to_boundary(v);
    }

    {
        Point_3 p1, p2;
        {
            Point_3 p = vertex->point;
            Point_3 nb1 = vertex_heading->point;
            Point_3 nb2 = vertex_back->point;
            Vector_3 pi = nb1 - p;
            Vector_3 pj = nb2 - p;
            Vector_3 vi = CGAL::cross_product(normal, pi);
            Vector_3 vj = CGAL::cross_product(normal, pj);
            const Vector_3 vec = (vi - vj) / 2. / 4.;
            p1 = p + vec;
            p2 = p - vec;
            p1 += oracle->vector_to_nearest_point_on_boundary(p1);
            p2 += oracle->vector_to_nearest_point_on_boundary(p2);
        }

        new_vit->point = p2;
        vertex->point = p1;
    }

    if (split.vid_to_create >= 0) {
        new_vit->id = split.vid_to_create;
        vertex->point = split.point_original;
        new_vit->point = split.point_create;
    } else {
        new_vit->id = Vertex::id_counter++;
    }

    if (new_vit->on_boundary_or_sharp_edge) {
        if (vertex->pre_vid_over_sharp_edge == vertex->next_vid_over_sharp_edge &&
            vertex->next_vid_over_sharp_edge == vertex->id) {
            // the vertex is a sharp vertex on the edge of a sharp edge
            new_vit->pre_vid_over_sharp_edge = new_vit->next_vid_over_sharp_edge = new_vit->id;
            vertex->pre_vid_over_sharp_edge = vertex->next_vid_over_sharp_edge = -1;
        } else {
            if (vertex->pre_vid_over_sharp_edge >= 0) {
                const auto pre_v = vid_map.at(vertex->pre_vid_over_sharp_edge);
                if (pre_v->next_vid_over_sharp_edge != vertex->id)
                    throw;
                pre_v->next_vid_over_sharp_edge = new_vit->id;
                new_vit->pre_vid_over_sharp_edge = pre_v->id;
                vertex->pre_vid_over_sharp_edge = -1;
            }
            if (vertex->next_vid_over_sharp_edge >= 0) {
                const auto next_v = vid_map.at(vertex->next_vid_over_sharp_edge);
                if (next_v->pre_vid_over_sharp_edge != vertex->id)
                    throw;
                next_v->pre_vid_over_sharp_edge = new_vit->id;
                new_vit->next_vid_over_sharp_edge = next_v->id;
                vertex->next_vid_over_sharp_edge = -1;
            }
        }

        if (vertex->is_sharp_vertex) {
            new_vit->is_sharp_vertex = true;
            vertex->is_sharp_vertex = false;
        }
    }

    for (const auto v : {new_vit, vertex, vertex_back, vertex_heading}) {
        v->on_boundary = is_boundary(v);
        update_vertex_valence(v);
    }

    if (split.qid_to_create >= 0) {
        qit->id = split.qid_to_create;
    } else {
        qit->id = Quad::id_counter++;
    }

    qit->is_new = true;
    vid_map[new_vit->id] = new_vit;
    return new_vit;
}

void Quad2d3d::break_high_valence() {
    int count = 0;
    std::set<int> taken_vid;
    std::vector<V> vs;
    for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
        if (effective_valence(vit) < 6)
            continue;
        if (vit->on_boundary_or_sharp_edge)
            continue;
        if (taken_vid.find(vit->id) != taken_vid.end())
            continue;
        const auto nbs = collect_quad_one_ring_vertices_in_order(vit);
        bool taken = false;
        for (const auto &nbv : nbs) {
            if (taken_vid.find(nbv->id) != taken_vid.end()) {
                taken = true;
                break;
            }
        }
        if (!taken) {
            vs.push_back(vit);
        }
    }
    for (const auto &vit : vs) {
        const auto nbs = collect_quad_one_ring_vertices_in_order(vit);

        // std::cout << green << "consider break vertex " << vit->id << white << std::endl;
        int best_cost = 100000;
        std::pair<int, int> best_pair;
        for (int i = 0; i < (nbs.size() + 1) / 2; i++) {
            // std::cout << "split i = " << i << std::endl;
            const int valence_i = effective_valence(nbs[i]);
            for (int j = i + nbs.size() / 2 - 1; j <= i + nbs.size() / 2 + 1; j++) {
                const int valence_j = effective_valence(nbs[j % nbs.size()]);
                const int valence_left = j - i + 1;
                const int valence_right = nbs.size() - valence_left + 2;
                auto cal_val = [](int valence) { return (valence - 4) * (valence - 4); };
                const int cost =
                    cal_val(valence_i + 1) + cal_val(valence_j + 1) + cal_val(valence_left) + cal_val(valence_right);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_pair = {i, int(j % nbs.size())};
                }
            }
        }
        // std::cout << "consider i,j= " << best_pair.first << best_pair.second << " cost=" << best_cost <<
        // std::endl;
        Split s;
        s.vertex_id = vit->id;
        s.vertex_back_id = nbs[best_pair.first]->id;
        s.vertex_heading_id = nbs[best_pair.second]->id;
        split_edge(s, true);
        count++;
    }
    std::cout << green << "broke " << count << " high valence vertices " << white << std::endl;
}

void Quad2d3d::collapse_low_valence() {
    int count = 0;
    while (true) {
        auto vit = vertices.begin();
        for (; vit != vertices.end(); ++vit) {
            if (is_boundary_or_on_sharp_edge(vit))
                continue;
            if (vit->valence != 2)
                continue;
            auto q = vit->quad;
            const int k = q->index(vit);
            const int k_oppo = (k + 2) % 4;
            Collapse c;
            c.quad = q;
            c.k_from = k;
            c.k_to = k_oppo;
            c.has_point_original = true;
            c.point_original = q->corners.at(k_oppo)->point;
            if (collapse_quad(c)) {
                count++;
                break;
            }
        }
        if (vit == vertices.end()) {
            std::cout << green << "broke " << count << " low valence vertices " << white << std::endl;
            return;
        }
    }
}

void Quad2d3d::collapse_thin_quad() {
    while (true) {
        auto qit = quads.begin();
        for (; qit != quads.end(); ++qit) {
            const Point_3 p0 = qit->corners[0]->point;
            const Point_3 p1 = qit->corners[1]->point;
            const Point_3 p2 = qit->corners[2]->point;
            const Point_3 p3 = qit->corners[3]->point;

            const double d02 = CGAL::squared_distance(p0, p2);
            const double d13 = CGAL::squared_distance(p1, p3);

            if (d02 < 0.05 * d13) {
                std::cout << "collapse quad " << qit->id << std::endl;
                Collapse c;
                c.quad = qit;
                c.k_from = 0;
                c.k_to = 2;
                c.has_point_original = false;
                if (collapse_quad(c))
                    break;
            } else if (d13 < 0.05 * d02) {
                std::cout << "collapse quad " << qit->id << std::endl;
                Collapse c;
                c.quad = qit;
                c.k_from = 0;
                c.k_to = 2;
                c.has_point_original = false;
                if (collapse_quad(c))
                    break;
            }
        }
        if (qit == quads.end()) {
            break;
        }
    }
}

void Quad2d3d::find_quad(int id) { std::cout << "not implemented" << std::endl; }

void Quad2d3d::find_vertex(int id) {
    if (vid_map.count(id) == 0)
        std::cout << "not found" << std::endl;
    auto v = vid_map.at(id);
    std::cout << "vertex at (" << v->point.x() << ", " << v->point.y() << ")" << std::endl;
}

void Quad2d3d::select_vertex_for_collapse(const Point_3 &p) {
    V v;
    if (!locate_vertex(p, v))
        return;

    selected_vertices.push_back(v);
    if (selected_vertices.size() != 2)
        return;

    const auto v0 = selected_vertices[0];
    const auto v1 = selected_vertices[1];
    selected_vertices.clear();
    bool topo_ok;
    Collapse c = find_quad_from_diag(v0, v1, topo_ok);
    if (!topo_ok) {
        std::cout << "invalid topology" << std::endl;
        return;
    }
    if (collapse_quad(c)) {
        std::cout << "cannot collapse" << std::endl;
    }
}

void Quad2d3d::select_vertex_for_split(const Point_3 &p) {
    V v;
    if (!locate_vertex(p, v))
        return;

    selected_vertices.push_back(v);
    if (selected_vertices.size() != 3)
        return;

    auto v0 = selected_vertices[0];
    auto v1 = selected_vertices[1];
    auto v2 = selected_vertices[2];
    selected_vertices.clear();
    Split s;
    s.vertex_id = v1->id;
    s.vertex_heading_id = v0->id;
    s.vertex_back_id = v2->id;
    split_edge(s, true);
}

void Quad2d3d::update_vertex_valence(const Vertex_handle &v) {
    v->valence = 0;
    auto qit = v->quad;
    do {
        v->valence++;
        qit = rotate_quad_ccw(qit, v);
    } while (qit != v->quad && qit != quads.end());
}

std::vector<Quad2d3d::Vertex_handle> Quad2d3d::collect_quad_one_ring_vertices_in_order(Vertex_handle_const v) const {
    std::vector<V> nb_vs;
    nb_vs.reserve(5);
    auto q = v->quad;
    do {
        nb_vs.push_back(q->corners[(q->index(v) + 1) % 4]);
        auto q_next = rotate_quad_ccw(q, v);
        if (q_next == quads.end()) {
            nb_vs.push_back(q->corners[(q->index(v) + 3) % 4]);
            break;
        }
        q = q_next;
    } while (q != v->quad);
    return nb_vs;
}

Vector_3 Quad2d3d::calc_vertex_normal(const V &v) const {
    Vector_3 n = CGAL::NULL_VECTOR;
    auto q = v->quad;
    do {
        n += q->calc_normal();
        q = rotate_quad_ccw(q, v);
    } while (q != v->quad && q != quads.end());
    return safe_normalize(n);
}

int Quad2d3d::calc_vertex_gaussian_curvature_valence(const Vertex_handle_const &v) const {
    auto q = v->quad;
    std::vector<double> angles;
    double angle_sum = 0.;
    do {
        const double angle = q->angle(q->index(v));
        angles.push_back(angle);
        q = rotate_quad_ccw(q, v);
    } while (q != v->quad && q != quads.end());
    std::sort(begin(angles), end(angles));
    for (double a : angles)
        angle_sum += a;
    int multiples_90_angle = (int)std::round(angle_sum / (M_PI / 2.0));
    multiples_90_angle = std::max(multiples_90_angle, 1);
    return multiples_90_angle;
}

Quad2d3d::Collapse Quad2d3d::find_quad_from_diag(Vertex_handle v0, Vertex_handle v1, bool &topo_ok) const {
    topo_ok = true;
    if (v0->on_boundary_or_sharp_edge && v1->on_boundary_or_sharp_edge)
        topo_ok = false;

    auto q = v0->quad;
    int found = 0;
    Quad_handle q_found;
    do {
        int k = q->index(v0);
        if (q->corners[(k + 2) % 4] == v1) {
            found++;
            q_found = q;
        }
        q = rotate_quad_ccw(q, v0);
    } while (q != v0->quad && q != quads.end());

    if (found == 0) {
        topo_ok = false;
        return {q_found, -1, -1};
    }
    if (found > 1) {
        topo_ok = false;
    }
    return {q_found, q_found->index(v0), q_found->index(v1)};
}

std::vector<Quad2d3d::Path> Quad2d3d::path_to(const std::unordered_set<Vertex_handle, Vertex_handle_hash> &roots,
                                              const std::function<bool(const Vertex_handle &)> &is_target,
                                              const std::function<bool(const Vertex_handle &)> &vertex_ok,
                                              const int max_number_of_path, const bool randomize_path) const {
    Vertex::flag_counter++;
    std::unordered_map<int, V> parent_vertices;
    std::queue<V> bfs_queue;

    const auto push = [&bfs_queue, this](const V &v) {
        // if (v->temp_flag == Vertex::flag_counter) {
        //    throw;
        //}
        bfs_queue.push(v);
        v->temp_flag = Vertex::flag_counter;

        // to mark vertices sharing the same sharp feature as visited to prevent bugs
        bool loop = false;
        auto cur_v = v;
        while (true) {
            if (cur_v->next_vid_over_sharp_edge > 0) {
                cur_v = vid_map.at(cur_v->next_vid_over_sharp_edge);
                cur_v->temp_flag = Vertex::flag_counter;
                if (cur_v == v) {
                    loop = true;
                    break;
                }
            } else {
                break;
            }
        }
        if (!loop) {
            cur_v = v;
            while (true) {
                if (cur_v->pre_vid_over_sharp_edge > 0) {
                    cur_v = vid_map.at(cur_v->pre_vid_over_sharp_edge);
                    cur_v->temp_flag = Vertex::flag_counter;
                    if (cur_v == v) {
                        throw;
                    }
                } else {
                    break;
                }
            }
        }
    };
    std::default_random_engine e(-1);
    const auto pop = [this, &bfs_queue, &push, &parent_vertices, &vertex_ok, &roots, randomize_path, &e]() {
        if (randomize_path) {
            const int rotation = std::uniform_int_distribution<int>(0, int(bfs_queue.size()) - 1)(e);
            for (int i = 0; i < rotation; i++) {
                bfs_queue.push(bfs_queue.front());
                bfs_queue.pop();
            }
        }
        const V v = bfs_queue.front();
        bfs_queue.pop();

        if (roots.count(v) == 0 && !vertex_ok(v))
            return v;

        std::vector<V> interior_vs;
        std::vector<V> border_vs;
        std::vector<V> nbs = collect_quad_one_ring_vertices_in_order(v);
        std::sort(begin(nbs), end(nbs), [](const auto &a, const auto &b) { return a->id < b->id; });
        for (const auto &next_v : nbs) {
            // In this case, other vertex has been visited already.
            if (next_v->temp_flag == Vertex::flag_counter)
                continue;
            if (!next_v->on_boundary_or_sharp_edge)
                interior_vs.push_back(next_v);
            else
                border_vs.push_back(next_v);
            parent_vertices.emplace(next_v->id, v);
        }
        // push interior first, then border, we prefer interior v
        for (auto vv : interior_vs)
            push(vv);
        for (auto vv : border_vs)
            push(vv);

        return v;
    };

    std::vector<Path> ps;

    for (const V &root : roots)
        push(root);

    while (!bfs_queue.empty()) {
        V cur_v = pop();

        if (is_target(cur_v)) {
            ps.emplace_back();
            auto &p = ps.back();
            p.push_back(cur_v);

            V v_path = cur_v;
            while (roots.count(v_path) == 0) {
                v_path = parent_vertices.at(v_path->id);
                p.push_back(v_path);
            }
            std::reverse(p.begin(), p.end());
            if (ps.size() >= max_number_of_path)
                break;
        }
    }
    return ps;
}

Quad2d3d::Path Quad2d3d::build_path_from_vid_list(const Vertex_handle &start, const std::vector<int> &vid_list) const {
    Path p;
    p.push_back(start);
    if (start->id != vid_list.front())
        throw;
    for (int i = 1; i < vid_list.size(); i++) {
        const int vid = vid_list[i];
        bool found = false;
        for (const auto &nb : collect_quad_one_ring_vertices_in_order(p.back())) {
            if (nb->id == vid) {
                p.push_back(nb);
                found = true;
                break;
            }
        }
        if (!found)
            throw;
    }
    return p;
}

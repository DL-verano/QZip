#include "quad.h"
#include "tools/kd_tree_3d.h"
#include <set>

typedef Quad2d3d::Vertex_handle V;

void Quad2d3d::laplacian_smooth_iter(bool only_calculate) {
    struct VertexMovement {
        double count = 0;
        Vector_3 sum = CGAL::NULL_VECTOR;
        int oracle_count = 0;
        Vector_3 oracle_pulling = CGAL::NULL_VECTOR;
        Vector_3 normal = CGAL::NULL_VECTOR;
        double average_edge_length;
    };

    std::unordered_map<Vertex_handle, VertexMovement, Vertex_handle_hash> v_moves;
    for (auto v = vertices.begin(); v != vertices.end(); ++v) {
        if (v->is_frozen)
            continue;
        auto &v_move = v_moves[v];

        const Vector_3 normal = calc_vertex_normal(v);
        v_move.normal = normal;

        auto project_vec_to_normal_plane = [&normal](const Vector_3 &v) -> Vector_3 {
            return v - (v * normal) * normal;
        };

        std::vector<Quad_handle> one_ring_quads;
        {
            auto qit = v->quad;
            do {
                one_ring_quads.push_back(qit);
                qit = rotate_quad_ccw(qit, v);
            } while (qit != v->quad && qit != quads.end());
        }

        std::vector<double> angles;
        std::vector<Vector_3> vecs;
        std::vector<V> nb_vs;
        int i_longest = 0;
        double sq_length_longest = 0.0;
        for (int i = 0; i < one_ring_quads.size(); i++) {
            const auto qit = one_ring_quads[i];
            const int k = qit->index(v);

            const int a = (k + 1) % 4;
            const int b = (k + 2) % 4;
            const int c = (k + 3) % 4;

            Vector_3 va = qit->corners[a]->point - v->point;
            Vector_3 vc = qit->corners[c]->point - v->point;
            const double angle =
                std::acos((va / std::sqrt(va.squared_length())) * (vc / std::sqrt(vc.squared_length())));

            Vector_3 n = CGAL::cross_product(va, vc);

            vecs.push_back(va);
            angles.push_back(angle);
            nb_vs.push_back(qit->corners[a]);
            const double sq_length = va.squared_length();
            const bool orientation_ok = n * normal > 0.;
            if (orientation_ok && sq_length > sq_length_longest) {
                sq_length_longest = sq_length;
                i_longest = i;
            }
        }
        double sum_angle = 0.;
        for (int i = 0; i < one_ring_quads.size(); i++)
            sum_angle += angles[i];
        for (int i = 0; i < one_ring_quads.size(); i++)
            angles[i] *= 2 * 3.1415926 / sum_angle;
        auto rotate_vec_by_angle = [&normal](const Vector_3 &v, double angle) -> Vector_3 {
            Vector_3 y = CGAL::cross_product(normal, v);
            y /= std::sqrt(y.squared_length());
            Vector_3 x = v / std::sqrt(v.squared_length());
            Vector_3 dir = x * std::cos(angle) + y * std::sin(angle);
            return dir;
        };
        double rotation_angle = 0.;
        Vector_3 first_vec = project_vec_to_normal_plane(vecs[i_longest]);
        double length = std::sqrt(first_vec.squared_length());
        if (only_calculate)
            rendering_vectors.emplace_back(RenderVec{v->point, v->point + normal * length * 0.25, 4});
        first_vec /= length;
        Vector_3 dir_sum = CGAL::NULL_VECTOR;
        for (int i = 0; i < one_ring_quads.size(); i++) {
            const int k = (i + i_longest) % one_ring_quads.size();
            const Vector_3 dir =
                use_polar_laplacian && !v->on_boundary_or_sharp_edge
                    ? rotate_vec_by_angle(first_vec, rotation_angle) * std::sqrt(vecs[k].squared_length())
                    : vecs[k];
            dir_sum += dir;
            if (only_calculate)
                rendering_vectors.emplace_back(RenderVec{v->point, v->point + dir / 3., 1});
            rotation_angle += angles[k];
            const Vector_3 target_point = v->point + dir - CGAL::ORIGIN;
            const V nb = nb_vs[k];
            const int nbv = nb->on_boundary_or_sharp_edge ? effective_valence(nb) : nb->valence;
            const int vv = v->on_boundary_or_sharp_edge ? effective_valence(v) : v->valence;
            const int num_high_valence = (nbv > 4 ? 1 : 0) + (vv > 4 ? 1 : 0);
            const int num_low_valence = (nbv < 4 ? 1 : 0) + (vv < 4 ? 1 : 0);
            const int valence_count = num_high_valence - num_low_valence;
            double weight;
            if (valence_count == 2)
                weight = 2.0;
            else if (valence_count == -2)
                weight = 0.5;
            else if (valence_count == 1)
                weight = 2.0;
            else if (valence_count == -1)
                weight = 0.5;
            else if (valence_count == 0)
                weight = 1.0;
            else
                throw;
            v_move.sum += target_point * weight;
            v_move.count += weight;
        }

        v_move.sum /= v_move.count;

        if (only_calculate)
            rendering_vectors.emplace_back(RenderVec{v->point, v->point + dir_sum / 3., 4});
    }
    if (only_calculate)
        return;

    if (smooth_oracle_pulls) {
        KdTree3D tree;
        std::vector<Point_3> all_pts;
        std::vector<Vertex_handle> vhs;
        for (auto v = vertices.begin(); v != vertices.end(); ++v) {
            all_pts.push_back(v->point);
            vhs.push_back(v);
        }
        tree.set_points(all_pts);
        const std::vector<std::pair<Kernel::Point_3, Kernel::Vector_3>> &sample_normals =
            oracle->surface_sample_and_normals();

        std::vector<int> idxes(sample_normals.size());
#pragma omp parallel for
        for (int a = 0; a < sample_normals.size(); a++) {
            const Point_3 &s = sample_normals[a].first;
            const Vector_3 &normal = sample_normals[a].second;
            const int index = tree.query(s);
            const auto &v = vhs[index];
            const auto &v_move = v_moves.at(v);
            if (v_move.normal * normal > 0.707 /*45 degrees*/) {
                idxes[a] = index;
            } else {
                idxes[a] = -1;
            }
        }

        for (int i = 0; i < sample_normals.size(); i++) {
            const int idx = idxes[i];
            if (idx < 0)
                continue;
            const auto &s = sample_normals[i].first;
            auto v = vhs[idx];
            auto &v_move = v_moves.at(v);
            v_move.oracle_pulling += s - v->point;
            v_move.oracle_count++;
        }
    }

    for (const auto &pair : v_moves) {
        const VertexMovement &move = pair.second;
        const Vector_3 normal_pulling =
            move.oracle_count > 0 ? (move.oracle_pulling / move.oracle_count) * 0.8 : CGAL::NULL_VECTOR;
        const Vector_3 laplacian = (CGAL::ORIGIN + pair.second.sum - pair.first->point) * laplacian_step;
        pair.first->point += laplacian + normal_pulling;
    }

    {
        // stick shared sharp feature vertices together
        std::set<int> done_vids;
        for (auto v = vertices.begin(); v != vertices.end(); ++v) {
            if (!v->on_boundary_or_sharp_edge)
                continue;
            if (done_vids.find(v->id) != done_vids.end())
                continue;
            // adjacent to sharp edge
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
            std::vector<V> v_ring;
            while (cur_v->next_vid_over_sharp_edge >= 0) {
                v_ring.push_back(cur_v);
                auto ok = done_vids.insert(cur_v->id);
                if (!ok.second)
                    throw;
                cur_v = vid_map.at(cur_v->next_vid_over_sharp_edge);
                if (cur_v == v)
                    break;
            }
            Vector_3 sum = CGAL::NULL_VECTOR;
            Point_3 frozen_p;
            bool is_frozen = false;
            int count = 0;
            for (const auto &vi : v_ring) {
                sum += vi->point - CGAL::ORIGIN;
                count++;
                if (vi->is_frozen) {
                    frozen_p = vi->point;
                    is_frozen = true;
                    break;
                }
            }
            sum /= count;
            if (is_frozen) {
                for (const auto &vi : v_ring) {
                    vi->point = frozen_p;
                    vi->is_frozen = true;
                }
            } else {
                for (const auto &vi : v_ring) {
                    vi->point = CGAL::ORIGIN + sum;
                    if (std::isnan(sum.x()) || std::isnan(sum.y()) || std::isnan(sum.z()))
                        throw;
                }
            }
        }
    }

    if (smooth_project_to_surface) {
        for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
            auto &v_move = v_moves.at(vit);

            const auto q = vit->quad;
            double length = 0.;
            const auto p0 = q->corners[0]->point;
            const auto p1 = q->corners[1]->point;
            const auto p2 = q->corners[2]->point;
            const auto p3 = q->corners[3]->point;
            double l0 = CGAL::squared_distance(p0, p1);
            double l1 = CGAL::squared_distance(p1, p2);
            double l2 = CGAL::squared_distance(p2, p3);
            double l3 = CGAL::squared_distance(p3, p0);
            length += std::sqrt(l0);
            length += std::sqrt(l1);
            length += std::sqrt(l2);
            length += std::sqrt(l3);
            length /= 4.;
            length *= 1.0;
            if (std::isnan(length))
                throw;
            v_move.average_edge_length = length;
        }
    }

    for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
        if (vit->is_frozen)
            continue;
        if (vit->is_sharp_vertex && oracle->has_sharp_vertices() && smooth_project_to_feature) {
            vit->point += oracle->vector_to_nearest_point_on_sharp_vertices(vit->point);
        } else if (vit->on_boundary && smooth_project_to_feature) {
            vit->point += oracle->vector_to_nearest_point_on_border_edges(vit->point);
        } else if (vit->on_boundary_or_sharp_edge && oracle->has_sharp_edges() && smooth_project_to_feature) {
            vit->point += oracle->vector_to_nearest_point_on_sharp_edges(vit->point);
        } else if (smooth_project_to_surface) {
            const auto &v_move = v_moves.at(vit);
            vit->point += oracle->oriented_segment_intersection(vit->point, v_move.normal * v_move.average_edge_length);
        }
    }
}

void Quad2d3d::laplacian_smooth() {
    // laplacian_iters == 0 means only calculate, not execute
    const bool only_calculate = laplacian_iters == 0;
    const int iters = only_calculate ? 1 : laplacian_iters;
    if (only_calculate && !rendering_vectors.empty()) {
        rendering_vectors.clear();
        return;
    }
    rendering_vectors.clear();

    for (int it = 0; it < iters; it++) {
        laplacian_smooth_iter(only_calculate);
    }

    update_all_quad_stretch();
    update_all_quad_size_ratio();

}

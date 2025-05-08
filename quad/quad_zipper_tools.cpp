#include "../include/console_color.h"
#include "../tools/kd_tree_3d.h"
#include "quad.h"
#include "tools/scope_timer.h"
#include <map>
#include <random>
#include <set>
#include <thread>

//#pragma optimize("", off)

typedef Quad2d3d::Vertex_handle V;

namespace {
int find_index(const std::vector<V> &vs, const V &v) {
    const auto it = std::find(begin(vs), end(vs), v);
    if (it == end(vs))
        throw;
    return it - begin(vs);
}

int modulo(int a, int period) { return (a + period * 10) % period; }

int modulo_add(int value, int add, int period) { return modulo(value + add, period); }

int normal_add(int value, int add, int period) { return value + add; }

int modulo_equal(int a, int b, int period) { return modulo(a, period) == modulo(b, period); }

int normal_equal(int a, int b, int period) { return a == b; }

auto get_add_func(const V &v) {
    return std::bind(!v->on_boundary_or_sharp_edge ? modulo_add : normal_add, std::placeholders::_1,
                     std::placeholders::_2, v->valence);
}

auto get_equal_func(const V &v) {
    return std::bind(!v->on_boundary_or_sharp_edge ? modulo_equal : normal_equal, std::placeholders::_1,
                     std::placeholders::_2, v->valence);
}

int rotate_k_around_vertex(const V &v, int k, int rotation_ccw) {
    const int rotated_k = get_add_func(v)(k, rotation_ccw);
    if (rotated_k < 0 || v->on_boundary_or_sharp_edge && rotated_k > v->valence) {
        return -1;
    }
    return rotated_k;
}

int determine_rotation_of_node_on_path(int last_dir, int k_next, int k_pre, const V &v, const Quad2d3d *quad2d3d,
                                       bool &ok) {
    if (k_pre == k_next) { // go backwards, folded, we do not allow this at this stage
        throw;
    }
    int rotation;
    ok = true;
    if (last_dir == (Quad2d3d::VCompass::COLLAPSE | Quad2d3d::VCompass::LEFT) ||
        last_dir == (Quad2d3d::VCompass::SPLIT | Quad2d3d::VCompass::RIGHT)) {
        /*             ^
         *             |
         *             |
         *             |  \
         *         ----O-- |  count from right side
         *             ^  /
         *             | /
         *             |
         *             |
         */
        if (!v->on_boundary_or_sharp_edge) {
            rotation = modulo_add(k_next, -k_pre, v->valence);
        } else {
            if (k_next < k_pre) {
                // cannot count from this ride since it's a boundary
                ok = false;
                return 0;
            }
            rotation = modulo_add(k_next, -k_pre,
                                  quad2d3d->effective_valence(v) /*using effective valence to treat boundary vertex*/);
        }
    } else {
        /*             ^
         *             |
         *           7 |
         *          /  |
         *         | --O----  count from left side
         *          \  ^
         *           \ |
         *             |
         *             |
         */
        if (!v->on_boundary_or_sharp_edge) {
            rotation = modulo_add(k_pre, -k_next, v->valence);
        } else {
            if (k_next > k_pre) {
                // cannot count from this ride since it's a boundary
                ok = false;
                return 0;
            }
            rotation = modulo_add(k_pre, -k_next,
                                  quad2d3d->effective_valence(v) /*using effective valence to treat boundary vertex*/);
        }
    }
    int turn;
    const int vc_dir = last_dir & (Quad2d3d::VCompass::RIGHT | Quad2d3d::VCompass::LEFT);
    if (last_dir & Quad2d3d::VCompass::COLLAPSE) {
        if (rotation == 1)
            turn = Quad2d3d::VCompass::switch_LR(vc_dir);
        else if (rotation == 2)
            turn = 0;
        else
            turn = vc_dir;
    } else {
        if (rotation == 1)
            turn = vc_dir;
        else if (rotation == 2)
            turn = 0;
        else
            turn = Quad2d3d::VCompass::switch_LR(vc_dir);
    }
    return turn;
}
} // namespace

int Quad2d3d::count_turns_in_path(const Path &path, int starting_dir, bool &ok) const {
    if (path.size() < 2)
        throw;
    int turns = 0;
    int last_dir = starting_dir;
    for (int i = 1; i < path.size() - 1; i++) {
        const V &pre = path[i - 1];
        const V &cur = path[i];
        const V &next = path[i + 1];
        const auto nbs = collect_quad_one_ring_vertices_in_order(cur);
        const int k_pre = find_index(nbs, pre);
        const int k_next = find_index(nbs, next);

        const int local_turn_dir = determine_rotation_of_node_on_path(last_dir, k_next, k_pre, cur, this, ok);
        if (!ok)
            return 0;
        int local_turn;
        if (local_turn_dir == 0)
            local_turn = 0;
        else if (local_turn_dir & VCompass::LEFT) {
            local_turn = +1;
        } else if (local_turn_dir & VCompass::RIGHT) {
            local_turn = -1;
        } else
            throw;
        last_dir = VCompass::RotateCounterClockwise(last_dir, local_turn);
        turns += local_turn;
    }
    return turns;
}

bool Quad2d3d::check_path_self_intersect_in_regular_grid(const Path &path) const {
    if (path.size() < 2)
        throw;
    int dir = 0;
    std::pair<int, int> p{0, 0};
    std::set<std::pair<int, int>> seen;
    seen.insert(p);
    p = {1, 0};
    seen.insert(p);
    const auto move = [&]() {
        switch (dir) {
        case 0:
            p.first++;
            break;
        case 1:
            p.second++;
            break;
        case 2:
            p.first--;
            break;
        case 3:
            p.second--;
            break;
        }
    };
    for (int i = 2; i < path.size() - 2; i++) {
        const V &pre = path[i - 1];
        const V &cur = path[i];
        const V &next = path[i + 1];
        const auto nbs = collect_quad_one_ring_vertices_in_order(cur);
        if (nbs.size() != 4)
            throw;
        const int k_pre = find_index(nbs, pre);
        const int k_next = find_index(nbs, next);
        const int rotate = (k_next + 4 - k_pre) % 4;
        if (rotate == 0)
            throw;
        if (rotate == 1) { // left
            dir = (dir + 1) % 4;
        }
        if (rotate == 2) { // straight
        }
        if (rotate == 3) { // right
            dir = (dir + 3) % 4;
        }
        move();
        const auto it = seen.insert(p);
        if (!it.second)
            return true;
    }
    return false;
}

bool Quad2d3d::adjust_zipper_path_start_end_vertex(Path &path, int starting_dir, bool throw_when_fail) const {
    bool ok;
    const int turn = (count_turns_in_path(path, starting_dir, ok) + 400) % 4;
    if (!ok) {
        if (throw_when_fail)
            throw;
        return false;
    }
    const int tail_dir = VCompass::RotateCounterClockwise(starting_dir, turn);

    const V start = path.front();
    const V tail = path.back();
    if (effective_valence(start) != 3 && effective_valence(start) < 5) {
        std::cout << "Invalid valence for starting vertex" << std::endl;
        if (throw_when_fail)
            throw;
        return false;
    }
    if (effective_valence(tail) != 3 && effective_valence(tail) < 5) {
        std::cout << "Invalid valence for tail vertex" << std::endl;
        if (throw_when_fail)
            throw;
        return false;
    }
    if (effective_valence(start) == 3) {
        if (starting_dir & VCompass::SPLIT) {
            // nothing
        } else {
            const auto nbs = collect_quad_one_ring_vertices_in_order(start);
            const int k = find_index(nbs, path[1]);
            const int side_k = rotate_k_around_vertex(start, k, starting_dir & VCompass::RIGHT ? -1 : +1);
            if (side_k < 0) {
                std::cout << "cannot move there" << std::endl;
                if (throw_when_fail)
                    throw;
                return false;
            }
            const V &side = nbs[side_k];
            const auto nbs_side = collect_quad_one_ring_vertices_in_order(side);
            const int k_side = find_index(nbs_side, start);
            const int k_oppo = rotate_k_around_vertex(side, k_side, starting_dir & VCompass::RIGHT ? -1 : +1);
            const V &oppo = nbs_side.at(k_oppo);
            Path temp;
            temp.push_back(side);
            temp.push_back(oppo);
            auto second = path.begin();
            ++second;
            temp.insert(temp.end(), second, path.end());
            std::swap(temp, path);
        }
    } else {
        if (starting_dir & VCompass::COLLAPSE) {
            // nothing
        } else {
            const auto nbs = collect_quad_one_ring_vertices_in_order(start);
            const int k = find_index(nbs, path[1]);
            const int side_k = rotate_k_around_vertex(start, k, starting_dir & VCompass::RIGHT ? -2 : +2);
            if (side_k < 0) {
                std::cout << "cannot move there" << std::endl;
                if (throw_when_fail)
                    throw;
                return false;
            }
            const V &side = nbs[side_k];
            Path temp;
            temp.push_back(side);
            temp.insert(temp.end(), path.begin(), path.end());
            std::swap(temp, path);
        }
    }
    if (effective_valence(tail) == 3) {
        // nothing
    } else {
        const V v = path[path.size() - 2];
        const auto nbs = collect_quad_one_ring_vertices_in_order(v);
        const auto tail_nbs = collect_quad_one_ring_vertices_in_order(tail);
        const int k = find_index(nbs, tail);
        const int k_tail = find_index(tail_nbs, v);
        if (tail_dir & VCompass::SPLIT) {
            const int kk = rotate_k_around_vertex(tail, k_tail, tail_dir & VCompass::LEFT ? +2 : -2);
            if (kk < 0) {
                std::cout << "cannot move there due to tail constraint" << std::endl;
                if (throw_when_fail)
                    throw;
                return false;
            }
            const V additional_v = tail_nbs[kk];
            path.push_back(additional_v);
        } else {
            const int kk = rotate_k_around_vertex(v, k, tail_dir & VCompass::LEFT ? +1 : -1);
            if (kk < 0) {
                std::cout << "cannot move there due to tail constraint" << std::endl;
                if (throw_when_fail)
                    throw;
                return false;
            }
            const V side = nbs[kk];
            path.pop_back();
            path.push_back(side);
            const auto nbs_side = collect_quad_one_ring_vertices_in_order(side);
            const int k_side = find_index(nbs_side, v);
            const int k2 = rotate_k_around_vertex(side, k_side, tail_dir & VCompass::LEFT ? +1 : -1);
            if (k2 < 0)
                throw;
            path.push_back(nbs_side[k2]);
        }
    }
    return true;
}

static std::vector<int> remove_folded_part(const std::vector<int> &vid_list) {
    auto do_it = [](std::vector<int> &l) -> bool {
        std::vector<int> result;
        bool need_continue = false;
        for (auto item : l) {
            result.push_back(item);
            if (result.size() > 2 && result[result.size() - 3] == result.back()) {
                need_continue = true;
                result.pop_back();
                result.pop_back();
            }
        }
        std::swap(result, l);
        return need_continue;
    };
    std::vector<int> list = vid_list;
    while (do_it(list)) {
    }
    return list;
}

bool Quad2d3d::convert_simple_path_to_zipper_path(const std::vector<int> &path_vid, int start_compass_dir,
                                                  std::vector<VCompass> &compass, double &quad_shape_and_size_score,
                                                  bool adjust_extremity_vertices, bool throw_when_fail) {
    quad_shape_and_size_score = 0.;
    Path path;
    V v_start_adjust_next;
    {
        Path path_to_adjust;
        for (int id : path_vid)
            path_to_adjust.push_back(vid_map.at(id));
        if (adjust_extremity_vertices)
            if (!adjust_zipper_path_start_end_vertex(path_to_adjust, start_compass_dir, throw_when_fail)) {
                if (throw_when_fail)
                    throw;
                return false;
            }
        const V v_start_adjust = path_to_adjust.front();
        const std::vector<int> vid_list_adjust = [&path_to_adjust]() {
            std::vector<int> vs;
            for (int i = 0; i < path_to_adjust.size(); i++)
                vs.push_back(path_to_adjust[i]->id);
            return vs;
        }();

        v_start_adjust_next = [this, &v_start_adjust, &vid_list_adjust] {
            for (const auto &nb : collect_quad_one_ring_vertices_in_order(v_start_adjust)) {
                if (nb->id == vid_list_adjust[1]) {
                    return nb;
                }
            }
            throw;
        }();
        const auto vid_list_unfold = remove_folded_part(vid_list_adjust);
        path = build_path_from_vid_list(v_start_adjust, vid_list_unfold);
        if (path[0] != v_start_adjust)
            throw;
    }

    const std::vector<std::vector<V>> path_nbs = [&path, this]() {
        std::vector<std::vector<V>> nbs;
        for (const auto &v : path)
            nbs.push_back(collect_quad_one_ring_vertices_in_order(v));
        return nbs;
    }();
    compass.clear();
    { // determine compass direction of each node
        int last_dir = start_compass_dir;
        V virtual_last_v;
        if (v_start_adjust_next != path[1]) {
            // the starting vertex is in the folded part, now we assume we just finished the folded part and coming back
            // to this starting vertex
            last_dir = VCompass::switch_CS(start_compass_dir);
            virtual_last_v = v_start_adjust_next;
        }
        for (int i = start_compass_dir & VCompass::COLLAPSE ? 0 : 1; i < path.size() - 1; i++) {
            compass.emplace_back();
            VCompass &vc = compass.back();

            const V &v = path[i];
            const V &v_next = path[i + 1];
            const auto &nbs = path_nbs[i];
            const int k_next = find_index(nbs, v_next);

            int turn = 0;
            if (i != 0 || virtual_last_v != V()) {
                V last_v = i > 0 ? path[i - 1] : virtual_last_v;
                const int k_pre = find_index(nbs, last_v);
                bool ok = true;
                turn = determine_rotation_of_node_on_path(last_dir, k_next, k_pre, v, this, ok);
                if (!ok) {
                    if (debug_quad_zipper)
                        std::cout << "invalid route, cannot turn because blocked by boundary" << std::endl;
                    if (throw_when_fail)
                        throw;
                    return false;
                }
            }

            if (turn == 0) {
                vc.next_dir = last_dir;
                vc.pre_dir = VCompass::switch_CS(last_dir);
            } else {
                if (last_dir & turn) {
                    vc.next_dir = VCompass::switch_CS(VCompass::switch_LR(last_dir));
                    vc.pre_dir = VCompass::switch_LR(last_dir);
                } else {
                    vc.next_dir = VCompass::switch_LR(last_dir);
                    vc.pre_dir = VCompass::switch_CS(last_dir);
                }
                last_dir = vc.next_dir;
            }

            if (use_shape_score) {
                {
                    const Quad_handle q_left = get_quad_from_edge_ccw(v, v_next);
                    if (q_left != quads.end()) {
                        calculate_quad_stretch(q_left);
                        const int k = q_left->index(v);
                        const bool is_collapse = vc.next_dir & VCompass::COLLAPSE;
                        if (q_left->stretch_direction == 1 || q_left->stretch_direction == 2) {
                            const bool parallel = (q_left->stretch_direction == 1 && (k == 0 || k == 2)) ||
                                                  (q_left->stretch_direction == 2 && (k == 1 || k == 3));
                            if (parallel == is_collapse) // good
                                quad_shape_and_size_score += calc_shape_score(q_left->stretch_ratio);
                            else // bad
                                quad_shape_and_size_score -= calc_shape_score(q_left->stretch_ratio);
                        } else if (q_left->stretch_direction == 3 || q_left->stretch_direction == 4) {
                            const bool prefer_RC_or_LS = (q_left->stretch_direction == 3 && (k == 0 || k == 2)) ||
                                                         (q_left->stretch_direction == 4 && (k == 1 || k == 3));
                            const bool RC_or_LS = is_collapse && (vc.next_dir & VCompass::RIGHT) ||
                                                  !is_collapse && (vc.next_dir & VCompass::LEFT);
                            if (prefer_RC_or_LS == RC_or_LS) // good
                                quad_shape_and_size_score += calc_shape_score(q_left->stretch_ratio);
                            else // bad
                                quad_shape_and_size_score -= calc_shape_score(q_left->stretch_ratio);
                        }
                    }
                }
                {
                    const Quad_handle q_right = get_quad_from_edge_ccw(v_next, v);
                    if (q_right != quads.end()) {
                        calculate_quad_stretch(q_right);
                        const int k = q_right->index(v);
                        const bool is_collapse = vc.next_dir & VCompass::COLLAPSE;
                        if (q_right->stretch_direction == 1 || q_right->stretch_direction == 2) {
                            const bool parallel = (q_right->stretch_direction == 1 && (k == 1 || k == 3)) ||
                                                  (q_right->stretch_direction == 2 && (k == 0 || k == 2));
                            if (parallel == is_collapse) // good
                                quad_shape_and_size_score += calc_shape_score(q_right->stretch_ratio);
                            else // bad
                                quad_shape_and_size_score -= calc_shape_score(q_right->stretch_ratio);
                        } else if (q_right->stretch_direction == 3 || q_right->stretch_direction == 4) {
                            const bool prefer_LC = (q_right->stretch_direction == 3 && (k == 0 || k == 2)) ||
                                                   (q_right->stretch_direction == 4 && (k == 1 || k == 3));
                            const bool LC_or_RS = is_collapse && (vc.next_dir & VCompass::LEFT) ||
                                                  !is_collapse && (vc.next_dir & VCompass::RIGHT);
                            if (prefer_LC == LC_or_RS) // good
                                quad_shape_and_size_score += calc_shape_score(q_right->stretch_ratio);
                            else // bad
                                quad_shape_and_size_score -= calc_shape_score(q_right->stretch_ratio);
                        }
                    }
                }
            }
            if (use_size_score) {
                auto apply_size_score = [&](const Quad_handle &q) {
                    if (q == quads.end())
                        return;
                    calculate_quad_size_ratio(q);
                    if (q->problematic_size) {
                        const bool large = q->size_ratio > 1.0;
                        const bool is_collapse = vc.next_dir & VCompass::COLLAPSE;
                        if (large && is_collapse || !large && !is_collapse) { // bad
                            quad_shape_and_size_score -= calc_size_score(q->size_ratio);
                        } else { // good
                            quad_shape_and_size_score += calc_size_score(q->size_ratio);
                        }
                    }
                };
                apply_size_score(get_quad_from_edge_ccw(v, v_next));
                apply_size_score(get_quad_from_edge_ccw(v_next, v));
            }
        }
    }

    V last_v = path[0];
    // determine compass topology of each node
    auto it = compass.begin();
    for (int i = start_compass_dir & VCompass::COLLAPSE ? 0 : 1; i < path.size() - 1; i++, ++it) {
        const V &v = path[i];
        const V &v_next = path[i + 1];
        const auto &nbs = path_nbs[i];
        const int k_next = find_index(nbs, v_next);

        VCompass &vc = *it;
        const int last_dir = it == compass.begin() ? vc.next_dir : (it - 1)->next_dir;

        // straight
        if (vc.next_dir == last_dir) {
            if (last_dir & VCompass::COLLAPSE) {
                const int k_side = rotate_k_around_vertex(v, k_next, last_dir & VCompass::RIGHT ? +1 : -1);
                if (k_side < 0) {
                    if (debug_quad_zipper)
                        std::cout << "invalid route, collapse meet boundary!" << std::endl;
                    if (throw_when_fail)
                        throw;
                    return false;
                }
                bool ok;
                vc.topo = find_quad_from_diag(nbs.at(k_side), v_next, ok);
                if (!ok) {
                    if (debug_quad_zipper)
                        std::cout << "invalid route, valence 2 vertex found during collapse!" << std::endl;
                    if (throw_when_fail)
                        throw;
                    return false;
                }
            } else {
                Split s;
                s.vertex_id = v->id;
                s.vertex_heading_id = v_next->id;
                s.vertex_back_id = last_v->id;
                vc.topo = s;
            }
        } else {
            if (vc.next_dir & VCompass::SPLIT) {
                Split s;
                s.vertex_id = v->id;
                s.vertex_heading_id = v_next->id;
                if (last_dir & VCompass::SPLIT) {
                    s.vertex_back_id = last_v->id;
                } else {
                    const Collapse &collapse = std::get<Collapse>((it - 1)->topo);
                    const int k = (collapse.k_to + (last_dir & VCompass::RIGHT ? +1 : -1) + 4) % 4;
                    s.vertex_back_id = collapse.quad->corners.at(k)->id;
                }
                vc.topo = s;
            } else {
                if (last_dir & VCompass::SPLIT) {
                    bool ok;
                    vc.topo = find_quad_from_diag(last_v, v_next, ok);
                    if (!ok) {
                        if (debug_quad_zipper)
                            std::cout << "invalid route, valence 2 vertex found during collapse!" << std::endl;
                        if (throw_when_fail)
                            throw;
                        return false;
                    }
                } else {
                    const int k_side = rotate_k_around_vertex(v, k_next, vc.next_dir & VCompass::RIGHT ? +1 : -1);
                    if (k_side < 0) {
                        if (debug_quad_zipper)
                            std::cout << "invalid route, collapse meet boundary!" << std::endl;
                        if (throw_when_fail)
                            throw;
                        return false;
                    }
                    bool ok;
                    vc.topo = find_quad_from_diag(nbs.at(k_side), v_next, ok);
                    if (!ok) {
                        if (debug_quad_zipper)
                            std::cout << "invalid route, valence 2 vertex found during collapse!" << std::endl;
                        if (throw_when_fail)
                            throw;
                        return false;
                    }
                }
            }
        }
        last_v = v;
    }
    return true;
}

double Quad2d3d::search_for_determining_potential(const Vertex_handle &v_start, const std::vector<int> &vid_to_ignore,
                                                  bool early_out_for_nearest) const {
    if (debug_quad_zipper >= 2)
        std::cout << "search_for_determining_potential v_start: " << v_start->id << std::endl;
    const int valence_effective = effective_valence(v_start);
    if (valence_effective == 4)
        return 0.f;
    const int max_distance = potential_search_radius;
    const int max_nb = 20;
    Vertex::flag_counter++;
    double potential = 0.f;
    int distance = 0;
    const int max_boundary_punish_distance = 3;
    int opposite_nb_found = 0;
    int same_nb_found = 0;
    bool boundary_found = false;

    const bool valence_3_or_5 = valence_effective < 4;
    std::vector<V> wave_front{v_start};
    v_start->temp_flag = Vertex::flag_counter;
    std::vector<V> new_wave_front;

    while (!wave_front.empty() && distance <= max_distance) {
        double boundary_punish = 0.;
        double appealing_price = 0.;
        double repelling_price = 0.;
        for (const auto &v : wave_front) {
            const int valence_effect = effective_valence(v);
            if (!boundary_found && v->on_boundary_or_sharp_edge &&
                distance <= max_boundary_punish_distance) { // boundary punishment
                boundary_punish = potential_settings.boundary_push_score / (distance + 1);
                boundary_found = true;
                if (debug_quad_zipper >= 2)
                    std::cout << "  found boundary " << v->id << " at distance " << distance
                              << ", punish = " << boundary_punish << std::endl;
            }
            if (v != v_start && valence_effect != 4 &&
                std::find(begin(vid_to_ignore), end(vid_to_ignore), v->id) == end(vid_to_ignore)) {
                const bool diff_type = (valence_effect > 4 && valence_3_or_5 || valence_effect < 4 && !valence_3_or_5);
                if (opposite_nb_found < max_nb && diff_type) {
                    appealing_price += -potential_settings.appealing_score / distance;
                    if (debug_quad_zipper >= 2)
                        std::cout << "  found opposite vertex " << v->id << " eff_val: " << valence_effect
                                  << " at distance " << distance << ", appealing_price = " << appealing_price
                                  << std::endl;
                    opposite_nb_found++;
                }
                if (same_nb_found < max_nb && !diff_type) {
                    repelling_price += potential_settings.repelling_score / distance;
                    if (debug_quad_zipper >= 2)
                        std::cout << "  found similar vertex " << v->id << " eff_val: " << valence_effect
                                  << " at distance " << distance << ", repelling_price = " << repelling_price
                                  << std::endl;
                    same_nb_found++;
                }
            }
            if (v->temp_flag != Vertex::flag_counter)
                throw;
        }
        if (potential_settings.use_boundary_push)
            potential += boundary_punish;
        if (potential_settings.use_diff_type_appealing)
            potential += appealing_price;
        if (potential_settings.use_same_type_repelling)
            potential += repelling_price;

        if (early_out_for_nearest && distance >= max_boundary_punish_distance && opposite_nb_found >= max_nb &&
            same_nb_found >= max_nb) {
            break;
        }
        new_wave_front.clear();
        new_wave_front.reserve(wave_front.size() * 1.2);
        for (const auto &v : wave_front) {
            for (const auto &nb : collect_quad_one_ring_vertices_in_order(v)) {
                if (nb->temp_flag != Vertex::flag_counter /*not visited yet*/) {
                    new_wave_front.push_back(nb);
                    nb->temp_flag = Vertex::flag_counter; // visited
                }
            }
        }
        distance += 1;
        new_wave_front.swap(wave_front);
    }

    if (potential_settings.use_separatrix && !is_boundary_or_on_sharp_edge(v_start)) {
        if (debug_quad_zipper >= 2)
            std::cout << "  Separatrix for v " << v_start->id << std::endl;
        const int max_length = potential_search_radius;
        const auto one_ring = collect_quad_one_ring_vertices_in_order(v_start);
        std::vector<int> distance_separatrix(one_ring.size(), std::numeric_limits<int>::max());
        for (int i = 0; i < one_ring.size(); i++) {
            const V &start = one_ring[i];
            if (debug_quad_zipper >= 2)
                std::cout << "  One ring v " << start->id << std::endl;
            if (is_boundary_or_on_sharp_edge(start))
                continue;
            if (start->valence != 4) {
                distance_separatrix[i] = 0;
                continue;
            } // already on separatrix
            std::vector<V> separatrix;
            separatrix.push_back(v_start);
            separatrix.push_back(start);
            {
                bool found_singularity = false;
                for (int j = 0; j < max_length; j++) {
                    const V &cur = separatrix.back();
                    const V &last = separatrix[separatrix.size() - 2];
                    const auto cur_one_ring = collect_quad_one_ring_vertices_in_order(cur);
                    const int k = find_index(cur_one_ring, last);
                    const int k_next = modulo_add(k, 2, 4);
                    const V next = cur_one_ring[k_next];
                    if (is_boundary_or_on_sharp_edge(next))
                        break;
                    if (next->valence != 4) {
                        found_singularity = true;
                        if (debug_quad_zipper >= 2)
                            std::cout << "  found singularity on separatrix: v " << next->id << std::endl;
                        break;
                    }
                    separatrix.push_back(next);
                }

                if (false) {
                    std::cout << "  v: ";
                    for (const V v : separatrix) {
                        std::cout << v->id << ", ";
                    }
                    std::cout << std::endl;
                }

                if (found_singularity) { // already on separatrix
                    distance_separatrix[i] = 0;
                    break;
                }
            }

            auto offset_and_shorten_regular_path = [this](const std::vector<V> &path, bool to_left) -> std::vector<V> {
                std::vector<V> r;
                for (int i = 1; i < path.size(); i++) {
                    const V &cur = path[i];
                    const V &last = path[i - 1];
                    const auto cur_one_ring = collect_quad_one_ring_vertices_in_order(cur);
                    const int k = find_index(cur_one_ring, last);
                    const int k_side = modulo_add(k, to_left ? 3 : 1, 4);
                    const V side = cur_one_ring[k_side];
                    r.push_back(side);
                }
                return r;
            };
            int left_distance = std::numeric_limits<int>::max();
            int right_distance = std::numeric_limits<int>::max();
            for (bool left_or_right : {true, false}) {
                if (debug_quad_zipper >= 2)
                    std::cout << "  offset to " << (left_or_right ? "left" : "right") << std::endl;
                std::vector<V> offset = separatrix;
                int round = 0;
                while (true) {
                    if (debug_quad_zipper >= 2)
                        std::cout << "  round " << round << std::endl;
                    round++;
                    offset = offset_and_shorten_regular_path(offset, left_or_right);
                    if (false) {
                        std::cout << "  v: ";
                        for (const V o : offset) {
                            std::cout << o->id << ", ";
                        }
                        std::cout << std::endl;
                    }
                    bool found_singularity = false;
                    for (int j = 0; j < offset.size(); j++) {
                        if (is_boundary_or_on_sharp_edge(offset[j])) {
                            if (debug_quad_zipper >= 2)
                                std::cout << "  hit boundary " << offset[j]->id << std::endl;
                            offset.resize(j);
                            break;
                        }
                        if (offset[j]->valence != 4 && j > 0 /*ignore first vertex to avoid double counting*/) {
                            if (left_or_right) {
                                left_distance = round + 1;
                            } else {
                                right_distance = round + 1;
                            }
                            found_singularity = true;

                            if (debug_quad_zipper >= 2)
                                std::cout << "  found singularity " << offset[j]->id << std::endl;
                            break;
                        }
                    }
                    if (found_singularity)
                        break;

                    if (offset.size() < 2)
                        break;
                }
            }
            const int d = std::min(left_distance, right_distance);
            distance_separatrix[i] = d;
        }
        for (int i = 0; i < distance_separatrix.size(); i++) {
            const int d = distance_separatrix[i];
            const double cost =
                d == std::numeric_limits<int>::max() ? 0. : potential_settings.separatrix_score / (d + 1);
            if (debug_quad_zipper >= 2)
                std::cout << "  Direction v " << one_ring[i]->id << ", distance " << d << ", cost " << cost
                          << std::endl;
            potential -= cost;
        }
    }

    return potential;
}

double Quad2d3d::determine_potential_for_compass_path(const std::vector<VCompass> &path) const {
    std::unordered_set<int> vs_id;
    for (const auto &vc : path) {
        if (vc.next_dir & VCompass::COLLAPSE) {
            const auto &quad = std::get<Collapse>(vc.topo).quad;
            for (int i = 0; i < 4; i++)
                vs_id.insert(quad->corners.at(i)->id);
        } else {
            const auto &s = std::get<Split>(vc.topo);
            vs_id.insert(s.vertex_id);
            vs_id.insert(s.vertex_heading_id);
            vs_id.insert(s.vertex_back_id);
        }
    }
    if (debug_quad_zipper)
        std::cout << "    vertices potentials in compass path:" << std::endl;
    double potential = 0;
    std::vector<int> vs_irr_id;
    for (int vid : vs_id)
        if (effective_valence(vid_map.at(vid)) != 4)
            vs_irr_id.push_back(vid);

    std::unordered_set<Quad_handle, Quad_handle_hash> one_ring_quads;

    for (int vid : vs_id) {
        const auto v = vid_map.at(vid);
        const int val = v->valence;
        const int eff_val = effective_valence(v);
        const int curvature_preferred_valence = use_gaussian_curvature_valence || v->on_boundary_or_sharp_edge
                                                    ? calc_vertex_gaussian_curvature_valence(v)
                                                    : 4;

        const int valence_badness = std::abs(curvature_preferred_valence - val);
        double valence_price = 0.;
        if (valence_badness == 1) {
            valence_price = 1e4;
        } else if (valence_badness == 2) {
            valence_price = 1e6;
        } else if (valence_badness >= 3) {
            valence_price = 1e7;
        }

        if (debug_quad_zipper >= 2)
            std::cout << "      preferred / effective valence: " << curvature_preferred_valence << " / " << val
                      << std::endl;
        double potential_price = 0.;
        if (eff_val == 3 || eff_val == 5) {
            potential_price = search_for_determining_potential(v, {}, true);
        }
        const double pot = potential_price + valence_price;

        if (pot != 0. && debug_quad_zipper)
            printf("      summary: vid %7d valence %d pot %.5f = valence price: %.5f + potential price %.5f\n", vid,
                   val, pot, valence_price, potential_price);
        potential += pot;

        auto q = v->quad;
        do {
            one_ring_quads.insert(q);
            q = rotate_quad_ccw(q, v);
        } while (q != quads.end() && q != v->quad);
    }

    int flat_quad = 0;
    for (const auto &q : one_ring_quads) {
        for (int k = 0; k < 4; k++) {
            if (q->neighbors[k] == quads.end() && q->neighbors[(k + 1) % 4] == quads.end() &&
                !q->corners[(k + 1) % 4]->is_sharp_vertex) {
                flat_quad++;
                std::cout << "flat quad " << q->id << std::endl;
                break;
            }
        }
    }
    if (flat_quad > 0) {
        const double cost = flat_quad * 1e8;
        printf("      In total %d flat quad found, extra cost %f\n", flat_quad, cost);

        potential += cost;
    }
    return potential;
}

bool Quad2d3d::execute_compass_path(bool reverse, std::function<void()> callback) {
    std::unordered_map<int, std::set<int>> vid_to_node;

    for (int i = 0; i < compass_path.size(); i++) {
        auto &vc = compass_path[i];
        if (vc.next_dir & VCompass::SPLIT) {
            const auto &s = std::get<Split>(vc.topo);
            vid_to_node[s.vertex_id].insert(i);
            vid_to_node[s.vertex_heading_id].insert(i);
            vid_to_node[s.vertex_back_id].insert(i);
        }
    }

    struct CollapseRecord // a collapse event influence a split item
    {
        int compass_split_idx;
        enum { VERTEX, VERTEX_HEADING, VERTEX_BACK } type;
        int v_from;
        int v_to;
    };

    std::vector<std::vector<CollapseRecord>> compass_collapse_events(compass_path.size());

    auto treat_collapse_event = [&vid_to_node, &compass_collapse_events, this](int current_idx, V v, V tar) {
        if (v == tar)
            return;
        auto it = vid_to_node.find(v->id);
        if (it == vid_to_node.end())
            return;
        std::vector<int> nodes;
        for (int i : it->second) {
            auto &node = compass_path[i];
            if (!(node.next_dir & VCompass::SPLIT))
                continue;
            auto &split = std::get<Split>(node.topo);
            bool influenced = false;
            CollapseRecord record;
            record.compass_split_idx = i;

            if (split.vertex_id == v->id) {
                split.vertex_id = tar->id;
                record.type = CollapseRecord::VERTEX;
                record.v_from = v->id;
                record.v_to = tar->id;
                influenced = true;
            }
            if (split.vertex_heading_id == v->id) {
                split.vertex_heading_id = tar->id;
                record.type = CollapseRecord::VERTEX_HEADING;
                record.v_from = v->id;
                record.v_to = tar->id;
                influenced = true;
            }
            if (split.vertex_back_id == v->id) {
                split.vertex_back_id = tar->id;
                record.type = CollapseRecord::VERTEX_BACK;
                record.v_from = v->id;
                record.v_to = tar->id;
                influenced = true;
            }
            if (influenced) {
                nodes.push_back(i);
                vid_to_node[tar->id].insert(i);
                if (current_idx >= 0)
                    compass_collapse_events[current_idx].push_back(record);
            }
        }
        for (int i : nodes) {
            it->second.erase(i);
        }
    };

    auto treat_split_event = [&vid_to_node, this](V v, V v_side0, V v_side1, V v_new) {
        auto it = vid_to_node.find(v->id);
        if (it == vid_to_node.end())
            return;

        const auto one_ring_v = collect_quad_one_ring_vertices_in_order(v);
        const auto one_ring_v_new = collect_quad_one_ring_vertices_in_order(v_new);

        std::vector<int> nodes;
        for (int i : it->second) {
            auto &node = compass_path[i];
            if (!(node.next_dir & VCompass::SPLIT))
                continue;
            auto &split = std::get<Split>(node.topo);
            if (split.vertex_id == v_side0->id || split.vertex_id == v_side1->id)
                continue;
            if (split.vertex_heading_id == v->id || split.vertex_back_id == v->id) {
                const bool in_v_ring = std::find_if(begin(one_ring_v), end(one_ring_v), [&](const V &vt) {
                                           return vt->id == split.vertex_id;
                                       }) != end(one_ring_v);
                const bool in_v_new_ring = std::find_if(begin(one_ring_v_new), end(one_ring_v_new), [&](const V &vt) {
                                               return vt->id == split.vertex_id;
                                           }) != end(one_ring_v_new);
                if (in_v_new_ring && in_v_ring)
                    throw;
                if (in_v_new_ring) {
                    if (split.vertex_heading_id == v->id)
                        split.vertex_heading_id = v_new->id;
                    if (split.vertex_back_id == v->id)
                        split.vertex_back_id = v_new->id;
                    nodes.push_back(i);
                    vid_to_node[v_new->id].insert(i);
                }
            }
        }
        for (int i : nodes) {
            it->second.erase(i);
        }
    };

    auto remove_node_from_map = [&vid_to_node](int node_i, int vid) {
        auto it = vid_to_node.find(vid);
        if (it == vid_to_node.end())
            throw;
        if (it->second.erase(node_i) != 1)
            throw;
    };

    auto undo_collapse = [&](int idx) {
        for (const auto &collapse : compass_collapse_events[idx]) {
            Split &s = std::get<Split>(compass_path.at(collapse.compass_split_idx).topo);
            int *vertex_id_ptr = nullptr;
            switch (collapse.type) {
            case CollapseRecord::VERTEX:
                vertex_id_ptr = &s.vertex_id;
                break;
            case CollapseRecord::VERTEX_HEADING:
                vertex_id_ptr = &s.vertex_heading_id;
                if (s.vertex_heading_id != collapse.v_to)
                    throw;
                break;
            case CollapseRecord::VERTEX_BACK:
                vertex_id_ptr = &s.vertex_back_id;
                break;
            }
            if (*vertex_id_ptr != collapse.v_to)
                throw;
            remove_node_from_map(collapse.compass_split_idx, *vertex_id_ptr);
            *vertex_id_ptr = collapse.v_from;
            vid_to_node[*vertex_id_ptr].insert(collapse.compass_split_idx);
        }
    };

    bool succeed = true;

    std::vector<int> order(compass_path.size());
    std::iota(begin(order), end(order), 0);

    for (int itr = 0; itr < 2; itr++) {
        bool forward = itr == 0;
        std::vector<int> executed_idxes;

        bool ok = true;
        for (int i : order) {
            auto &vc = compass_path[i];
            VCompass *vc_next_ptr = i == compass_path.size() - 1 || !forward ? nullptr : &compass_path[i + 1];
            if (vc.next_dir & VCompass::COLLAPSE) {
                // collapse v0 to v1, v0 disappear, v1 left
                const auto &collapse = std::get<Collapse>(vc.topo);

                const V v_from = collapse.quad->corners.at(collapse.k_from);
                const V v_right = collapse.quad->corners.at((collapse.k_from + 1) % 4);
                const V v_to = collapse.quad->corners.at(collapse.k_to);
                const V v_left = collapse.quad->corners.at((collapse.k_to + 1) % 4);

                if (debug_quad_zipper >= 2) {
                    std::cout << (vc.next_dir & VCompass::LEFT ? "LEFT " : "RIGHT ") << "Collapse v" << v_from->id
                              << " to v" << v_to->id << std::endl;
                }

                Split s;
                s.vertex_id = v_to->id;
                s.vertex_heading_id = vc.next_dir & VCompass::LEFT ? v_left->id : v_right->id;
                s.vertex_back_id = vc.next_dir & VCompass::LEFT ? v_right->id : v_left->id;
                s.vid_to_create = v_from->id;
                s.qid_to_create = collapse.quad->id;
                s.point_create = v_from->point;
                s.point_original = v_to->point;

                if (!collapse_quad(collapse, itr == 1)) {
                    if (itr == 1)
                        throw;
                    ok = false;
                    break;
                }
                treat_collapse_event(i, v_from, v_to);

                vid_to_node[s.vertex_id].insert(i);
                vid_to_node[s.vertex_heading_id].insert(i);
                vid_to_node[s.vertex_back_id].insert(i);

                std::swap(vc.next_dir, vc.pre_dir);
                vc.topo = s;
            } else {
                Split s = std::get<Split>(vc.topo);
                remove_node_from_map(i, s.vertex_id);
                remove_node_from_map(i, s.vertex_heading_id);
                remove_node_from_map(i, s.vertex_back_id);
                int *next_split_vertex_back_id = nullptr;
                if (vc_next_ptr && vc_next_ptr->next_dir & VCompass::SPLIT) {
                    auto &next_s = std::get<Split>(vc_next_ptr->topo);
                    int *next_vertex_back_id = &next_s.vertex_back_id;
                    if (*next_vertex_back_id != s.vertex_id)
                        throw;
                    next_split_vertex_back_id = next_vertex_back_id;
                    remove_node_from_map(i + 1, *next_vertex_back_id);
                }
                const V center_v = vid_map.at(s.vertex_id);
                const Point_3 point_original = center_v->point;

                if (debug_quad_zipper >= 2) {
                    std::cout << (vc.next_dir & VCompass::LEFT ? "LEFT " : "RIGHT ") << "Split v" << s.vertex_id
                              << " with head v" << s.vertex_heading_id << " and back v" << s.vertex_back_id
                              << std::endl;
                }

                const bool split_turn = vc.next_dir == VCompass::switch_CS(VCompass::switch_LR(vc.pre_dir));
                // if continuous split_turn, change the split order
                if (split_turn) {
                    std::swap(s.vertex_back_id, s.vertex_heading_id);
                }
                // v2 is the newly created one, v1 should == ss.v
                V v2 = split_edge(s, vc.next_dir & VCompass::LEFT);

                std::swap(vc.next_dir, vc.pre_dir);

                bool topo_ok;
                vc.topo = find_quad_from_diag(v2, center_v, topo_ok);
                if (!topo_ok)
                    throw;
                std::get<Collapse>(vc.topo).has_point_original = true;
                std::get<Collapse>(vc.topo).point_original = point_original;

                if (next_split_vertex_back_id) {
                    const int vid_influence_next = split_turn ? s.vertex_id : v2->id;
                    *next_split_vertex_back_id = vid_influence_next;
                    vid_to_node[vid_influence_next].insert(i + 1);
                }
                if (forward)
                    treat_split_event(center_v, vid_map.at(s.vertex_back_id), vid_map.at(s.vertex_heading_id), v2);
                else
                    undo_collapse(i);
            }

            executed_idxes.push_back(i);

            if (debug_quad_zipper >= 3) {
                emit need_repaint();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (debug_quad_zipper)
                check_valid();
        }

        if (forward)
            order.assign(rbegin(executed_idxes), rend(executed_idxes));

        if (!ok) {
            succeed = false;
            continue; // reserve to cancel this execution
        }
        if (forward)
            if (callback)
                callback();
        if (!reverse)
            break;
    }
    return succeed;
}

void Quad2d3d::try_all_four_zipper_directions(const std::vector<int> &path, int &best_dir, double &potential_drop) {
    if (debug_quad_zipper)
        std::cout << green << " ==== try_all_four_zipper_directions for path v " << path.front() << " to v "
                  << path.back() << " ====" << white << std::endl;
    best_dir = -1;
    potential_drop = 0.;
    const int compass_starts[4] = {VCompass::LEFT | VCompass::COLLAPSE, VCompass::RIGHT | VCompass::COLLAPSE,
                                   VCompass::LEFT | VCompass::SPLIT, VCompass::RIGHT | VCompass::SPLIT};
    for (int i = 0; i < 4; i++) {
        if (debug_quad_zipper)
            check_valid();
        const int dir = compass_starts[i];
        if (debug_quad_zipper)
            std::cout << green << "  dir  " << VCompass::DirToString(dir) << ":" << white << std::endl;

        double quad_shape_and_size_score = 0.;
        if (!convert_simple_path_to_zipper_path(path, dir, compass_path, quad_shape_and_size_score)) {
            if (debug_quad_zipper)
                std::cout << "  validating boundary condition failed!" << std::endl;
            continue;
        }

        double pot_drop;
        {
            const double pot = determine_potential_for_compass_path(compass_path);
            if (debug_quad_zipper)
                printf("  before: pot: %.8f\n\n", pot);
            if (debug_quad_zipper >= 1)
                emit need_repaint();

            double pot_after;
            if (!execute_compass_path(true, [&]() {
                    pot_after = determine_potential_for_compass_path(compass_path);
                    if (debug_quad_zipper >= 1)
                        emit need_repaint();
                })) {

                if (debug_quad_zipper)
                    std::cout << "  execute compass path failed!" << std::endl;
                continue;
            }
            pot_drop = pot - pot_after;
            if (debug_quad_zipper) {
                printf("  after : pot: %.8f\n", pot_after);
                printf("  drop  %.8f\n", pot_drop);
            }
            if (use_shape_score) {
                if (debug_quad_zipper)
                    std::cout << "quad shape and size score = " << quad_shape_and_size_score << std::endl;
                pot_drop += quad_shape_and_size_score;
            }
        }
        if (debug_quad_zipper)
            std::cout << yellow << "  pot drop:  " << pot_drop << white << std::endl;
        if (pot_drop > potential_drop) {
            best_dir = dir;
            potential_drop = pot_drop;
        }
    }
    if (best_dir >= 0) {
        if (debug_quad_zipper)
            std::cout << green << "   best dir for v " << path.front() << " is " << VCompass::DirToString(best_dir)
                      << white << std::endl;
    }
    compass_path.clear();
}

bool Quad2d3d::quad_move_given_singularity_pair(const std::vector<int> &vid_list, int start_compass_dir,
                                                bool adjust_extremity, bool throw_when_fail) {

    if (start_compass_dir <= 0)
        throw;

    compass_path.clear();

    double quad_shape_violation;
    if (!convert_simple_path_to_zipper_path(vid_list, start_compass_dir, compass_path, quad_shape_violation,
                                            adjust_extremity, throw_when_fail)) {
        return false;
    }

    for (const auto &vc : compass_path) {
        if (vc.next_dir & VCompass::COLLAPSE)
            std::get<Collapse>(vc.topo).quad->is_need_collapse = true;
    }

    const double pot = determine_potential_for_compass_path(compass_path);
    if (debug_quad_zipper >= 1)

        printf("  before: pot: %.8f\n", pot);

    if (debug_quad_zipper >= 1)
        emit need_repaint();

    double pot_after;
    const bool ok = execute_compass_path(reverse_quad_zipper,
                                         [&]() { pot_after = determine_potential_for_compass_path(compass_path); });
    if (debug_quad_zipper >= 1)
        emit need_repaint();
    if (ok) {
        double pot_drop = pot - pot_after;
        if (debug_quad_zipper >= 1) {
            printf("  after : pot: %.8f\n", pot_after);
            printf("  drop  %.8f\n", pot_drop);
        }
        if (use_shape_score) {
            if (debug_quad_zipper)
                std::cout << "quad_shape_violation = " << quad_shape_violation << std::endl;
            pot_drop -= quad_shape_violation;
        }
        if (debug_quad_zipper >= 1)
            std::cout << "  pot drop:  " << pot_drop << std::endl;
    } else {
        std::cout << "  failed to execute compass path" << std::endl;
    }
    compass_path.clear();
    return true;
}

void Quad2d3d::batch_quad_zipper(bool hold) {
    check_valid();
    update_current_average_side_length();
    print_statistics();

    if (!hold)
        held_zipper_paths.clear();
    std::vector<PathWithDir> best_paths_filtered;
    if (!held_zipper_paths.empty()) {
        best_paths_filtered.swap(held_zipper_paths);
    } else {
        size_t hash = 0;
        for (const auto &v : vertices) {
            hash ^= std::hash<size_t>{}(v.valence + hash);
        }
        std::default_random_engine e{unsigned(hash)};

        std::cout << green << "==== auto_quad_zipper_move ====" << white << std::endl;
        std::vector<V> vs_start;
        for (auto v = vertices.begin(); v != vertices.end(); ++v) {
            if (v->is_frozen)
                continue;
            if (effective_valence(v) >= 5 || effective_valence(v) == 3)
                vs_start.push_back(v);
        }
        std::shuffle(begin(vs_start), end(vs_start), e);
        std::vector<std::vector<int>> paths_ids;
        std::set<std::pair<int, int>> v_pair_seen;
        {
            const int selections = std::min(int(vs_start.size()), int(vs_start.size() * 0.5 + 10));
            std::cout << "Generating paths starting from " << selections << " singularities among " << vs_start.size()
                      << " singularities" << std::endl;
            ScopeTimer t("Generating path");
            for (int i = 0; i < selections; i++) {
                const auto &v_start = vs_start[i];
                auto add_paths = [&](const std::function<bool(const Vertex_handle &)> &is_target) {
                    std::vector<Path> paths_to_target =
                        path_to({v_start}, is_target, [this](const V &v) { return effective_valence(v) == 4; }, 3);
                    if (paths_to_target.empty())
                        return;
                    for (const auto &path : paths_to_target) {
                        int v1_id = v_start->id;
                        int v2_id = path.back()->id;
                        if (v1_id > v2_id)
                            std::swap(v1_id, v2_id);
                        if (v_pair_seen.emplace(v1_id, v2_id).second) {
                            paths_ids.emplace_back();
                            auto &item = paths_ids.back();
                            for (const auto &v : path)
                                item.push_back(v->id);
                            if (debug_quad_zipper >= 1)
                                std::cout << "path v " << v_start->id << " to v " << path.back()->id << std::endl;
                        }
                    }
                };
                add_paths([this, &v_start](const V &cur_v) -> bool {
                    if (cur_v == v_start)
                        return false;
                    return effective_valence(cur_v) <= 3;
                });
                add_paths([this, &v_start](const V &cur_v) -> bool {
                    if (cur_v == v_start)
                        return false;
                    return effective_valence(cur_v) >=
                           5 /*&& !cur_v->is_sharp_edge_feature*/; // it's possible to end with v5 on sharp edge
                });
            }
            if (paths_ids.empty()) {
                std::cerr << "no available 5-3 path" << std::endl;
                return;
            }
        }

        // std::sort(begin(paths_ids), end(paths_ids), [](const auto& a, const auto& b) {return a.size() < b.size(); });

        std::vector<PathWithDir> best_paths;
        {
            ScopeTimer t("Try path", true, false);
            int total_length = 0;
            for (const auto &path_ids : paths_ids) {
                if (path_ids.empty())
                    throw;
                total_length += path_ids.size();
                simple_path = path_ids;
                if (debug_quad_zipper) {
                    for (int id : simple_path)
                        std::cout << id << " ";
                    std::cout << std::endl;
                }

                if (debug_quad_zipper >= 1)
                    emit need_repaint();
                auto path_vid = std::move(simple_path);
                simple_path.clear();
                int best_dir;
                double potential_drop;

                try_all_four_zipper_directions(path_vid, best_dir, potential_drop);

                if (best_dir >= 0 && potential_drop > 0.) {
                    PathWithDir path_dir;
                    path_dir.path = std::move(path_vid);
                    path_dir.dir = best_dir;
                    path_dir.potential_drop = potential_drop;
                    best_paths.push_back(std::move(path_dir));
                } else {
                    if (debug_quad_zipper)
                        std::cout << "no favorable movements, try another one..." << std::endl;
                }
            }
            std::sort(begin(best_paths), end(best_paths), [](const PathWithDir &p1, const PathWithDir &p2) {
                return p1.potential_drop > p2.potential_drop;
            });
            std::cout << "collected " << best_paths.size() << " qzip paths" << std::endl;
            std::cout << "ave qzip length: " << (double)total_length / paths_ids.size() << std::endl;
        }

        std::set<int> touched_vertices;
        {
            ScopeTimer t("Pick path");
            for (const PathWithDir &p : best_paths) {
                bool already_taken = false;
                for (int vid : p.path) {
                    auto nbs = collect_quad_one_ring_vertices_in_order(vid_map.at(vid));
                    nbs.push_back(vid_map.at(vid));
                    for (const V &v : nbs) {
                        if (touched_vertices.find(v->id) != end(touched_vertices)) {
                            already_taken = true;
                            break;
                        }
                    }
                    if (already_taken) {
                        break;
                    }
                }
                if (!already_taken) {
                    best_paths_filtered.push_back(p);
                    for (int vid : p.path) {
                        auto nbs = collect_quad_one_ring_vertices_in_order(vid_map.at(vid));
                        nbs.push_back(vid_map.at(vid));
                        for (const V &nb : nbs) {
                            touched_vertices.insert(nb->id);
                            auto nbnbs = collect_quad_one_ring_vertices_in_order(nb);
                            for (const V &nbnb : nbnbs)
                                touched_vertices.insert(nbnb->id);
                        }
                    }
                }
            }
        }

        std::cout << yellow << "Found " << best_paths_filtered.size() << " non intersecting paths" << white
                  << std::endl;
        best_paths_filtered.resize(
            std::min(int(best_paths_filtered.size()), int(best_paths_filtered.size() * 0.5) + 10));

        if (hold) {
            held_zipper_paths.swap(best_paths_filtered);
            return;
        }
    }
    {
        ScopeTimer t("Execute path");
        for (const auto &best_path : best_paths_filtered) {

            if (debug_quad_zipper >= 1) {
                std::cout << yellow << "execute path :" << white << std::endl;
                std::cout << "path from v " << best_path.path.front() << " valence "
                          << effective_valence(vid_map.at(best_path.path.front())) << std::endl;
                std::cout << "       to v " << best_path.path.back() << " valence "
                          << effective_valence(vid_map.at(best_path.path.back())) << ":" << std::endl;
                for (const auto &vid : best_path.path)
                    std::cout << vid << " ";
                std::cout << std::endl;
            }

            simple_path = best_path.path;
            if (debug_quad_zipper >= 1)
                emit need_repaint();
            simple_path.clear();

            if (!quad_move_given_singularity_pair(best_path.path, best_path.dir, true, true))
                throw;
        }
    }
    if (best_paths_filtered.empty()) {
        std::cout << "All paths tried, no favorable movements" << std::endl;
    } else {
        std::cout << yellow << "Executed " << best_paths_filtered.size() << " paths" << white << std::endl;
    }

    check_valid();
    print_statistics();
}

void Quad2d3d::select_to_build_quad_zipper(const V &v, bool auto_adjust_extremity) {
    if (simple_path.empty()) {
        selected_vertices.push_back(v);
        simple_path.push_back(v->id);
        std::cout << "Starting vertex selected: vid " << v->id << std::endl;
        return;
    }

    if (simple_path.back() == v->id) {
        run_single_quad_zipper(auto_adjust_extremity);
        return;
    }

    if (simple_path.front() == v->id) {
        run_single_quad_zipper(auto_adjust_extremity, -1);
        return;
    }

    if (std::find(begin(simple_path), end(simple_path), v->id) != end(simple_path)) {
        std::cout << "same vertex as before, cancel" << std::endl;
        simple_path.clear();
        selected_vertices.clear();
        return;
    }

    const V tail = selected_vertices.back();

    const auto is_target = [v](const V &cur_v) { return cur_v == v; };
    const auto vertex_ok = [this](const V &v) { return effective_valence(v) == 4; };
    std::cout << "find path from v" << tail->id << " to v" << v->id << std::endl;
    std::vector<Path> paths = path_to({tail}, is_target, vertex_ok);
    if (paths.empty()) {
        std::cout << "Cannot found path" << std::endl;
        return;
    }
    const auto &path = paths.front();

    for (int i = 1; i < path.size(); i++)
        simple_path.push_back(path[i]->id);
    selected_vertices.push_back(v);
}

void Quad2d3d::run_single_quad_zipper(bool auto_adjust_extremity, int dir) {
    if (dir == 0) {
        while (true) {
            std::cout << "Please type in the direction you want to start: LC, LS, RC, RS" << std::endl;
            std::string type;
            std::cin >> type;
            if (type == "LC") {
                dir = VCompass::LEFT | VCompass::COLLAPSE;
                break;
            }
            if (type == "LS") {
                dir = VCompass::LEFT | VCompass::SPLIT;
                break;
            }
            if (type == "RC") {
                dir = VCompass::RIGHT | VCompass::COLLAPSE;
                break;
            }
            if (type == "RS") {
                dir = VCompass::RIGHT | VCompass::SPLIT;
                break;
            }
        }
    }

    const auto path = simple_path;

    selected_vertices.clear();
    emit need_repaint();
    update_current_average_side_length();

    for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
        qit->is_need_collapse = false;
        qit->is_new = false;
    }

    if (dir < 0) {
        int best_dir;
        double potential_drop;
        try_all_four_zipper_directions(path, best_dir, potential_drop);

        if (best_dir >= 0) {
            std::cout << "  decide to execute dir " << VCompass::DirToString(best_dir) << std::endl;
            dir = best_dir;
        } else {
            std::cout << "  no favorable movements" << std::endl;
            return;
        }
    }
    if (!quad_move_given_singularity_pair(path, dir, auto_adjust_extremity)) {
        std::cout << "Failed" << std::endl;
    } else {
        std::cout << "Done" << std::endl;
    }

    for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
        qit->is_need_collapse = false;
        qit->is_new = false;
    }
    simple_path.clear();

    std::cout << quads.size() << " quads, " << vertices.size() << " vertices" << std::endl;
}

void Quad2d3d::qzip_and_laplacian(int iters) {
    for (int i = 0; i < iters; i++) {
        std::cout << green << "Iteration " << i + 1 << "/" << iters << white << std::endl;
        this->batch_quad_zipper(true);
        emit need_repaint();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        this->batch_quad_zipper(true);
        emit need_repaint();
        this->laplacian_smooth();
        emit need_repaint();

        break_high_valence();
        collapse_low_valence();
        emit need_repaint();
    }
}

void Quad2d3d::func1() {
    std::vector<V> singularities;
    for (auto it = vertices.begin(); it != vertices.end(); ++it)
        if (effective_valence(it) != 4)
            singularities.push_back(it);

    int total = 0;
    int intersected = 0;
    std::vector<std::vector<int>> self_intersecting_paths;
    std::set<int> seen_vertices_id;
    for (int i = 0; i < singularities.size() - 1; i++) {
        const auto vi = singularities[i];
        seen_vertices_id.insert(vi->id);
        const auto is_target = [&](const V &cur_v) {
            return cur_v != vi && effective_valence(cur_v) != 4 &&
                   seen_vertices_id.find(cur_v->id) == seen_vertices_id.end();
        };
        const auto vertex_ok = [this](const V &v) { return effective_valence(v) == 4; };
        std::vector<Path> paths = path_to({vi}, is_target, vertex_ok, 1000000);
        for (const auto &p : paths) {
            total++;
            const bool intersect = check_path_self_intersect_in_regular_grid(p);
            if (intersect) {
                intersected++;
                self_intersecting_paths.emplace_back();
                for (const auto &v : p)
                    self_intersecting_paths.back().push_back(v->id);
            }
        }
    }
    simple_paths_for_render = std::move(self_intersecting_paths);
    std::cout << "Intersected / total: " << intersected << "/" << total << std::endl;
}

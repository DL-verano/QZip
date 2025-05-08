#pragma once
#include "../tools/ramp.h"
#include "meta_types.h"
#include "oracle.h"
#include <QObject>
#include <list>
#include <set>
#include <unordered_set>
#include <variant>

inline Vector_3 safe_normalize(const Vector_3 &n) {
    double sql = n.squared_length();
    if (sql == 0.)
        return CGAL::NULL_VECTOR;
    return n / std::sqrt(sql);
}

class Quad2d3d : public QObject {
    Q_OBJECT
  public:
  signals:
    void need_repaint();

    // basic structure
  public:
    struct Quad;
    struct Vertex;
    typedef std::list<Vertex>::iterator Vertex_handle;
    typedef std::list<Vertex>::const_iterator Vertex_handle_const;
    typedef std::list<Quad>::iterator Quad_handle;
    typedef std::list<Quad>::const_iterator Quad_handle_const;

    struct Vertex {
        int id = -1;
        Quad_handle quad;
        int next_vid_over_sharp_edge = -1;
        int pre_vid_over_sharp_edge = -1;
        Point_3 point;
        static int id_counter;
        int valence = 0;
        bool on_boundary_or_sharp_edge = false;
        bool on_boundary = false;
        bool is_sharp_vertex = false;
        bool is_frozen = false;
        int temp_flag = 0;
        static int flag_counter;
    };

    struct Quad {
        // counter-clock wise vertex indices, the k-th and k+1-th vertex forms the k-th edge
        int id = -1;
        std::array<Vertex_handle, 4> corners;
        std::array<Quad_handle, 4> neighbors;
        std::array<Quad_handle, 4> sharp_edge_neighbors;
        std::array<bool, 4> is_sharp_edge_candidate = {false, false, false, false};
        static int id_counter;
        // 0 - nearly isotropic, 1 - from 03 to 12, 2 - from 01 to 32, 3 - from 0 to 2, 4 - from 1 to 3
        int stretch_direction = -1;
        double stretch_ratio;

        bool problematic_size = false; // too large or too small
        double size_ratio;             // size / average_size

        // for render
        bool is_new = false;
        bool is_need_collapse = false;
        bool is_frozen = false;

        int index(Vertex_handle_const v) const {
            for (int i = 0; i < 4; i++)
                if (corners.at(i) == v)
                    return i;
            throw;
        }

        bool has(Vertex_handle_const v) const {
            for (int i = 0; i < 4; i++)
                if (corners.at(i) == v)
                    return true;
            return false;
        }

        int index(Quad_handle_const q) const {
            int count = 0;
            int r = -1;
            for (int i = 0; i < 4; i++) {
                if (neighbors.at(i) == q) {
                    r = i;
                    count++;
                }
            }
            if (r < 0)
                throw;
            if (count > 1)
                throw;
            return r;
        }

        bool has(Quad_handle_const q) const {
            for (int i = 0; i < 4; i++)
                if (neighbors.at(i) == q)
                    return true;
            return false;
        }

        int index_sharp_neighbor(Quad_handle_const q) const {
            int count = 0;
            int r = -1;
            for (int i = 0; i < 4; i++) {
                if (sharp_edge_neighbors.at(i) == q) {
                    r = i;
                    count++;
                }
            }
            if (r < 0)
                throw;
            if (count > 1)
                throw;
            return r;
        }

        bool has_sharp_neighbor(Quad_handle_const q) const {
            for (int i = 0; i < 4; i++)
                if (sharp_edge_neighbors.at(i) == q)
                    return true;
            return false;
        }

        int mirror_index(int k) const { return neighbors.at(k)->index(corners[(k + 1) % 4]); }

        Vector_3 calc_normal() const;

        double area() const;

        double angle(int k) const;
    };

    struct Vertex_handle_hash {
        size_t operator()(const Vertex_handle &v) const { return std::hash<void *>{}(&*v); }
    };

    struct Quad_handle_hash {
        size_t operator()(const Quad_handle &q) const { return std::hash<void *>{}(&*q); }
    };

  public:
    std::list<Vertex> vertices;
    std::list<Quad> quads;
    std::unordered_map<int, Vertex_handle> vid_map;
    bool use_gaussian_curvature_valence = false;

    void clear();
    bool empty() const { return vertices.empty(); }
    void construct_from_vertices_quads(std::list<Vertex> &&vertices, std::list<Quad> &&quads,
                                       const std::vector<int> *frozen_quad_ids = nullptr);
    std::vector<Point_3> dump_vertices() const;
    void check_valid() const;
    void map_sharp_feature_vertex(std::map<int, int> &feature_vid_map, std::set<int> &redundant_vids) const;
    void save_quad(const QString &filename) const;
    void save_quad_to_obj(const QString &filename) const;
    void load_quad(const QString &filename);
    void save_quad_sharp_edge_candidate(const QString &filename) const;
    void load_quad_sharp_edge_candidate(const QString &filename);
    bool is_boundary_or_on_sharp_edge(Vertex_handle_const v) const;
    bool is_boundary(Vertex_handle_const v) const;
    void rotate_vertex_quad_to_boundary(Vertex_handle v);
    static Quad_handle rotate_quad_ccw(Quad_handle_const q, Vertex_handle_const v) {
        int k = q->index(v);
        return q->neighbors[(k + 3) % 4];
    }

    static Quad_handle rotate_quad_cw(Quad_handle_const q, Vertex_handle_const v) {
        return q->neighbors.at(q->index(v));
    }
    Quad_handle_const get_quad_from_edge_ccw(Vertex_handle_const v0, Vertex_handle_const v1) const;
    Quad_handle get_quad_from_edge_ccw(Vertex_handle v0, Vertex_handle v1);
    bool locate_vertex(const Point_3 &p, Vertex_handle &v);
    std::string get_statistics() const;
    void print_statistics() const;
    void get_bounding_sphere(double &x, double &y, double &z, double &r) const;

    void calculate_quad_stretch(Quad_handle q);
    void update_all_quad_stretch();
    void calculate_quad_size_ratio(Quad_handle q);
    void update_current_average_side_length();
    void update_all_quad_size_ratio();
    double calc_shape_score(double quad_stretch_ratio) const;
    double calc_size_score(double quad_size_ratio) const;

    void set_oracle(const std::shared_ptr<Oracle> &o) { oracle = o; }
    void guess_quad_sharp_feature_from_oracle();
    void confirm_sharp_feature_guess(double sharp_edge_turn_angle);
    void split_given_sharp_edges(const std::vector<std::pair<Quad_handle, int>> &edges, double sharp_edge_turn_angle);
    void mark_sharp_vertex_by_angle(bool use_sharp_vertex, double sharp_angle_degree, double sharp_edge_turn_angle);

    void change_edge_feature_type(Quad_handle q, const Point_3 &intersection);
    std::pair<Vertex_handle, Quad_handle> pick(Kernel::Ray_3 &ray, Point_3 &intersection);
    void froze_quad(const Quad_handle &q);
    void isolate_frozen_quad();
    int effective_valence(const Vertex_handle_const& v) const;

  private:
    void split_vertices_by_sharp_edges(double sharp_edge_turn_angle);
    std::shared_ptr<Oracle> oracle;

    // basic operations
  public:
    typedef std::vector<Vertex_handle> Path;
    struct Split {
        int vertex_id;
        int vertex_heading_id;
        int vertex_back_id;
        int vid_to_create = -1;
        int qid_to_create = -1;
        Point_3 point_original, point_create;
    };

    struct Collapse {
        Quad_handle quad;
        int k_from, k_to;
        bool has_point_original = false;
        Point_3 point_original;
    };
    bool collapse_quad(const Collapse &collapse, bool tolerate_bad_case = false);
    Vertex_handle split_edge(Split split, bool is_left);
    void break_high_valence();
    void collapse_low_valence();
    void collapse_thin_quad();
    void find_quad(int id);
    void find_vertex(int id);

    void select_vertex_for_collapse(const Point_3 &p);
    void select_vertex_for_split(const Point_3 &p);

  private:
    void update_vertex_valence(const Vertex_handle &v);
    std::vector<Vertex_handle> collect_quad_one_ring_vertices_in_order(Vertex_handle_const v) const;
    Vector_3 calc_vertex_normal(const Vertex_handle &v) const;
    int calc_vertex_gaussian_curvature_valence(const Vertex_handle_const &v) const;
    Collapse find_quad_from_diag(Vertex_handle v0, Vertex_handle v1, bool &topo_ok) const;
    std::vector<Path> path_to(const std::unordered_set<Vertex_handle, Vertex_handle_hash> &roots,
                              const std::function<bool(const Vertex_handle &)> &is_target,
                              const std::function<bool(const Vertex_handle &)> &vertex_ok =
                                  [](const Vertex_handle &) { return true; },
                              int max_number_of_path = 1, bool randomize_path = false) const;
    Path build_path_from_vid_list(const Vertex_handle &start, const std::vector<int> &vid_list) const;

    // quad zipper
  public:
    struct VCompass {
        enum {
            UNDEFINED = 0,
            LEFT = 0x01,
            RIGHT = 0x02,
            COLLAPSE = 0x04,
            SPLIT = 0x08,
        };

        static int RotateCounterClockwise(int dir, int rotation) {
            rotation = rotation % 4;
            if (rotation < 0)
                rotation += 4;
            const int compass[4] = {RIGHT | COLLAPSE, LEFT | COLLAPSE, RIGHT | SPLIT, LEFT | SPLIT};
            int i = 0;
            for (; i < 4; i++)
                if (compass[i] == dir)
                    break;
            if (i == 4)
                throw;
            return compass[(rotation + i) % 4];
        }

        static std::string DirToString(int dir) {
            std::string s;
            if (dir & LEFT)
                s += "LEFT ";
            if (dir & RIGHT)
                s += "RIGHT ";
            if (dir & COLLAPSE)
                s += "COLLAPSE ";
            if (dir & SPLIT)
                s += "SPLIT ";
            return s;
        }

        int pre_dir = UNDEFINED;
        int next_dir = UNDEFINED;

        std::variant<std::monostate, Split, Collapse> topo;

        static int switch_LR(int d) {
            if (d == UNDEFINED)
                throw;
            return d ^ (LEFT | RIGHT);
        }

        static int switch_CS(int d) {
            if (d == UNDEFINED)
                throw;
            return d ^ (COLLAPSE | SPLIT);
        }
    };
    void set_debug_quad_zipper(int debug) { debug_quad_zipper = debug; }
    void set_reverse_quad_zipper(bool checked) { reverse_quad_zipper = checked; }
    void batch_quad_zipper(bool hold = false);
    void clean_held_quad_zipper() { held_zipper_paths.clear(); }
    void select_to_build_quad_zipper(const Vertex_handle &v, bool auto_adjust_extremity);
    void run_single_quad_zipper(bool auto_adjust_extremity, int dir = 0);
    void set_zipper_potential_search_radius(int radius) { potential_search_radius = radius; }
    void set_zipper_potential_boundary(bool checked) { potential_settings.use_boundary_push = checked; }
    void set_zipper_potential_boundary_score(double score) { potential_settings.boundary_push_score = score; }
    void set_zipper_potential_appealing(bool checked) { potential_settings.use_diff_type_appealing = checked; }
    void set_zipper_potential_appealing_score(double score) { potential_settings.appealing_score = score; }
    void set_zipper_potential_repelling(bool checked) { potential_settings.use_same_type_repelling = checked; }
    void set_zipper_potential_repelling_score(double score) { potential_settings.repelling_score = score; }
    void set_zipper_potential_separatrix(bool checked) { potential_settings.use_separatrix = checked; }
    void set_zipper_potential_separatrix_score(double score) { potential_settings.separatrix_score = score; }
    void qzip_and_laplacian(int iters);
    void func1();
    struct PotentialSettings {
        bool use_boundary_push = true;
        bool use_same_type_repelling = true;
        bool use_diff_type_appealing = true;
        bool use_separatrix = false;
        double boundary_push_score = 5.0;
        double appealing_score = 1.0;
        double repelling_score = 1.0;
        double separatrix_score = 1.0;
    };

  private:
    int debug_quad_zipper = 0;
    bool reverse_quad_zipper = false;
    int potential_search_radius = 50;
    std::vector<VCompass> compass_path;
    std::vector<int> simple_path;
    std::vector<std::vector<int>> simple_paths_for_render;
    std::vector<Vertex_handle> selected_vertices;
    PotentialSettings potential_settings;
    int highlighted_vertex_id = -1;
    int highlighted_quad_id = -1;
    std::vector<int> highlight_vertices;

    struct PathWithDir {
        std::vector<int> path;
        int dir = -1;
        double potential_drop = 0.;
    };
    std::vector<PathWithDir> held_zipper_paths;
    int count_turns_in_path(const Path &path, int starting_dir, bool& ok) const;
    bool check_path_self_intersect_in_regular_grid(const Path &path) const;
    bool adjust_zipper_path_start_end_vertex(Path &path, int starting_dir, bool throw_when_fail = false) const;
    bool convert_simple_path_to_zipper_path(const std::vector<int> &path, int start_compass_dir,
                                            std::vector<VCompass> &compass, double &quad_shape_and_size_score,
                                            bool adjust_extremity_vertices = true, bool throw_when_fail = false);
    double search_for_determining_potential(const Vertex_handle &v_start, const std::vector<int> &vid_to_ignore,
                                            bool early_out_for_nearest = true) const;

    double determine_potential_for_compass_path(const std::vector<VCompass> &path) const;
    bool execute_compass_path(bool reverse, std::function<void()> callback = nullptr);
    void try_all_four_zipper_directions(const std::vector<int> &path, int &best_dir, double &potential_drop);
    bool quad_move_given_singularity_pair(const std::vector<int> &vid_list, int start_compass_dir,
                                          bool adjust_extremity = true, bool throw_when_fail = false);

    // renderings
  public:
    void gl_draw(const Ramp &ramp) const;
    void toggle_view_face_id(bool checked) { view_face_id = checked; }
    void toggle_view_vertex_id(bool checked) { view_vertex_id = checked; }
    void toggle_view_quad_edge(bool checked) { view_quad_edge = checked; }
    void toggle_view_face(bool checked) { view_face = checked; }
    void toggle_view_quad_valence(bool checked) { view_quad_valence = checked; }
    void toggle_view_preferred_valence(bool checked) { view_preferred_valence = checked; }
    void toggle_view_rendering_vectors(bool checked) { view_rendering_vectors = checked; }
    void toggle_view_problem_size(bool checked) { view_problem_size = checked; }
    void toggle_view_long_quad(bool checked) { view_long_quad = checked; }
    void toggle_view_vertex_feature(bool checked) { view_vertex_feature = checked; }
    void set_material_index(int index) { material_index = index; }
    void set_render_z_sort(bool sort) { render_with_z_sort = sort; }
    void set_render_quad_scar(bool value) { render_quad_scar = value; }
    void toggle_view_seam_shrink(bool value) { view_seam_shrink = value; }
    void toggle_view_fixed_vertex(bool value) { view_fixed_vertex = value; }
    void highlight_vertex_id(int id);
    void highlight_quad_id(int id);

  private:
    bool view_face_id = false;
    bool view_vertex_id = false;
    bool view_quad_edge = true;
    bool view_face = true;
    bool view_quad_valence = true;
    bool view_preferred_valence = false;
    bool view_rendering_vectors = true;
    bool view_long_quad = true;
    bool view_vertex_feature = true;
    bool view_problem_size = true;
    bool render_with_z_sort = true;
    bool render_quad_scar = false;
    bool view_seam_shrink = false;
    bool view_fixed_vertex = false;
    int material_index = 0;

    // smoothing
  public:
    struct RenderVec {
        Point_3 p;
        Point_3 t;
        int type; // 1,2,3
    };
    std::vector<RenderVec> rendering_vectors;

    double long_quad_threshold = 1.0;
    bool use_shape_score = false;
    double shape_score_strength = 1.0;
    double shape_score_gamma = 1.0;

    bool use_size_score = false;
    double current_average_side_length = 0.;
    double size_ratio_threshold = 1.0;
    double size_score_strength = 1.0;
    double size_score_gamma = 1.0;

    bool use_polar_laplacian = false;
    bool smooth_project_to_surface = true;
    bool smooth_project_to_feature = true;
    double laplacian_step = 0.5;
    int laplacian_iters = 1;
    bool smooth_oracle_pulls = false;

    void laplacian_smooth_iter(bool only_calculate = false);
    void laplacian_smooth();
};

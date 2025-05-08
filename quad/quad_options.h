#pragma once

#include "quad.h"
#include "ui_quad_options.h"

class QuadOptions : public QWidget, private Ui::QuadOptions {
    Q_OBJECT
  private:
    Quad2d3d *quad;
    bool is_picking_edge_for_feature_type_change = false;
    bool is_picking_vertex_for_build_quad_zipper = false;
    bool is_picking_quad_for_frozen = false;
    void emit_picking() {
        emit is_picking_vertex(is_picking_edge_for_feature_type_change || is_picking_vertex_for_build_quad_zipper ||
                               is_picking_quad_for_frozen);
    }

  public:
    QuadOptions(QWidget *parent) : QWidget(parent) { setupUi(this); }
    void set_quad_data(Quad2d3d &q) {
        quad = &q;
        update_button_group();
        update_render_group();
    }
  signals:
    void is_picking_vertex(bool is_picking);

  public slots:

    void update_button_group();

    void update_render_group();

    void on_check_smooth_polar_projection_toggled(bool) { update_button_group(); }
    void on_check_laplacian_project_to_surface_toggled(bool) { update_button_group(); }
    void on_check_laplacian_project_to_feature_toggled(bool) { update_button_group(); }
    void on_spin_laplacian_step_valueChanged(double) { update_button_group(); }
    void on_spin_smooth_times_valueChanged(int) { update_button_group(); }
    void on_check_oracle_pulls_toggled(bool) { update_button_group(); }

    void on_button_laplacian_smooth_clicked() {
        quad->laplacian_smooth();
        emit quad->need_repaint();
    }
    void on_button_mark_sharp_quad_vertex_clicked(bool) {
        quad->mark_sharp_vertex_by_angle(this->check_use_sharp_quad_vertex->isChecked(),
                                         this->spin_sharp_quad_degree->value(), this->spin_sharp_edge_degree->value());
        emit quad->need_repaint();
    }
    void on_button_manual_pick_path_toggled(bool checked) {
        is_picking_vertex_for_build_quad_zipper = checked;
        emit_picking();
    }
    void on_check_guess_quad_sharp_feature_from_oracle_clicked(bool) {
        quad->guess_quad_sharp_feature_from_oracle();
        emit quad->need_repaint();
    }
    void on_button_change_edge_feature_type_toggled(bool checked) {
        is_picking_edge_for_feature_type_change = checked;
        emit_picking();
    }
    void on_check_apply_feature_guess_clicked(bool) {
        quad->confirm_sharp_feature_guess(this->spin_sharp_edge_degree->value());
    }
    void on_button_freeze_quad_toggled(bool checked) {
        is_picking_quad_for_frozen = checked;
        emit_picking();
    }
    void on_button_isolate_frozen_quad_clicked(bool) { quad->isolate_frozen_quad(); }
    void when_ray_picking(double x_cam, double y_cam, double z_cam, double dir_x, double dir_y, double dir_z) {
        Kernel::Ray_3 r(Point_3(x_cam, y_cam, z_cam), Vector_3(dir_x, dir_y, dir_z));
        Point_3 intersection;
        auto pair = quad->pick(r, intersection);
        auto v = pair.first;
        auto q = pair.second;
        if (v == quad->vertices.end())
            return;
        if (is_picking_edge_for_feature_type_change) {
            quad->change_edge_feature_type(q, intersection);
        } else if (is_picking_vertex_for_build_quad_zipper) {
            quad->select_to_build_quad_zipper(v, true);
        } else if (is_picking_quad_for_frozen) {
            quad->froze_quad(q);
        }
        emit quad->need_repaint();
    }

    // potentials
    void on_spin_quad_zipper_potential_radius_valueChanged(int) { update_button_group(); }
    void on_check_zipper_potential_boundary_toggled(bool) { update_button_group(); }
    void on_check_zipper_potential_repelling_toggled(bool) { update_button_group(); }
    void on_check_zipper_potential_appealing_toggled(bool) { update_button_group(); }
    void on_check_zipper_potential_separatrix_toggled(bool) { update_button_group(); }
    void on_spin_boundary_push_score_valueChanged(double) { update_button_group(); }
    void on_spin_apealing_score_valueChanged(double) { update_button_group(); }
    void on_spin_repelling_score_valueChanged(double) { update_button_group(); }
    void on_spin_separatrix_score_valueChanged(double) { update_button_group(); }

    void on_spin_debug_quad_zipper_valueChanged(int) { update_button_group(); }
    void on_button_quad_zipper_clicked() {
        quad->batch_quad_zipper(this->check_quad_zipper_hold->isChecked());
        emit quad->need_repaint();
    }
    void on_button_clean_hold_clicked() {
        quad->clean_held_quad_zipper();
        emit quad->need_repaint();
    }
    void on_check_reverse_quad_zipper_clicked(bool) { update_button_group(); }
    void on_button_break_high_valence_clicked() {
        quad->break_high_valence();
        emit quad->need_repaint();
    }
    void on_button_collapse_low_valence_clicked() {
        quad->collapse_low_valence();
        emit quad->need_repaint();
    }
    void on_button_collapse_thin_quad_clicked() {
        quad->collapse_low_valence();
        emit quad->need_repaint();
    }
    void on_button_qzip_func1_clicked() { quad->func1(); }
    void on_check_use_gaussian_curvature_valence_toggled(bool) { update_button_group(); }

    // shape score
    void on_group_shape_score_toggled(bool) {
        update_button_group();
        quad->update_all_quad_stretch();
        emit quad->need_repaint();
    }
    void on_spin_long_quad_ratio_threshold_valueChanged(double) {
        update_button_group();
        quad->update_all_quad_stretch();
        emit quad->need_repaint();
    }
    void on_spin_shape_score_strength_valueChanged(double) { update_button_group(); }
    void on_spin_shape_score_gamma_valueChanged(double) { update_button_group(); }

    // size score
    void on_group_size_score_toggled(bool) {
        update_button_group();
        quad->update_all_quad_size_ratio();
        emit quad->need_repaint();
    }
    void on_spin_size_ratio_threshold_valueChanged(double) {
        update_button_group();
        quad->update_all_quad_size_ratio();
        emit quad->need_repaint();
    }
    void on_spin_size_score_strength_valueChanged(double) { update_button_group(); }

    void on_button_qzip_and_laplacian_clicked() { quad->qzip_and_laplacian(this->spin_qzip_laplacian_iters->value()); }

    void on_check_render_edge_toggled(bool) { update_render_group(); }
    void on_check_render_face_toggled(bool) { update_render_group(); }
    void on_check_render_valence_toggled(bool) { update_render_group(); }
    void on_check_render_preferred_valence_toggled(bool) { update_render_group(); }
    void on_check_render_long_quad_toggled(bool) { update_render_group(); }
    void on_check_render_vertex_feature_toggled(bool) { update_render_group(); }
    void on_check_render_vec_toggled(bool) { update_render_group(); }
    void on_check_render_problem_size_toggled(bool) { update_render_group(); }
    void on_check_render_fixed_vertex_toggled(bool) { update_render_group(); }

    void on_button_print_statistic_clicked(bool);
    void on_combo_materials_currentIndexChanged(int index) { update_render_group(); }
    void on_check_render_with_z_sort_toggled(bool) { update_render_group(); }
    void on_check_render_quad_scar_toggled(bool) { update_render_group(); }

    void on_check_view_vertex_id_toggled(bool) { update_render_group(); }
    void on_check_view_quad_id_toggled(bool) { update_render_group(); }
    void on_check_seam_shrink_toggled(bool) { update_render_group(); }
    void on_button_find_vertex_id_clicked(bool) {
        quad->highlight_vertex_id(spin_find_vertex_id->value());
        emit quad->need_repaint();
    }
    void on_button_find_quad_id_clicked(bool) {
        quad->highlight_quad_id(spin_find_face_id->value());
        emit quad->need_repaint();
    }
};

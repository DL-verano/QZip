#include "quad_options.h"

void QuadOptions::update_button_group() {
    quad->use_polar_laplacian = this->check_smooth_polar_projection->isChecked();
    quad->smooth_project_to_surface = this->check_laplacian_project_to_surface->isChecked();
    quad->smooth_project_to_feature = this->check_laplacian_project_to_feature->isChecked();
    quad->laplacian_step = this->spin_laplacian_step->value();
    quad->laplacian_iters = this->spin_smooth_times->value();
    quad->smooth_oracle_pulls = this->check_oracle_pulls->isChecked();

    quad->set_zipper_potential_search_radius(spin_quad_zipper_potential_radius->value());
    quad->set_zipper_potential_boundary(check_zipper_potential_boundary->isChecked());
    quad->set_zipper_potential_repelling(check_zipper_potential_repelling->isChecked());
    quad->set_zipper_potential_appealing(check_zipper_potential_appealing->isChecked());
    quad->set_zipper_potential_separatrix(check_zipper_potential_separatrix->isChecked());
    quad->set_zipper_potential_boundary_score(spin_boundary_push_score->value());
    quad->set_zipper_potential_appealing_score(spin_apealing_score->value());
    quad->set_zipper_potential_repelling_score(spin_repelling_score->value());
    quad->set_zipper_potential_separatrix_score(spin_separatrix_score->value());

    quad->set_debug_quad_zipper(spin_debug_quad_zipper->value());
    quad->set_reverse_quad_zipper(check_reverse_quad_zipper->isChecked());

    quad->shape_score_strength = spin_shape_score_strength->value();
    quad->long_quad_threshold = spin_long_quad_ratio_threshold->value();
    quad->use_shape_score = group_shape_score->isChecked();
    quad->shape_score_gamma = spin_shape_score_gamma->value();

    quad->use_gaussian_curvature_valence = check_use_gaussian_curvature_valence->isChecked();

    quad->use_size_score = group_size_score->isChecked();
    quad->size_ratio_threshold = spin_size_ratio_threshold->value();
    quad->size_score_gamma = spin_size_score_strength->value();
}

void QuadOptions::update_render_group() {
    quad->toggle_view_quad_edge(this->check_render_edge->isChecked());
    quad->toggle_view_face(this->check_render_face->isChecked());
    quad->toggle_view_quad_valence(this->check_render_valence->isChecked());
    quad->toggle_view_preferred_valence(this->check_render_preferred_valence->isChecked());
    quad->toggle_view_long_quad(this->check_render_long_quad->isChecked());
    quad->toggle_view_vertex_feature(this->check_render_vertex_feature->isChecked());
    quad->toggle_view_rendering_vectors(this->check_render_vec->isChecked());
    quad->toggle_view_problem_size(this->check_render_problem_size->isChecked());
    quad->set_material_index(this->combo_materials->currentIndex());
    quad->set_render_z_sort(this->check_render_with_z_sort->isChecked());
    quad->set_render_quad_scar(this->check_render_quad_scar->isChecked());
    quad->toggle_view_vertex_id(this->check_view_vertex_id->isChecked());
    quad->toggle_view_face_id(this->check_view_quad_id->isChecked());
    quad->toggle_view_seam_shrink(this->check_seam_shrink->isChecked());
    quad->toggle_view_fixed_vertex(this->check_render_fixed_vertex->isChecked());
    emit quad->need_repaint();
}

void QuadOptions::on_button_print_statistic_clicked(bool) { quad->print_statistics(); }

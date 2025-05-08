#include "../tools/gl_text.h"
#include "../tools/gl_tools.h"
#include "../tools/materials.h"
#include "quad.h"
#include "tools/scope_timer.h"
#include <random>

typedef Quad2d3d::Vertex_handle V;
typedef Quad2d3d::Vertex_handle_const cV;

void Quad2d3d::gl_draw(const Ramp &ramp) const {
    // get camera position
    GLdouble camera_pos[3];
    {
        GLdouble model_view[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
        GLdouble projection[16];
        glGetDoublev(GL_PROJECTION_MATRIX, projection);
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        gluUnProject((viewport[2] - viewport[0]) / 2, (viewport[3] - viewport[1]) / 2, 0.0, model_view, projection,
                     viewport, &camera_pos[0], &camera_pos[1], &camera_pos[2]);
    }
    const Point_3 p_cam(camera_pos[0], camera_pos[1], camera_pos[2]);

    std::map<int, Point_3> v_offset_pos;
    if (view_seam_shrink) {
        for (auto vit = begin(vertices); vit != end(vertices); ++vit) {
            if (!vit->on_boundary_or_sharp_edge)
                continue;
            auto q = vit->quad;
            Vector_3 vec = CGAL::NULL_VECTOR;
            int count = 0;
            while (q != quads.end()) {
                auto v_oppo = q->corners[(q->index(vit) + 2) % 4];
                vec += v_oppo->point - vit->point;
                count++;
                q = q->neighbors[(q->index(vit) + 3) % 4];
            }
            v_offset_pos[vit->id] = vit->point + vec * 0.3 / count;
        }
    }

    auto get_v_pos = [this, &v_offset_pos](const cV &v) {
        if (!view_seam_shrink)
            return v->point;
        if (!v->on_boundary_or_sharp_edge)
            return v->point;
        return v_offset_pos.at(v->id);
    };
    {
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        for (auto vit = begin(vertices); vit != end(vertices); ++vit) {
            const auto p = get_v_pos(vit);
            if (vit->next_vid_over_sharp_edge >= 0) {
                glColor3ub(200, 0, 0);
                auto next = vid_map.at(vit->next_vid_over_sharp_edge);
                auto q = get_v_pos(next);
                q = p + (q - p) * 0.4;
                glVertex3d(p.x(), p.y(), p.z());
                glVertex3d(q.x(), q.y(), q.z());
            }
            if (vit->pre_vid_over_sharp_edge >= 0) {
                glColor3ub(200, 200, 0);
                auto pre = vid_map.at(vit->pre_vid_over_sharp_edge);
                auto q = get_v_pos(pre);
                q = p + (q - p) * 0.5;
                glVertex3d(p.x(), p.y(), p.z());
                glVertex3d(q.x(), q.y(), q.z());
            }
        }
        glEnd();
    }

    if (!held_zipper_paths.empty()) {
        glLineWidth(4.0f);
        glBegin(GL_LINES);
        glColor3ub(200, 100, 10);
        for (const auto &p : held_zipper_paths) {
            for (int i = 0; i < int(p.path.size()) - 1; i++) {
                const auto &v0 = p.path[i];
                const auto &v1 = p.path[i + 1];
                const auto it0 = vid_map.find(v0);
                const auto it1 = vid_map.find(v1);
                if (it0 == vid_map.end() || it1 == vid_map.end())
                    continue;
                const Point_3 p1 = get_v_pos(it0->second);
                const Point_3 p2 = get_v_pos(it1->second);
                glVertex3d(p1.x(), p1.y(), p1.z());
                glVertex3d(p2.x(), p2.y(), p2.z());
            }
        }
        glEnd();
    }

    if (!simple_path.empty()) {
        glLineWidth(4.0f);
        glBegin(GL_LINES);
        glColor3ub(200, 150, 10);
        for (int i = 0; i < simple_path.size() - 1; i++) {
            const auto &v0 = simple_path[i];
            const auto &v1 = simple_path[i + 1];
            const auto it0 = vid_map.find(v0);
            const auto it1 = vid_map.find(v1);
            if (it0 == vid_map.end() || it1 == vid_map.end())
                continue;
            const Point_3 p1 = get_v_pos(it0->second);
            const Point_3 p2 = get_v_pos(it1->second);
            glVertex3d(p1.x(), p1.y(), p1.z());
            glVertex3d(p2.x(), p2.y(), p2.z());
        }
        glEnd();
    }
    {
        std::default_random_engine e(0);
        std::normal_distribution<double> n(0., 1.);
        unsigned char r = 255, g = 0, b = 0;
        for (const auto &path : simple_paths_for_render) {
            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glColor3ub(r, g, b);
            const auto v0id = path.front();
            const auto v0 = vid_map.at(v0id);
            Point_3 p0 = get_v_pos(v0);
            for (int i = 1; i < path.size(); i++) {
                const auto &v1id = path[i];
                const auto v1 = vid_map.at(v1id);
                Point_3 p = get_v_pos(v1);
                const double d = std::sqrt(CGAL::squared_distance(p0, p)) * 0.1;
                const Point_3 p1(p.x() + d * n(e), p.y() + d * n(e), p.z() + d * n(e));
                glVertex3d(p0.x(), p0.y(), p0.z());
                glVertex3d(p1.x(), p1.y(), p1.z());
                p0 = p1;
            }
            glEnd();
            r = (r * 113 + g * 13 + b * 3 + 1) * 117 % 256;
            g = (r * 13 + g * 113 + b * 17 + 2) * 113 % 256;
            b = (r * 117 + g * 13 + b * 5 + 3) * 211 % 256;
        }
    }

    if (!selected_vertices.empty()) {
        glPointSize(6.0f);
        glBegin(GL_POINTS);
        glColor3ub(255, 0, 0);
        for (auto v : selected_vertices) {
            const auto p = get_v_pos(v);
            glVertex3d(p.x(), p.y(), p.z());
        }

        glEnd();
    }

    if (view_quad_edge) {
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        for (const auto &quad : quads) {
            for (int i = 0; i < 4; i++) {
                int j = (i + 1) % 4;
                if (quad.neighbors.at(i) != quads.end() && quad.id > quad.neighbors.at(i)->id)
                    continue;
                if (quad.sharp_edge_neighbors.at(i) != quads.end())
                    glColor3ub(255, 255, 0);
                else if (quad.is_sharp_edge_candidate[i])
                    glColor3ub(200, 0, 0);
                else
                    glColor3ub(0, 0, 0);

                const auto p0 = get_v_pos(quad.corners.at(i));
                const auto p1 = get_v_pos(quad.corners.at(j));
                glVertex3d(p0.x(), p0.y(), p0.z());
                glVertex3d(p1.x(), p1.y(), p1.z());
            }
        }
        glEnd();
        if (!view_face) {

            glLineWidth(4.0f);
            glColor3ub(255, 255, 255);
            glBegin(GL_LINES);
            const Point_3 cam_p(camera_pos[0], camera_pos[1], camera_pos[2]);
            for (const auto &quad : quads) {
                for (int i = 0; i < 4; i++) {
                    int j = (i + 1) % 4;
                    if (quad.neighbors.at(i) == quads.end() || quad.id > quad.neighbors.at(i)->id)
                        continue;
                    const auto &v0 = quad.corners.at(i);
                    const auto &v1 = quad.corners.at(j);
                    const Point_3 p0 = cam_p + (get_v_pos(v0) - cam_p) * 1.01;
                    const Point_3 p1 = cam_p + (get_v_pos(v1) - cam_p) * 1.01;
                    glVertex3d(p0.x(), p0.y(), p0.z());
                    glVertex3d(p1.x(), p1.y(), p1.z());
                }
            }
            glEnd();
        }

        // prior direction
        if (view_long_quad && use_shape_score) {
            glBegin(GL_LINES);
            glColor3ub(200, 30, 30);
            for (const auto &q : quads) {
                const Point_3 &p0 = get_v_pos(q.corners[0]);
                const Point_3 &p1 = get_v_pos(q.corners[1]);
                const Point_3 &p2 = get_v_pos(q.corners[2]);
                const Point_3 &p3 = get_v_pos(q.corners[3]);
                const Point_3 m01 = CGAL::midpoint(p0, p1);
                const Point_3 m12 = CGAL::midpoint(p1, p2);
                const Point_3 m23 = CGAL::midpoint(p2, p3);
                const Point_3 m30 = CGAL::midpoint(p3, p0);
                if (q.stretch_direction == 1) {
                    ramp.gl_color(q.stretch_ratio, 1.0, 2.0);
                    glVertex3d(m30.x(), m30.y(), m30.z());
                    glVertex3d(m12.x(), m12.y(), m12.z());
                } else if (q.stretch_direction == 2) {
                    ramp.gl_color(q.stretch_ratio, 1.0, 2.0);
                    glVertex3d(m01.x(), m01.y(), m01.z());
                    glVertex3d(m23.x(), m23.y(), m23.z());
                } else if (q.stretch_direction == 3) {
                    ramp.gl_color(q.stretch_ratio, 1.0, 2.0);
                    glVertex3d(p0.x(), p0.y(), p0.z());
                    glVertex3d(p2.x(), p2.y(), p2.z());
                } else if (q.stretch_direction == 4) {
                    ramp.gl_color(q.stretch_ratio, 1.0, 2.0);
                    glVertex3d(p1.x(), p1.y(), p1.z());
                    glVertex3d(p3.x(), p3.y(), p3.z());
                }
            }
            glEnd();
        }
    }

    const double vertex_radius = current_average_side_length == 0.0 ? 0.015 : current_average_side_length / 3.;

    {
        ::glEnable(GL_LIGHTING);
        ::glEnable(GL_COLOR_MATERIAL);
        ::glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        gl_set_material(Material::Default);

        auto draw_vertex = [&get_v_pos](cV v, double radius, int r, int g, int b) {
            ::glMatrixMode(GL_MODELVIEW);
            ::glPushMatrix();
            glColor3ub(r, g, b);
            const auto p = get_v_pos(v);
            ::glTranslatef(p.x(), p.y(), p.z());
            GLUquadricObj *quadric = gluNewQuadric();
            gluQuadricDrawStyle(quadric, GLU_FILL);
            gluSphere(quadric, radius, 12, 6);
            gluDeleteQuadric(quadric);
            ::glPopMatrix();
        };

        if (view_vertex_feature) {
            for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
                if (vit->is_sharp_vertex)
                    draw_vertex(vit, vertex_radius, 200, 0, 0);
                else if (vit->on_boundary)
                    draw_vertex(vit, vertex_radius, 0, 200, 0);
                else if (vit->on_boundary_or_sharp_edge)
                    draw_vertex(vit, vertex_radius, 200, 0, 200);
            }
        }

        if (view_quad_valence) {
            for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
                unsigned char rv, gv, bv;
                const int valence_effective = effective_valence(vit);
                if (valence_effective != 4) {
                    if (valence_effective <= 2) {
                        rv = 128;
                        gv = 0;
                        bv = 128;
                    } else if (valence_effective >= 6) {
                        rv = 255;
                        gv = 0;
                        bv = 0;
                    } else if (valence_effective == 3) {
                        rv = 20;
                        gv = 20;
                        bv = 200;
                    } else if (valence_effective == 5) {
                        rv = 250;
                        gv = 50;
                        bv = 0;
                    }
                    draw_vertex(vit, vertex_radius, rv, gv, bv);
                }
            }
        }
        if (view_preferred_valence) {
            for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
                unsigned char rv, gv, bv;
                const int preferred_valence = calc_vertex_gaussian_curvature_valence(vit);
                if (preferred_valence >= 0 && preferred_valence != 4) {
                    if (preferred_valence <= 2) {
                        rv = 128;
                        gv = 0;
                        bv = 128;
                    } else if (preferred_valence >= 6) {
                        rv = 255;
                        gv = 0;
                        bv = 0;
                    } else if (preferred_valence == 3) {
                        rv = 20;
                        gv = 20;
                        bv = 200;
                    } else if (preferred_valence == 5) {
                        rv = 250;
                        gv = 50;
                        bv = 0;
                    }
                    draw_vertex(vit, vertex_radius, rv, gv, bv);
                }
            }
        }
        if (view_fixed_vertex) {
            for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
                if (vit->is_frozen) {
                    draw_vertex(vit, vertex_radius, 0, 255, 255);
                }
            }
        }

        if (highlighted_vertex_id >= 0) {
            auto it = vid_map.find(highlighted_vertex_id);
            if (it != vid_map.end()) {
                const auto v = it->second;
                draw_vertex(v, vertex_radius * 2, 255, 0, 0);
            }
        }

        if (highlighted_quad_id >= 0) {
            for (const auto &q : quads) {
                if (q.id != highlighted_quad_id)
                    continue;

                ::glMatrixMode(GL_MODELVIEW);
                ::glPushMatrix();
                glColor3ub(255, 0, 0);
                const Point_3 p = CGAL::midpoint(CGAL::midpoint(q.corners[0]->point, q.corners[1]->point),
                                                 CGAL::midpoint(q.corners[2]->point, q.corners[3]->point));
                ::glTranslatef(p.x(), p.y(), p.z());
                GLUquadricObj *quadric = gluNewQuadric();
                gluQuadricDrawStyle(quadric, GLU_FILL);
                gluSphere(quadric, vertex_radius * 2, 12, 6);
                gluDeleteQuadric(quadric);
                ::glPopMatrix();
            }
        }

        for (int id : highlight_vertices) {
            auto vit = vid_map.at(id);
            draw_vertex(vit, vertex_radius, 255, 255, 0);
        }
        ::glDisable(GL_COLOR_MATERIAL);
        ::glDisable(GL_LIGHTING);
    }

    glColor3ub(0, 0, 0);

    const double view_range = current_average_side_length * 20;
    if (view_face_id) {
        for (const auto &quad : quads) {
            const auto p0 = get_v_pos(quad.corners[0]);
            const auto p1 = get_v_pos(quad.corners[1]);
            const auto p2 = get_v_pos(quad.corners[2]);
            const auto p3 = get_v_pos(quad.corners[3]);
            if (CGAL::squared_distance(p_cam, p0) > view_range * view_range)
                continue;
            double x = (p0.x() + p1.x() + p2.x() + p3.x()) / 4.0;
            double y = (p0.y() + p1.y() + p2.y() + p3.y()) / 4.0;
            double z = (p0.z() + p1.z() + p2.z() + p3.z()) / 4.0;
            gl_draw_number_3d(x, y, z, 200, 0, 0, std::to_string(quad.id));
        }
    }
    if (view_vertex_id) {
        for (auto vit = vertices.begin(); vit != vertices.end(); ++vit) {
            const auto p = get_v_pos(vit);
            if (CGAL::squared_distance(p_cam, p) > view_range * view_range)
                continue;
            double x = p.x();
            double y = p.y();
            double z = p.z();
            gl_draw_number_3d(x, y, z, 0, 0, 0, std::to_string(vit->id));
        }
    }

    if (!compass_path.empty()) {
        glLineWidth(3.0f);
        glBegin(GL_LINES);
        if (false)
            for (int i = 0; i < compass_path.size(); i++) {
                const auto &vc = compass_path[i];
                if (vc.next_dir & VCompass::COLLAPSE) {
                    const auto &q = std::get<Collapse>(vc.topo);
                    V v0 = q.quad->corners.at(q.k_from);
                    V v1 = q.quad->corners.at(q.k_to);

                    const Point_3 p1 = v0->point;
                    const Point_3 p2 = v1->point;
                    const Point_3 p3 = p1 + 0.8 * (p2 - p1);
                    // Point mid = CGAL::midpoint(p1,p2);
                    if (vc.next_dir & VCompass::LEFT)
                        glColor3ub(0, 0, 255);
                    else
                        glColor3ub(0, 128, 128);
                    glVertex3d(p1.x(), p1.y(), p1.z());
                    glVertex3d(p3.x(), p3.y(), p3.z());
                } else {
                    const Split &s = std::get<Split>(vc.topo);
                    const auto vit = vid_map.find(s.vertex_id);
                    const auto head_it = vid_map.find(s.vertex_heading_id);
                    if (vit == vid_map.end() || head_it == vid_map.end())
                        continue;
                    const Point_3 p1 = vit->second->point;
                    const Vector_3 vec = (head_it->second->point - p1) * 0.8;
                    const Point_3 p2 = p1 + vec;
                    if (vc.next_dir & VCompass::LEFT) {
                        glColor3ub(255, 0, 0);
                    } else {
                        glColor3ub(0, 255, 0);
                    }
                    glVertex3d(p1.x(), p1.y(), p1.z());
                    glVertex3d(p2.x(), p2.y(), p2.z());
                }
            }
        else if (false) {
            glColor3ub(200, 200, 0);
            for (int i = 0; i < compass_path.size(); i++) {
                const auto &vc = compass_path[i];
                if (vc.next_dir & VCompass::COLLAPSE) {
                    const auto &q = std::get<Collapse>(vc.topo);
                    V v0 = q.quad->corners.at(q.k_from);
                    V v1 = q.quad->corners.at(q.k_to);

                    const Point_3 p0 = v0->point;
                    const Point_3 p1 = v1->point;
                    const Point_3 p00 = p0 + (p1 - p0) * 0.3;
                    const Point_3 p11 = p1 + (p0 - p1) * 0.3;

                    glVertex3d(p0.x(), p0.y(), p0.z());
                    glVertex3d(p00.x(), p00.y(), p00.z());
                    glVertex3d(p1.x(), p1.y(), p1.z());
                    glVertex3d(p11.x(), p11.y(), p11.z());
                } else {
                    const Split &s = std::get<Split>(vc.topo);
                    auto vit = vid_map.find(s.vertex_id);
                    auto head_it = vid_map.find(s.vertex_heading_id);
                    auto tail_it = vid_map.find(s.vertex_back_id);
                    if (vit == vid_map.end() || head_it == vid_map.end() || tail_it == vid_map.end())
                        continue;
                    Vertex_handle_const v = vit->second;
                    Vertex_handle_const head = head_it->second;
                    Vertex_handle_const tail = tail_it->second;
                    Quad_handle_const q1 = vc.next_dir & VCompass::LEFT ? get_quad_from_edge_ccw(v, head)
                                                                        : get_quad_from_edge_ccw(head, v);
                    Quad_handle_const q2 = vc.next_dir & VCompass::LEFT ? get_quad_from_edge_ccw(v, tail)
                                                                        : get_quad_from_edge_ccw(tail, v);
                    auto draw_in_quad = [](const Quad_handle_const &q, const Vertex_handle_const &v) {
                        const int k = q->index(v);
                        const auto oppo = q->corners[(k + 2) % 4];
                        const Point_3 p0 = v->point;
                        const Point_3 p1 = v->point + (oppo->point - v->point) * 0.3;
                        glVertex3d(p0.x(), p0.y(), p0.z());
                        glVertex3d(p1.x(), p1.y(), p1.z());
                    };
                    draw_in_quad(q1, v);
                    draw_in_quad(q2, v);
                }
            }
        }
        glEnd();
    }

    if (view_rendering_vectors && !rendering_vectors.empty()) {
        glLineWidth(3.0f);
        glBegin(GL_LINES);
        for (auto &pair : rendering_vectors) {
            const auto &x = pair.p;
            const auto &t = pair.t;
            switch (pair.type) {
            case 1:
                glColor3ub(255, 0, 0);
                break;
            case 2:
                glColor3ub(0, 255, 0);
                break;
            case 3:
                glColor3ub(0, 0, 255);
                break;
            case 4:
                glColor3ub(255, 255, 0);
                break;
            }
            glVertex3d(x[0], x[1], x[2]);
            glVertex3d(t[0], t[1], t[2]);
        }
        glEnd();
    }
    if (view_face) {
        ::glEnable(GL_LIGHTING);
        ::glEnable(GL_POLYGON_OFFSET_FILL); // do not overlap lines
        ::glPolygonOffset(1., 1.);

        auto draw_quad = [&get_v_pos](const Quad_handle_const &quad) {
            const auto a = get_v_pos(quad->corners[0]);
            const auto b = get_v_pos(quad->corners[1]);
            const auto c = get_v_pos(quad->corners[2]);
            const auto d = get_v_pos(quad->corners[3]);
            const Point_3 o = CGAL::midpoint(CGAL::midpoint(quad->corners[0]->point, quad->corners[1]->point),
                                             CGAL::midpoint(quad->corners[2]->point, quad->corners[3]->point));
            gl_shaded_triangle(o, a, b);
            gl_shaded_triangle(o, b, c);
            gl_shaded_triangle(o, c, d);
            gl_shaded_triangle(o, d, a);
        };
        if (view_problem_size) {
            ::glEnable(GL_COLOR_MATERIAL);
            ::glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            gl_set_material(Material::Default);
            ::glBegin(GL_TRIANGLES);
            for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
                if (!qit->problematic_size)
                    continue;
                double ratio_log = std::log2(qit->size_ratio);
                ramp.gl_color(ratio_log, -1.0, 1.0);
                draw_quad(qit);
            }
            ::glEnd();
            ::glDisable(GL_COLOR_MATERIAL);
        }

        if (render_quad_scar) {
            gl_set_material(Material::Default);
            ::glEnable(GL_COLOR_MATERIAL);
            ::glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

            ::glBegin(GL_TRIANGLES);
            ::glColor3ub(150, 150, 150);
            for (auto q = quads.begin(); q != quads.end(); ++q) {
                if (!q->is_need_collapse && !q->is_new)
                    draw_quad(q);
            }

            ::glColor3ub(20, 20, 240);
            for (auto q = quads.begin(); q != quads.end(); ++q) {
                if (q->is_need_collapse)
                    draw_quad(q);
            }

            ::glColor3ub(240, 20, 20);
            for (auto q = quads.begin(); q != quads.end(); ++q) {
                if (q->is_new)
                    draw_quad(q);
            }
            ::glEnd();
            ::glDisable(GL_COLOR_MATERIAL);
        } else {
            gl_set_material(Material(material_index));
            ::glDisable(GL_COLOR_MATERIAL);
            ::glEnable(GL_LIGHTING);
            std::vector<std::pair<Quad_handle_const, double>> quad_depth;
            if (render_with_z_sort) {
                GLdouble model_view[16];
                glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
                GLdouble projection[16];
                glGetDoublev(GL_PROJECTION_MATRIX, projection);
                GLint viewport[4];
                glGetIntegerv(GL_VIEWPORT, viewport);

                for (auto qit = quads.begin(); qit != quads.end(); ++qit) {

                    if (qit->problematic_size && view_problem_size)
                        continue;

                    const Point_3 center =
                        CGAL::midpoint(CGAL::midpoint(get_v_pos(qit->corners[0]), get_v_pos(qit->corners[1])),
                                       CGAL::midpoint(get_v_pos(qit->corners[2]), get_v_pos(qit->corners[3])));
                    GLdouble x, y, z;
                    // get window coords based on 3D coordinates
                    gluProject(center.x(), center.y(), center.z(), model_view, projection, viewport, &x, &y, &z);

                    quad_depth.emplace_back(qit, -z);
                }
                std::sort(quad_depth.begin(), quad_depth.end(),
                          [](std::pair<Quad_handle_const, double> &a, std::pair<Quad_handle_const, double> &b) -> bool {
                              return a.second < b.second;
                          });

                ::glBegin(GL_TRIANGLES);
                for (const auto &pair : quad_depth) {
                    if (!pair.first->is_frozen)
                        draw_quad(pair.first);
                }
                ::glEnd();
            } else {
                ::glBegin(GL_TRIANGLES);
                for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
                    if (!qit->is_frozen)
                        draw_quad(qit);
                }
                ::glEnd();
            }
            gl_set_material(Material::Copper);
            ::glBegin(GL_TRIANGLES);
            for (auto qit = quads.begin(); qit != quads.end(); ++qit) {
                if (qit->is_frozen)
                    draw_quad(qit);
            }
            ::glEnd();
        }
        ::glDisable(GL_LIGHTING);
        ::glDisable(GL_POLYGON_OFFSET_FILL); // do not overlap lines
    }
}

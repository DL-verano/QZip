#pragma once

#include "include_opengl.h"
#include "meta_types.h"

inline void gl_vertex(const Kernel::Point_3 &p) { ::glVertex3d(p.x(), p.y(), p.z()); }

// call glBegin(GL_TRIANGLES) by your self before calling this
inline void gl_shaded_triangle(const Kernel::Point_3 &a, const Kernel::Point_3 &b, const Kernel::Point_3 &c) {
    // compute normal
    Kernel::Vector_3 n = CGAL::cross_product(c - a, b - a);
    n = n / std::sqrt(n.squared_length());

    // draw one front facing
    ::glNormal3d(-n.x(), -n.y(), -n.z());
    gl_vertex(a);
    gl_vertex(b);
    gl_vertex(c);

    // and the other back facing
    ::glNormal3d(n.x(), n.y(), n.z());
    gl_vertex(a);
    gl_vertex(c);
    gl_vertex(b);
}

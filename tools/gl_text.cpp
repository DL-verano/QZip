#include "gl_text.h"
#include "include_opengl.h"
//#include <QtOpenGL>

void gl_draw_number(double x, double y, const std::string &s) {
    GLdouble modelMatrix[16];
    GLdouble projMatrix[16];
    GLint viewport[4];
    ::glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
    ::glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
    ::glGetIntegerv(GL_VIEWPORT, viewport);
    double xx, yy, zz;
    ::gluProject(x, y, 0, modelMatrix, projMatrix, viewport, &xx, &yy, &zz);
    int xv = viewport[0], yv = viewport[1], ww = viewport[2], hh = viewport[3];
    ::glWindowPos2i(xx, yy);
    const unsigned char *t = reinterpret_cast<const unsigned char *>(s.c_str());
    glutBitmapString(GLUT_BITMAP_8_BY_13, t);
}

void gl_draw_number_3d(double x, double y, double z, int r, int g, int b, const std::string &s) {
    GLdouble modelMatrix[16];
    GLdouble projMatrix[16];
    GLint viewport[4];
    ::glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
    ::glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
    ::glGetIntegerv(GL_VIEWPORT, viewport);
    double xx, yy, zz;
    ::gluProject(x, y, z, modelMatrix, projMatrix, viewport, &xx, &yy, &zz);
    int xv = viewport[0], yv = viewport[1], ww = viewport[2], hh = viewport[3];
    ::glColor3ub(r, g, b);
    ::glWindowPos2i(xx + 5, yy + 5);
    const unsigned char *t = reinterpret_cast<const unsigned char *>(s.c_str());
    glutBitmapString(GLUT_BITMAP_8_BY_13, t);
}

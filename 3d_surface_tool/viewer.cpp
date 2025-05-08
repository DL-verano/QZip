#include "viewer.h"
#include "scene.h"
#include <QApplication>
#include <QMouseEvent>

//#include <QGLFunctions>
//#include <CGAL/Qt/CreateOpenGLContext.h>
#include <QGLViewer/manipulatedCameraFrame.h>

void SurfaceViewer::updateSceneSphere(double x, double y, double z, double r) {
    this->setSceneCenter(qglviewer::Vec(x, y, z));
    this->setSceneRadius(r);
    this->showEntireScene();
}

SurfaceViewer::SurfaceViewer(QWidget *parent) : QGLViewer(parent) {
    // setBackgroundColor(::Qt::white);
    // setMouseBinding(Qt::NoModifier, Qt::LeftButton, NO_CLICK_ACTION, true);
}

void SurfaceViewer::setScene(Scene3DSurface *pScene) {
    m_pScene = pScene;
    connect(m_pScene, &Scene3DSurface::scene_sphere_update, this, &SurfaceViewer::updateSceneSphere);
}

void SurfaceViewer::draw() {
    QGLViewer::draw();
    if (show_axis)
        drawAxis();
    ::glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

    if (plane_cut) {
        ::glEnable(GL_CLIP_PLANE0);
        const GLdouble equation[] = {0.1, 0.1, 1.0, cut_value};
        ::glClipPlane(GL_CLIP_PLANE0, equation);
    } else {
        ::glDisable(GL_CLIP_PLANE0);
    }

    m_pScene->render();
}

void SurfaceViewer::initializeGL() {
    QGLViewer::initializeGL();

    ::glEnable(GL_LIGHTING);
    ::glEnable(GL_CULL_FACE);
    ::glCullFace(GL_BACK);

    this->camera()->setZNearCoefficient(0.00001);
    this->camera()->setZClippingCoefficient(1000.0);

    ::glDisable(GL_COLOR_MATERIAL);
    ::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    ::glEnable(GL_BLEND);

    glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 1.0f);
#ifdef GL_VERSION_1_2
    glLightModelf(GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR);
#endif
    // light
    float light = 0.6f;
    GLfloat lightColor0[] = {light, light, light, 1.0f}; // Color
    GLfloat lightPos0[] = {50.f, 50.f, 100.f, 1.0f};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
    glEnable(GL_LIGHT0);
    // GLfloat lightColor1[] = { light, light, light, 1.0f }; //Color
    // GLfloat lightPos1[] = { -4.0f, 1.0f, -8.0f, 1.0f };
    // glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor1);
    // glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);
    // glEnable(GL_LIGHT1);
    // Add ambient light
    GLfloat ambientColor[] = {0.3f, 0.3f, 0.3f, 1.0f}; // Color(0.2, 0.2, 0.2)
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);
    ::glEnable(GL_MULTISAMPLE);
    ::glEnable(GL_LINE_SMOOTH);
    //::glEnable(GL_POLYGON_SMOOTH);
    ::glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    //::glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
}

void SurfaceViewer::set_pivot_center(qglviewer::Vec center) { this->camera()->setPivotPoint(center); }

void SurfaceViewer::rotate_camera_around_pivot(double angle_in_radian) {
    qglviewer::ManipulatedCameraFrame *p = camera()->frame();
    auto pivot = camera()->pivotPoint();
    auto up = camera()->upVector();
    auto pos = camera()->position();
    auto vec = pos - pivot;
    auto new_pos = qglviewer::Quaternion(up, angle_in_radian).rotate(vec) + pivot;
    p->rotate(qglviewer::Quaternion(qglviewer::Vec(0.0, 1.0, 0.0), angle_in_radian));
    p->setPosition(new_pos);
}

void SurfaceViewer::mousePressEvent(QMouseEvent *e) {
    if (e->modifiers() == Qt::ControlModifier)
        m_custom_mouse = true;

    QGLViewer::mousePressEvent(e);
}

void SurfaceViewer::mouseReleaseEvent(QMouseEvent *e) {
    if (m_custom_mouse) {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        QApplication::restoreOverrideCursor();
        m_custom_mouse = false;
    }

    if (e->button() == Qt::LeftButton) {
        qglviewer::Vec orig;
        qglviewer::Vec dir;
        this->camera()->convertClickToLine(e->pos(), orig, dir);
        Kernel::Ray_3 r(Point_3(orig.x, orig.y, orig.z), Vector_3(dir.x, dir.y, dir.z));
        Kernel::Point_3 center;
        if (select_rotation_center) {

            if (m_pScene->get_intersection(r, center)) {
                set_pivot_center(qglviewer::Vec(center.x(), center.y(), center.z()));
                std::cout << "new pivot selected" << std::endl;
            }
            select_rotation_center = false;
        } else if (is_picking) {
            emit ray_picked(orig.x, orig.y, orig.z, dir.x, dir.y, dir.z);
        }
    }
    QGLViewer::mouseReleaseEvent(e);
}

#pragma once
#include "include_opengl.h"
#include <QGLViewer/qglviewer.h>

// forward declarations
class QWidget;
class Scene3DSurface;

class SurfaceViewer : public QGLViewer {
    Q_OBJECT

  private:
    Scene3DSurface *m_pScene = nullptr;
    bool m_custom_mouse = false;
    bool plane_cut = false;
    double cut_value;
    bool show_axis = false;
  signals:
    void ray_picked(double x_cam, double y_cam, double z_cam, double dir_x, double dir_y, double dir_z);
  public slots:
    void updateSceneSphere(double x, double y, double z, double r);
  public:
    bool select_rotation_center = false;
    bool is_picking = false;
    SurfaceViewer(QWidget *parent);

    // overload several QGLViewer virtual functions
    void draw() override;
    void initializeGL() override;
    void setScene(Scene3DSurface *pScene);
    void set_axis(bool value) { show_axis = value; }
    void set_plane_cut(bool value) { plane_cut = value; }
    void set_plane_cut_value(double value) { cut_value = value; }
    void set_is_picking(bool value) { is_picking = value; }

    void set_pivot_center(qglviewer::Vec center);

    void rotate_camera_around_pivot(double angle_in_radian);

  protected:
    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
};

#pragma once
#include "include_opengl.h"
#include <CGAL/Qt/DemosMainWindow.h>
#include <QtOpenGL/qgl.h>

class QDragEnterEvent;
class QDropEvent;

class SurfaceViewer;
class Scene3DSurface;

namespace Ui {
class MainWindow;
}

class MainWindow3DSurface : public CGAL::Qt::DemosMainWindow {
    Q_OBJECT
  private:
    Ui::MainWindow *ui;

    bool recording = false;

    SurfaceViewer *m_pSurfaceViewer;
    Scene3DSurface *m_pScene;

    void update_button_group();

    void update_render_option_and_redraw();

  public slots:
    // render
    void updateWindow();
    void save_snapshot_file(const QString &filename);

  public:
    MainWindow3DSurface(QWidget *parent = nullptr);
    ~MainWindow3DSurface();

  public slots:
    void on_actionOpen_triggered();
    void on_actionOpen_Triangle_Mesh_triggered();
    void on_actionOpen_Quad_Mesh_triggered();
    void on_actionSaveQuad_triggered();
    void on_actionExport_Quad_to_OBJ_triggered();
    void on_actionSave_Quad_Sharp_Edge_Candidate_triggered();
    void on_actionLoad_Quad_Sharp_Edge_Candidate_triggered();

    void on_actionRotation_Center_triggered();
    void on_actionRecord_toggled(bool checked);

    void on_actionConvert_OBJ_to_Off_triggered();
    void on_actionConvert_OBJ_to_Quad_triggered();
    void on_actionBatch_Runner_triggered();

    void on_check_show_oracle_toggled(bool) { update_render_option_and_redraw(); }
    void on_check_show_quad_surface_toggled(bool) { update_render_option_and_redraw(); }
    void on_check_show_oracle_sharp_edge_sample_toggled(bool) { update_render_option_and_redraw(); }
    void on_check_auto_recenter_when_load_toggled(bool value);
};

#include "window.h"
#include "scene.h"
#include "viewer.h"
#include <fstream>

#include <CGAL/Qt/debug.h>
#include <QFileDialog>
#include <QSettings>

#include "quad/read_obj.h"
#include "ui_surface.h"
#include <QInputDialog>
#include <QMessageBox>

void MainWindow3DSurface::updateWindow() {
    std::cout << "Update Window" << std::endl;
    m_pSurfaceViewer->update();
    m_pSurfaceViewer->repaint();
    if (recording) {
        m_pSurfaceViewer->saveSnapshot(true, true);
    }
}

void MainWindow3DSurface::save_snapshot_file(const QString &filename) {
    m_pSurfaceViewer->update();
    m_pSurfaceViewer->saveSnapshot(filename, true);
}

MainWindow3DSurface::MainWindow3DSurface(QWidget *parent) : CGAL::Qt::DemosMainWindow(parent) {
    ui = new Ui::MainWindow;
    ui->setupUi(this);

    // saves some pointers from ui, for latter use.
    m_pSurfaceViewer = ui->viewer;

    // does not save the state of the SurfaceViewer
    m_pSurfaceViewer->setStateFileName(QString::null);

    m_pSurfaceViewer->setSnapshotFormat("PNG");

    recording = false;

    // accepts drop events
    setAcceptDrops(true);

    // setups scene
    m_pScene = new Scene3DSurface;
    m_pSurfaceViewer->setScene(m_pScene);
    connect(m_pScene, SIGNAL(need_repaint(void)), this, SLOT(updateWindow(void)));
    connect(m_pScene, SIGNAL(need_save_snapshot(const QString &)), this, SLOT(save_snapshot_file(const QString &)));
    connect(m_pSurfaceViewer, &SurfaceViewer::ray_picked,
            [this](double x_cam, double y_cam, double z_cam, double dir_x, double dir_y, double dir_z) {
                ui->quad_options->when_ray_picking(x_cam, y_cam, z_cam, dir_x, dir_y, dir_z);
            });
    ui->quad_options->set_quad_data(m_pScene->get_quad());
    connect(ui->quad_options, &QuadOptions::is_picking_vertex,
            [this](bool is_picking) { m_pSurfaceViewer->set_is_picking(is_picking); });

    update_button_group();
    update_render_option_and_redraw();
}

MainWindow3DSurface::~MainWindow3DSurface() {
    delete m_pScene;
    delete ui;
}

void MainWindow3DSurface::on_actionOpen_triggered() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open surface file"), "", tr("mesh (*.off *.obj)"));
    if (fileName.isEmpty())
        return;

    QMessageBox::StandardButton load_sharp_feature =
        QMessageBox::question(this, "Load Sharp Feature", "Do you want to load feature line file ?");
    QString feature_line_fileName;
    double sharp_edge_threshold_degree = 0., sharp_edge_turn_threshold_degree = 0.;
    if (load_sharp_feature == QMessageBox::Yes) {
        feature_line_fileName =
            QFileDialog::getOpenFileName(this, tr("Open feature line file"), "", tr("feature line (*.fl)"));
        if (feature_line_fileName.isEmpty())
            return;
    } else {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        bool detect_sharp_edge;
        sharp_edge_threshold_degree = QInputDialog::getDouble(
            this, "Sharp edges and vertices?", "sharp edge when dihedral angle < (0.0 means no sharp edge)", 150., 0.0,
            180., 1, &detect_sharp_edge);
        if (!detect_sharp_edge)
            sharp_edge_threshold_degree = 0.;
    }

    bool detect_sharp_edge_turns;
    sharp_edge_turn_threshold_degree =
        QInputDialog::getDouble(this, "Sharp edges turns?", "sharp edge turns when angle < (0.0 means no sharp turn)",
                                160., 0.0, 180., 1, &detect_sharp_edge_turns);
    if (!detect_sharp_edge_turns)
        sharp_edge_turn_threshold_degree = 0.;

    m_pScene->open_surface_mesh(fileName, feature_line_fileName, sharp_edge_threshold_degree,
                                sharp_edge_turn_threshold_degree);
    QApplication::restoreOverrideCursor();
    update();
}

void MainWindow3DSurface::on_actionOpen_Triangle_Mesh_triggered() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open triangle mesh file"), "", tr("off (*.off)"));
    if (fileName.isEmpty())
        return;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    m_pScene->open_triangle_mesh(fileName);
    QApplication::restoreOverrideCursor();
    update();
}

void MainWindow3DSurface::on_actionOpen_Quad_Mesh_triggered() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open quad mesh file"), "", tr("Quads (*.quad *.obj)"));
    if (fileName.isEmpty())
        return;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    m_pScene->open_quad_mesh(fileName);
    QApplication::restoreOverrideCursor();
    update();
}

void MainWindow3DSurface::on_actionSaveQuad_triggered() {
    QString filename = QFileDialog::getSaveFileName(this, tr("Save file"), ".", "Quads (*.quad)");
    if (!filename.isEmpty()) {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        m_pScene->save_quad_mesh(filename);
        QApplication::restoreOverrideCursor();
    }
}

void MainWindow3DSurface::on_actionExport_Quad_to_OBJ_triggered() {
    QString filename = QFileDialog::getSaveFileName(this, tr("Export Quad to OBJ"), ".", "OBJ (*.obj)");
    if (!filename.isEmpty()) {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        m_pScene->save_quad_mesh_to_obj(filename);
        QApplication::restoreOverrideCursor();
    }
}

void MainWindow3DSurface::on_actionSave_Quad_Sharp_Edge_Candidate_triggered() {
    QString filename = QFileDialog::getSaveFileName(this, tr("Save file"), ".", "Quad Feature (*.qfea)");
    if (!filename.isEmpty()) {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        m_pScene->save_quad_mesh_sharp_edge_candidate(filename);
        QApplication::restoreOverrideCursor();
    }
}

void MainWindow3DSurface::on_actionLoad_Quad_Sharp_Edge_Candidate_triggered() {
    QString fileName =
        QFileDialog::getOpenFileName(this, tr("Open quad feature vertex file"), "", tr("Quads feature (*.qfea)"));
    if (fileName.isEmpty())
        return;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    m_pScene->open_quad_mesh_sharp_edge_candidate(fileName);
    QApplication::restoreOverrideCursor();
    update();
}

void MainWindow3DSurface::on_actionRotation_Center_triggered() { m_pSurfaceViewer->select_rotation_center = true; }

void MainWindow3DSurface::on_actionRecord_toggled(bool checked) { recording = checked; }

void MainWindow3DSurface::on_actionConvert_OBJ_to_Off_triggered() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open OBJ mesh file"), "", tr("OBJ (*.obj)"));
    if (fileName.isEmpty())
        return;

    std::vector<std::array<double, 3>> pts;
    std::vector<std::vector<int>> faces;
    if (!ReadOBJFile(fileName.toStdString(), pts, faces))
        return;

    std::ofstream out(fileName.toStdString() + ".off");
    out << "OFF\n";
    out << pts.size() << " " << faces.size() << " 0\n";
    for (const auto &p : pts) {
        out << p[0] << " " << p[1] << " " << p[2] << "\n";
    }
    for (const auto &f : faces) {
        out << f.size();
        for (int i : f) {
            out << " " << i;
        }
        out << std::endl;
    }
}

void MainWindow3DSurface::on_actionConvert_OBJ_to_Quad_triggered() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open OBJ mesh file"), "", tr("OBJ (*.obj)"));
    if (fileName.isEmpty())
        return;
    std::vector<std::array<double, 3>> pts;
    std::vector<std::vector<int>> faces;
    if (!ReadOBJFile(fileName.toStdString(), pts, faces))
        return;

    for (const auto &f : faces) {
        if (f.size() != 4) {
            std::cout << "non quad face found !" << std::endl;
            return;
        }
    }

    std::ofstream output(fileName.toStdString() + ".quad");
    output << "3d ";
    output << pts.size() << " vertices " << faces.size() << " quads\n";
    for (int i = 0; i < pts.size(); i++) {
        output << i << "\t" << pts[i][0] << "\t" << pts[i][1] << "\t" << pts[i][2] << "\n";
    }
    int count = 0;
    for (const auto &f : faces) {
        output << count++ << "\t";
        for (int i = 0; i < 4; i++)
            output << f[i] << "\t";
        output << "\n";
    }
    output.close();
    std::cout << "done" << std::endl;
}

void MainWindow3DSurface::on_actionBatch_Runner_triggered() {
    const std::string oracle_folder = "C:/Users/flm86/Downloads/PMQ2018_input/triangle_meshes/smooth/";
    const std::string result_folder = "C:/Users/flm86/Downloads/PMQ2018_input/qzip_result/qzipResult/smooth/";
    const std::vector<std::string> mesh_names = {
        "armadillo",
        "armchair",
        "bimba100K",
        "blade",
        "botijo",
        "buddha",
        "bumpy_sphere",
        "bumpy_torus",
        "bunnyBotsch",
        "buste",
        "camel",
        "camille_hand100K",
        "chair",
        //"chinese_lion100K",
        "cow2",
        "cup",
        "dancer_25k",
        //"dancer2",
        "deformed_armadillo",
        "dente",
        "dilo",
        "duck",
        "eight",
        "elephant",
        "elk",
        "eros100K",
        //"face-YO",
        "foot",
        "gargoyle100K",
        "foot",
        "genus3",
        "hand_olivier",
        "hand",
        //"helmet",
        "homer",
        "horse",
        "igea100k",
        "isidore_horse",
        "kitten100K",
        "knot1",
        "laurent_hand",
        "lion_recon_50K",
        "lucy100k",
        "magalie_hand100K",
        "mannequin_mc",
        "mannequin-devil",
        "mask",
        "moai",
        "mouse",
        "neptune0",
        "oil_pump100K",
        "oni",
        "pegaso",
        "pensatore",
        "pierrot100k",
        "pig",
        "polygirl",
        "pulley100K",
        "rabbit5k",
        "ramses",
        "red_circular_box100K",
        //"rgb_dragon",
        "robocat_deci",
        "rocker_arm",
        "rolling_stage100K",
        "santa",
        "screwdriver",
        "shark",
        "thai_statue",
        "torso",
        "uu-memento100k",
        "vase100K",
        "woodenfish",
        "wrench50k",
    };

    for (const auto &mesh : mesh_names) {
        m_pScene->auto_tool(oracle_folder, result_folder, mesh);
    }
}

void MainWindow3DSurface::on_check_auto_recenter_when_load_toggled(bool value) {
    m_pScene->set_auto_recenter_when_load(value);
}

void MainWindow3DSurface::update_button_group() {}

void MainWindow3DSurface::update_render_option_and_redraw() {
    m_pScene->set_view_oracle(this->ui->check_show_oracle->isChecked());
    m_pScene->set_view_quad(this->ui->check_show_quad_surface->isChecked());
    m_pScene->set_view_oracle_sharp_edge_sample(this->ui->check_show_oracle_sharp_edge_sample->isChecked());
    updateWindow();
}

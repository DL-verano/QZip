#include "include_opengl.h"
#include "window.h"
#include <QApplication>

#include <CGAL/Qt/resources.h>

int main(int argc, char **argv) {
    srand(0);

    glutInit(&argc, argv);

    QApplication app(argc, argv);

    // for windows
#if (QT_VERSION >= QT_VERSION_CHECK(5, 3, 0))
    app.setAttribute(Qt::AA_UseDesktopOpenGL);
#endif

    CGAL_QT_INIT_RESOURCES;

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(16);
    // format.setVersion(3, 2);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    QSurfaceFormat::setDefaultFormat(format);

    MainWindow3DSurface mainWindow;
    mainWindow.show();

    GLenum err = glewInit();
    if (GLEW_OK != err) {
        throw "error";
    }

    return app.exec();
}

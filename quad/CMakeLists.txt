project (quad)

set(CMAKE_INCLUDE_CURRENT_DIR ON)


file(GLOB HDRS	"*.h")

file(GLOB SRCS	"*.cpp")


qt5_wrap_ui(QUAD_UI_FILES quad_options.ui)

# The executable itself.
add_library(quad ${SRCS} ${HDRS} ${QUAD_UI_FILES})

qt5_use_modules(quad Gui)

# Link with GLUT & GLEW libraries
target_link_libraries( quad ${GLUT_LIBRARIES} ${GLEW_LIBRARIES})

# Link with Qt libraries
target_link_libraries( quad ${QT_LIBRARIES} )

# Link with libQGLViewer, Glut and OpenGL
target_link_libraries( quad ${QGLVIEWER_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCL_LIBRARIES})


# Link with CGAL
target_link_libraries( quad CGAL::CGAL CGAL::CGAL_Qt5
Qt5::Xml
Qt5::Core
Qt5::OpenGL
Qt5::Concurrent)
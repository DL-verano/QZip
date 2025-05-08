project (tools)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCL REQUIRED)

file(GLOB HDRS	"*.h")

file(GLOB SRCS	"*.cpp")


# The executable itself.
add_library(tools ${SRCS} ${HDRS})

target_include_directories(tools PUBLIC ${OpenCL_INCLUDE_DIRS})

target_link_libraries( tools ${GLUT_LIBRARIES} ${GLEW_LIBRARIES})

# Link with GLUT & GLEW libraries
target_link_libraries( tools ${GLUT_LIBRARIES} ${GLEW_LIBRARIES})

# Link with Qt libraries
target_link_libraries( tools ${QT_LIBRARIES} )

# Link with libQGLViewer, Glut and OpenGL
target_link_libraries( tools ${QGLVIEWER_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY}  ${OpenCL_LIBRARIES})


# Link with CGAL
target_link_libraries( tools CGAL::CGAL CGAL::CGAL_Qt5
Qt5::Xml
Qt5::Core
Qt5::OpenGL
Qt5::Concurrent)

target_link_libraries(tools Eigen3::Eigen)
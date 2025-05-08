project(3d_surface_tool)

# Include this package's headers first
include_directories(BEFORE ./ ./include)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
file(GLOB HDRS
	"*.h"
)

file(GLOB SRCS
	"*.cpp"
)


include_directories( ${QGLVIEWER_INCLUDE_DIR} ${CMAKE_BINARY_DIR}/quad)

# UI files (Qt Designer files)
qt5_wrap_ui(DT_UI_FILES surface.ui)

# The executable itself.
add_executable(3d_surface_tool ${SRCS} ${MOCS} ${DT_UI_FILES} ${DT_RESOURCE_FILES} ${HDRS})

qt5_use_modules(3d_surface_tool Gui OpenGL Xml Svg)

# Link with Qt libraries
target_link_libraries( 3d_surface_tool ${QT_LIBRARIES} )


target_link_libraries( 3d_surface_tool quad tools)
# Link with CGAL
target_link_libraries( 3d_surface_tool CGAL::CGAL CGAL::CGAL_Qt5)

# Link with libQGLSurfaceViewer, OpenGL
target_link_libraries( 3d_surface_tool ${QGLVIEWER_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )


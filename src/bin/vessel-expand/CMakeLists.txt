include(${PROJECT_SOURCE_DIR}/cmake/opengl.cmake)
set(APP_NAME vesselExpand)

find_package(OpenMP)

find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)
find_package(Qt5 COMPONENTS Core REQUIRED QUIET)
find_package(Qt5 COMPONENTS OpenGL REQUIRED QUIET)
find_package(Qt5 COMPONENTS Xml REQUIRED QUIET)

find_package(CGAL REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})

aux_source_directories(SOURCES "" .)
add_executable(${APP_NAME} ${SOURCES})
target_link_libraries(${APP_NAME} Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Xml)
target_link_libraries(${APP_NAME} CGAL::CGAL CGAL::CGAL_Core)
target_link_libraries(${APP_NAME} geogram_gfx geogram ${GLFW_LIBRARIES})

if(OpenMP_CXX_FOUND)
    target_link_libraries(${APP_NAME} OpenMP::OpenMP_CXX)
endif()

install_runtime_targets(${APP_NAME})

set_target_properties(${APP_NAME} PROPERTIES FOLDER "Vessel")

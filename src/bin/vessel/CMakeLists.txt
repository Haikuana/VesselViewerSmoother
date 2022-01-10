include(${PROJECT_SOURCE_DIR}/cmake/opengl.cmake)
set(APP_NAME vessel)

find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)
find_package(Qt5 COMPONENTS Core REQUIRED QUIET)
find_package(Qt5 COMPONENTS OpenGL REQUIRED QUIET)
find_package(Qt5 COMPONENTS Xml REQUIRED QUIET)

find_package(CGAL REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})

aux_source_directories(SOURCES "" .)
add_executable(${APP_NAME} ${SOURCES})

#target_link_libraries(${APP_NAME} FloatTetwild CLI11::CLI11 warnings::all)	
#target_compile_definitions(${APP_NAME} PUBLIC USE_FTETWILD_REMESHING)

target_link_libraries(${APP_NAME} nanoflann::nanoflann)	
target_compile_definitions(${APP_NAME} PUBLIC VorPrint_WITH_nanoflannKdtree)

target_link_libraries(${APP_NAME} Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Xml)
target_link_libraries(${APP_NAME} CGAL::CGAL CGAL::CGAL_Core)
target_link_libraries(${APP_NAME} geogram_gfx geogram ${GLFW_LIBRARIES})
install_runtime_targets(${APP_NAME})

set_target_properties(${APP_NAME} PROPERTIES FOLDER "Vessel")

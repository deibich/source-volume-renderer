find_package(imgui CONFIG REQUIRED)

add_library(utils NanoViewer.h NanoViewer.cpp)
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(utils
  PUBLIC
    owl::owl
    owl_viewer
    imgui::imgui
)

add_library(SourceVolumeRenderer::utils ALIAS utils)

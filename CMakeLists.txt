cmake_minimum_required(VERSION 3.25)

project(SourceVolumeRenderer
  VERSION 1.0
  LANGUAGES CXX CUDA
)

find_package(tinyxml2 CONFIG REQUIRED)
find_package(unofficial-cfitsio CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)

set(owl_dir ${PROJECT_SOURCE_DIR}/thirdParty/owl)
OPTION(OWL_BUILD_SAMPLES OFF)
add_subdirectory(${owl_dir} EXCLUDE_FROM_ALL)
add_subdirectory(${owl_dir}/samples/common)

add_subdirectory(owl_programs)
add_subdirectory(utils)

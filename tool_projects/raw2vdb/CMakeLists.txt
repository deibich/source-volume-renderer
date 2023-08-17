cmake_minimum_required(VERSION 3.10)
project(raw2vdb)

find_package(OpenVDB CONFIG REQUIRED)

add_executable(raw2vdb raw2vdb.cpp)

target_link_libraries(raw2vdb PRIVATE OpenVDB::openvdb)

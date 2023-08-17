cmake_minimum_required(VERSION 3.10)

project(fits2raw)


find_package(unofficial-cfitsio CONFIG REQUIRED)

add_executable(fits2raw fits2raw.cpp)

target_link_libraries(fits2raw cfitsio)
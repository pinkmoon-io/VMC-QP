cmake_minimum_required(VERSION 3.4...3.18)
project(VMC LANGUAGES CXX)
INCLUDE_DIRECTORIES(/usr/include/eigen3/Eigen)
INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(./inc)
INCLUDE_DIRECTORIES(./)

# ADD_SUBDIRECTORY(Goldfarb_Optimizer)
add_subdirectory(pybind11)
ADD_SUBDIRECTORY(src)





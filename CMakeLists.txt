cmake_minimum_required(VERSION 3.21)
project(QuadtreeImplementation)

#set(CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/qt/6.2.2/lib/cmake/")

#find_package(Qt6 COMPONENTS Widgets REQUIRED)

#include_directories(/opt/homebrew/Cellar/boost/1.76.0/include/boost)
#include_directories(/home/linuxbrew/.linuxbrew/include/)
#include_directories(/home/linuxbrew/.linuxbrew/Cellar/qt/6.2.2/include/)

include_directories(/opt/homebrew/include)
include_directories(/opt/homebrew/Cellar)
include_directories(/opt/homebrew/Cellar/boost/1.76.0/include/boost)


find_package(CGAL)
#find_package(Qt6Core REQUIRED)
#find_package(Qt6Widgets REQUIRED)
#find_package(Qt6Quick REQUIRED)

#find_package(OpenMP)
#link_directories(/usr/local/lib/)
#link_directories(/home/linuxbrew/.linuxbrew/Cellar/)

#set(CMAKE_CXX_FLAGS "-O3 -fopenmp -Wall -Wextra")
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O0 -fexceptions -stdlib=libc++ -Xpreprocessor -fopenmp -frounding-math -Wall -Wextra -Wno-unused")
set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/qt/6.2.2/lib/cmake/")
#set(CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/qt/6.2.2/QtWidgets/")
#set(CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/boost/1.76.0/include/boost")

link_directories(/opt/homebrew/lib/)
link_directories(/opt/homebrew/Cellar/)
link_directories(/opt/homebrew/Cellar/boost/1.76.0/include/boost)

add_executable(QuadtreeImplementation main.cpp QuadtreeNeighborFinder.h)

target_link_libraries(QuadtreeImplementation libgmp.a)
target_link_libraries(QuadtreeImplementation libmpfr.a)
target_link_libraries(QuadtreeImplementation libomp.a)
#target_link_libraries(Sp_Mer Qt6::Core)
#target_link_libraries(Sp_Mer Qt6::Widgets)
target_link_libraries(QuadtreeImplementation CGAL)

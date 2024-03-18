cmake_minimum_required(VERSION 3.0.0)
project(SUN3DGroundtruthOptimizer VERSION 0.1.0 LANGUAGES C CXX)

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(FMT REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(SUN3D_GTOptimizer_Lib SHARED
    src/ThreadPool.cc
    src/Dataloader.cc
    src/FrameExtractor.cc
    src/SIFTextractor.cc
)

include(CTest)
enable_testing()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable(SUN3DGroundtruthOptimizer src/GroundtruthOptimizer.cc)

target_include_directories(SUN3D_GTOptimizer_Lib PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/3rdParty/Sophus
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(SUN3D_GTOptimizer_Lib PRIVATE fmt::fmt-header-only)

target_link_libraries(SUN3DGroundtruthOptimizer SUN3D_GTOptimizer_Lib ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

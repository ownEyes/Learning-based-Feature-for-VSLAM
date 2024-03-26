cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CUDA_ARCHITECTURES 52 60 61 75 80 89)

project(SUN3DGroundtruthOptimizer VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CUDA_HOST_COMPILER "${CMAKE_CXX_COMPILER}")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -ccbin ${CMAKE_CXX_COMPILER}")

message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "CMAKE CUDA FLAGS: ${CMAKE_CUDA_FLAGS}")
message(STATUS "CMAKE CUDA HOST Compiler: ${CMAKE_CUDA_HOST_COMPILER}")

set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
set(Torch_DIR ${PROJECT_SOURCE_DIR}/3rdParty/libtorch/share/cmake/Torch)
set(fmt_DIR ${PROJECT_SOURCE_DIR}/3rdParty/fmt/build)

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(FMT REQUIRED)
find_package(Torch REQUIRED)

set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.1/bin/nvcc")
# Set the C++ flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(SUN3D_GTOptimizer_Lib SHARED
    src/ThreadPool.cc
    src/Dataloader.cc
    src/FrameExtractor.cc
    src/SIFTextractor.cc
    src/BundleAdjustment.cc
    src/SuperPointInference.cc
    src/ICP_BAsolver.cc
)

include(CTest)
enable_testing()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable(SUN3DGroundtruthOptimizer src/GroundtruthOptimizer.cc)

set(G2O_BUILD_PATH ${PROJECT_SOURCE_DIR}/3rdParty/g2o/build)

include_directories(
  "${TORCH_INCLUDE_DIRS}"
)

target_include_directories(SUN3D_GTOptimizer_Lib PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/3rdParty/Sophus
    ${G2O_BUILD_PATH} 
    ${PROJECT_SOURCE_DIR}/3rdParty/g2o
    ${EIGEN3_INCLUDE_DIR}
)

# set(CMAKE_VERBOSE_MAKEFILE ON)

target_link_libraries(SUN3D_GTOptimizer_Lib PRIVATE 
    fmt::fmt-header-only
    )

target_link_libraries(SUN3D_GTOptimizer_Lib PUBLIC  
    ${OpenCV_LIBS}
    "${G2O_BUILD_PATH}/lib/libg2o_core.so"
    "${G2O_BUILD_PATH}/lib/libg2o_stuff.so"
    "${G2O_BUILD_PATH}/lib/libg2o_types_sba.so"
    "${G2O_BUILD_PATH}/lib/libg2o_solver_dense.so"
    "${G2O_BUILD_PATH}/lib/libg2o_solver_csparse.so"
    "${G2O_BUILD_PATH}/lib/libg2o_csparse_extension.so"
    ${TORCH_LIBRARIES}
    )

target_link_libraries(SUN3DGroundtruthOptimizer SUN3D_GTOptimizer_Lib ${TORCH_LIBRARIES} ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

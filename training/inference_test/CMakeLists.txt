cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CUDA_ARCHITECTURES 52 60 61 75 80 89)

project(InferenceTest VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CUDA_HOST_COMPILER "${CMAKE_CXX_COMPILER}")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -ccbin ${CMAKE_CXX_COMPILER}")

message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "CMAKE CUDA FLAGS: ${CMAKE_CUDA_FLAGS}")
message(STATUS "CMAKE CUDA HOST Compiler: ${CMAKE_CUDA_HOST_COMPILER}")

set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
set(Torch_DIR ${PROJECT_SOURCE_DIR}/3rdParty/libtorch/share/cmake/Torch)
set(DBoW3_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/3rdParty/DBoW2/DBoW2" )
set(DBoW3_LIBS "${PROJECT_SOURCE_DIR}/3rdParty/DBoW2/lib/libDBoW2.so" )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)

set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.1/bin/nvcc")
# Set the C++ flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(Inference_Test_Lib SHARED
    src/Inference.cc
)

include(CTest)
enable_testing()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable(InferenceTest src/match.cc)
add_executable(DBoWCreation src/create_dbow.cc)


include_directories(
  "${TORCH_INCLUDE_DIRS}"
)

target_include_directories(Inference_Test_Lib PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${DBoW3_INCLUDE_DIRS}
)


target_link_libraries(InferenceTest Inference_Test_Lib ${TORCH_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(DBoWCreation Inference_Test_Lib ${TORCH_LIBRARIES} ${OpenCV_LIBS} ${DBoW3_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
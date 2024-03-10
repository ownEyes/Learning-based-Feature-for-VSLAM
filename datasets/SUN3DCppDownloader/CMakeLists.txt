cmake_minimum_required(VERSION 3.5)

project(SUN3D_Downloader)
set(CMAKE_CXX_STANDARD 17)

find_package(CURL)

include_directories(
	${CURL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(SUN3D_Downloader_Lib SHARED
    src/ThreadPool.cc
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(SUN3D_Downloader
	src/SUN3D.cc)

target_link_libraries(SUN3D_Downloader SUN3D_Downloader_Lib  ${CURL_LIBRARIES})


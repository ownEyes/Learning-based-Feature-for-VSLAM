cmake_minimum_required(VERSION 3.5)

project(SUN3DCppDownloader)
set(CMAKE_CXX_STANDARD 17)

find_package(CURL)

add_executable(SUN3DCppDownloader
	SUN3D.cc)

target_link_libraries(SUN3DCppDownloader ${CURL_LIBRARIES})
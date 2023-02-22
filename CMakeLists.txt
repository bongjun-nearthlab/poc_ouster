cmake_minimum_required(VERSION 3.11)

project(ouster)
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(CURL REQUIRED)
find_package(spdlog REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS CXX)
find_package(MPI REQUIRED)

include_directories(ouster
    ${HDF5_INCLUDE_DIRS}
)

add_executable(ouster 
    ${CMAKE_SOURCE_DIR}/src/ouster_main.cpp
)

target_include_directories(ouster PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/optional-lite
)

target_link_libraries(ouster
    PUBLIC Eigen3::Eigen
    PRIVATE CURL::libcurl jsoncpp_lib spdlog::spdlog
    PRIVATE pthread
	PRIVATE ${MPI_LIBRARIES}
	PRIVATE ${HDF5_LIBRARIES}
	PRIVATE ${HDF5_CXX_LIBRARIES}
)
target_compile_definitions(ouster 
    PRIVATE EIGEN_MPL2_ONLY
)

target_link_libraries(ouster
    PUBLIC ${CMAKE_SOURCE_DIR}/lib/ouster/libouster_client.so
)
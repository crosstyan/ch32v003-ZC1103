cmake_minimum_required(VERSION 3.20)
project(se_de_test)

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

set(GIT_DIR_LOOKUP_POLICY ALLOW_LOOKING_ABOVE_CMAKE_SOURCE_DIR)
set(ETL_DIR ${ROOT_DIR}/libs/etl)
add_subdirectory(${ETL_DIR} etl)
include_directories(${ETL_DIR}/include)

set(CNL_DIRECTORY ${ROOT_DIR}/libs/cnl)
include_directories(${CNL_DIRECTORY}/include)

include_directories(include)

add_executable(${PROJECT_NAME} main.cpp)
target_compile_features(${CMAKE_PROJECT_NAME} PRIVATE cxx_std_23)
target_link_libraries(${CMAKE_PROJECT_NAME}
        PRIVATE
        etl::etl)

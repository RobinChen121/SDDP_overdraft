cmake_minimum_required(VERSION 3.30)
project(Overdraft)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "/O2")
else ()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif ()

add_executable(${PROJECT_NAME}
        sdp/single_product.cpp
        sdp/cash_leadtime_state.h
        sdp/pmf.cpp
        sdp/pmf.h
)

#-----------------------------------
# Boost library
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
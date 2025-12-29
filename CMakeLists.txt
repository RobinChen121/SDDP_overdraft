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
        #        sdp/single_product_sdp_testing.cpp
        #        sdp/two_product_sdp.cpp
        #        #        sdp/single_product_sdp.cpp
        #        sdp/cash_leadtime_state.h

        sdp/pmf.cpp

        sddp/single_product_testing.cpp
        sddp/sampling.cpp

        common.cpp
)

#-----------------------------------
# Boost library
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

### Gurobi library
if (APPLE)
    set(GUROBI_HOME "/Library/gurobi1201/macos_universal2")
    set(GUROBI_INCLUDE_DIR "${GUROBI_HOME}/include")
    set(GUROBI_LIB_DIR "${GUROBI_HOME}/lib")

    include_directories(${GUROBI_INCLUDE_DIR})
    include_directories(${GUROBI_LIB_DIR})
    target_link_libraries(${PROJECT_NAME}
            "${GUROBI_LIB_DIR}/libgurobi_c++.a"
            "${GUROBI_LIB_DIR}/libgurobi120.dylib"
    )
else ()
    option(CXX "enable C++ compilation" ON)
    if (CXX)
        enable_language(CXX)
    endif ()
    set(CMAKE_MODULE_PATH "C:/gurobi1202/win64")
    set(GUROBI_DIR "C:/gurobi1202/win64")
    set(ENV{GUROBI_HOME} "${GUROBI_DIR}")

    message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

    # Visual Studio compiler with static runtime libraries
    if (MSVC AND MT)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
        set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
    endif ()
    find_package(GUROBI 120 REQUIRED)
    include_directories(${GUROBI_INCLUDE_DIRS})
    include_directories(${GUROBI_CXX_LIBRARY})
    message(STATUS "Gurobi include dirs: ${GUROBI_INCLUDE_DIRS}")
    message(STATUS "Gurobi libraries: ${GUROBI_CXX_LIBRARY}")

    if (CXX)
        target_link_libraries(${RUN_NAME} PRIVATE optimized ${GUROBI_CXX_LIBRARY}
                debug ${GUROBI_CXX_DEBUG_LIBRARY})
    endif ()

    target_link_libraries(${RUN_NAME} PRIVATE ${GUROBI_LIBRARY})

    if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
        include(FeatureSummary)
        feature_summary(WHAT ALL)
    endif ()
endif ()
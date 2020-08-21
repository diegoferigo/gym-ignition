# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_subdirectory(thrift)

# ============
# ScenarioYarp
# ============

find_package(iDynTree REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

set(SCENARIO_YARP_PUBLIC_HDRS
    include/scenario/yarp/World.h
    include/scenario/yarp/Model.h
    include/scenario/yarp/Joint.h
    include/scenario/yarp/Link.h
    include/scenario/yarp/utils.h
    )

add_library(ScenarioYarp
    ${SCENARIO_YARP_PUBLIC_HDRS}
    include/scenario/yarp/Log.h
    include/scenario/yarp/helpers.h
    include/scenario/yarp/YarpRobots.h
    include/scenario/yarp/YarpInterfaces.h
    src/World.cpp
    src/Model.cpp
    src/Joint.cpp
    src/Link.cpp
    src/utils.cpp
    src/helpers.cpp
    src/YarpRobots.cpp
    src/YarpInterfaces.cpp
    )

target_include_directories(ScenarioYarp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(ScenarioYarp
    PUBLIC
    ScenarioBase
    PRIVATE
    ClockRpc
    Eigen3::Eigen
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_sig
#    YARP::YARP_init
    YARP::YARP_robotinterface
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-modelio-urdf
    iDynTree::idyntree-high-level
    )

set_target_properties(ScenarioYarp PROPERTIES
    PUBLIC_HEADER "${SCENARIO_YARP_PUBLIC_HDRS}")

# ===================
# Install the targets
# ===================

if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS
        ScenarioYarp
        EXPORT scenario
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/scenario/yarp)
endif()

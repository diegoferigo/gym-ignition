# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(YARP 3.0.0 REQUIRED)

# TODO: sooner or later this file will be stored in YARP
yarp_add_idl(CLOCK_RPC_FILES clock_rpc.thrift)

# Add the library target
add_library(ClockRpc ${CLOCK_RPC_FILES} clock_rpc.thrift)

# Link it with YARP
find_package(YARP 3.0.0 REQUIRED)
target_link_libraries(ClockRpc YARP::YARP_OS)
target_include_directories(ClockRpc PUBLIC ${YARP_ADD_IDL_INCLUDE_DIR})

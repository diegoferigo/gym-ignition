# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
#  This project is dual licensed under LGPL v2.1+ or Apache License.
#
# -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
#
#  This software may be modified and distributed under the terms of the
#  GNU Lesser General Public License v2.1 or any later version.
#
# -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

add_library(ActuationDelay SHARED
    ActuationDelay.h
    ActuationDelay.cpp)

target_link_libraries(ActuationDelay
    PUBLIC
    ignition-gazebo3::core
    PRIVATE
#    ExtraComponents
    ScenarioGazebo)

target_include_directories(ActuationDelay PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

# TODO: install with bindings in PyPI mode
if(NOT CMAKE_BUILD_TYPE STREQUAL "PyPI")
    install(
        TARGETS ActuationDelay
        EXPORT scenario
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/scenario/plugins
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/scenario/plugins
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/scenario/plugins)
endif()

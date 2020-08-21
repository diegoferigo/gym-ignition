/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_YARP_UTILS_H
#define SCENARIO_YARP_UTILS_H

#include <memory>
#include <string>

namespace scenario::yarp::utils {
    class GazeboClassicRunner;
    // hp: robot name set from the user
    std::string FindYarpResource(const std::string& resource);
} // namespace scenario::yarp::utils

class scenario::yarp::utils::GazeboClassicRunner
{
public:
    GazeboClassicRunner(const std::string& serverPortName);
    ~GazeboClassicRunner();

    bool initialize();
    bool step(const double dt);
    bool pause(const bool paused);
    bool resetTime();

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_YARP_UTILS_H

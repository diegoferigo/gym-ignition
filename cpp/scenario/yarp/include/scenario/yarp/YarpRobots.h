/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_YARP_YARPROBOTS_H
#define SCENARIO_YARP_YARPROBOTS_H

#include <memory>
#include <string>
#include <vector>

namespace scenario::yarp {
    class YarpRobots;
} // namespace scenario::yarp

namespace yarp::robotinterface {
    class Robot;
} // namespace yarp::robotinterface

class scenario::yarp::YarpRobots
{
public:
    YarpRobots();
    ~YarpRobots();

    bool loadXml(const std::string& xml, const bool join = true);
    const std::vector<::yarp::robotinterface::Robot>& getRobots();

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_YARP_YARPROBOTS_H

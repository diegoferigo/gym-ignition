/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/yarp/utils.h"
#include "ClockServer.h"
#include "scenario/yarp/Log.h"

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>

#include <iostream>

using namespace scenario::yarp::utils;

std::string scenario::yarp::utils::FindYarpResource(const std::string& resource)
{
    return ::yarp::os::ResourceFinder::getResourceFinderSingleton()
        .findFileByName(resource);
}

class GazeboClassicRunner::Impl
{
public:
    //    ::yarp::os::Network network;
    std::unique_ptr<::yarp::os::Network> network = nullptr;

    double stepSize;
    std::string serverPortName;
    ::yarp::os::Port clientPort;
    GazeboYarpPlugins::ClockServer clockServer;
};

scenario::yarp::utils::GazeboClassicRunner::GazeboClassicRunner(
    const std::string& serverPortName)
    : pImpl{std::make_unique<Impl>()}
{
    pImpl->serverPortName = serverPortName;
}

GazeboClassicRunner::~GazeboClassicRunner()
{
    if (pImpl->clientPort.isOpen()) {
        // Finish the simulation if it was pending
        // pImpl->clockServer.continueSimulation();

        if (!::yarp::os::Network::disconnect(pImpl->clientPort.getName(),
                                             pImpl->serverPortName)) {
            sError
                << "Failed to disconnect the clock server from the simulator";
        }

        pImpl->clientPort.close();
    }
}

bool GazeboClassicRunner::initialize()
{
    pImpl->network = std::make_unique<::yarp::os::Network>();

    if (!::yarp::os::Network::initialized()
        || !::yarp::os::Network::checkNetwork(5.0)) {
        sError << "Failed to initialize YARP network";
        return false;
    }

    // Connect the ports on first run
    if (!(pImpl->clientPort.open("...")
          && ::yarp::os::Network::connect(pImpl->clientPort.getName(),
                                          pImpl->serverPortName))) {
        sError << "Error connecting to the simulator clock server";
        return false;
    }

    // Configure the Wire object from which ClockServer inherits
    if (!pImpl->clockServer.yarp().attachAsClient(pImpl->clientPort)) {
        sError << "Failed to attach the local port to the ClockServer";
        return false;
    }

    if (!pImpl->clockServer.yarp().isValid()) {
        sError << "The ClockServer is not valid";
        return false;
    }

    // Get the physics step size
    pImpl->stepSize = pImpl->clockServer.getStepSize();
    sDebug << "Detected physics step size of" << pImpl->stepSize;

    return true;
}

bool GazeboClassicRunner::step(const double dt)
{
    const double steps = dt / pImpl->stepSize;
    const std::int32_t stepsInt = steps;

    if (std::abs(steps - stepsInt) > 1E-8) {
        sWarning << "Rounding dt to" << pImpl->stepSize * stepsInt;
    }

    // Step the simulator
    //    sDebug << "Executing" << stepsInt << "steps";
    pImpl->clockServer.stepSimulationAndWait(stepsInt);

    return true;
}

bool GazeboClassicRunner::pause(const bool paused)
{
    if (paused) {
        pImpl->clockServer.pauseSimulation();
    }
    else {
        pImpl->clockServer.continueSimulation();
    }

    return true;
}

bool GazeboClassicRunner::resetTime()
{
    pImpl->clockServer.resetSimulationTime();
    return true;
}

/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/yarp/YarpRobots.h"
#include "scenario/yarp/Log.h"
#include "scenario/yarp/utils.h"

#include <cassert>
#include <unordered_map>
#include <yarp/robotinterface/Robot.h>
#include <yarp/robotinterface/XMLReader.h>

using namespace scenario::yarp;

class YarpRobots::Impl
{
public:
    using DeviceName = std::string;
    using Robot = ::yarp::robotinterface::Robot;
    using Device = ::yarp::robotinterface::Device;

    std::vector<Robot> robots;
    std::unordered_map<DeviceName, Device*> nameToDevice;

    ::yarp::robotinterface::DevicePtrList allDevices() const
    {
        ::yarp::robotinterface::DevicePtrList devicePtrs;
        devicePtrs.reserve(nameToDevice.size());

        for (auto& [_, ptr] : nameToDevice) {
            assert(ptr);
            sWarning << ptr->name();
            devicePtrs.push_back(ptr);
        }

        return devicePtrs;
    }
};

YarpRobots::YarpRobots()
    : pImpl{std::make_unique<Impl>()}
{}

YarpRobots::~YarpRobots()
{
    using namespace ::yarp::robotinterface;
    for (auto& robot : pImpl->robots) {
        if (!robot.enterPhase(ActionPhaseInterrupt1)) {
            sWarning << "Failed to enter phase ActionPhaseInterrupt1";
            continue;
        }

        if (!robot.enterPhase(ActionPhaseShutdown)) {
            sWarning << "Failed to enter phase ActionPhaseShutdown";
            continue;
        }
    }
}

bool YarpRobots::loadXml(const std::string& xml, const bool join)
{
    using namespace ::yarp::robotinterface;

    const std::string xmlAbsPath = utils::FindYarpResource(xml);

    if (xmlAbsPath.empty()) {
        sError << "Failed to find file" << xml;
        return false;
    }

    XMLReader reader;
    XMLReaderResult result = reader.getRobotFromFile(xmlAbsPath);

    if (!result.parsingIsSuccessful) {
        sError << "Failed to parse xml file" << xml;
        return false;
    }

    // Get all the existing devices
    auto allDevices = join ? pImpl->allDevices() : DevicePtrList();
    sError << allDevices.size();

    // Open and attach the devices
    if (!result.robot.enterPhase(ActionPhaseStartup, allDevices)) {
        sError << "Failed to enter startup phase";
        return false;
    }

    pImpl->robots.push_back(std::move(result.robot));

    for (auto* device : pImpl->robots.back().devicePtrs()) {
        assert(device);
        if (!device->isOpen()) {
            assert(join);
            sDebug << "Device" << device->name() << "has not been opened";
            continue;
        }

        if (join) {
            assert(pImpl->nameToDevice.find(device->name())
                   == pImpl->nameToDevice.end());
            pImpl->nameToDevice[device->name()] = device;
            sDebug << "Created new device" << device->name();
            continue;
        }

        if (pImpl->nameToDevice.find(device->name())
            != pImpl->nameToDevice.end()) {
            sDebug << "Device" << device->name()
                   << "already exists and has not been joined";
            continue;
        }

        sDebug << "Created new device" << device->name();
        pImpl->nameToDevice[device->name()] = device;
    }

    sError << "returning loadXml";
    return true;
}

const std::vector<yarp::robotinterface::Robot>& YarpRobots::getRobots()
{
    return pImpl->robots;
}

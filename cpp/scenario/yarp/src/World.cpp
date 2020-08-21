/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/yarp/World.h"
#include "scenario/yarp/Log.h"
#include "scenario/yarp/Model.h"

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

using namespace scenario::yarp;

class World::Impl
{
public:
    ::yarp::os::Network network;

    using ModelName = std::string;
    std::unordered_map<ModelName, std::shared_ptr<Model>> models;
};

World::World()
    : pImpl{std::make_unique<Impl>()}
{}

World::~World() = default;

bool World::initialize()
{
    if (!::yarp::os::Network::initialized()
        || !::yarp::os::Network::checkNetwork(5.0)) {
        sError << "YARP server wasn't found active";
        return false;
    }

    if (::yarp::os::Time::isSystemClock()) {
        sDebug << "Using system clock";
    }

    if (::yarp::os::Time::isNetworkClock()) {
        sDebug << "Using network clock";
    }

    return true;
}

bool World::insertModel(const std::string& name,
                        const std::string& urdfFile,
                        const std::vector<std::string>& xmls,
                        const std::string& rcbrDeviceName)
{
    if (pImpl->models.find(name) != pImpl->models.end()) {
        sError << "Model" << name << "already exists";
        return false;
    }

    auto model = std::make_shared<Model>();

    if (!model->initializeURDF(urdfFile)) {
        sError << "Failed to initialize URDF file";
        return false;
    }

    if (xmls.empty()) {
        sError << "You need to pass at least the RCBR xml";
        return false;
    }

    // NOTE: hardcoded first entry rcbr
    // TODO: use the rcbrDeviceName to extract the device and pass it to
    //       YarpInterfaces
    if (!model->initializeRCBR(xmls[0])) {
        sError << "Failed to initialize the RCBR";
        return false;
    }

    // Load all the others
    // NOTE: for the moment we do not join any device
    for (size_t i = 1; i < xmls.size(); ++i) {
        if (!model->launchYarpRobotInterfaceXml(xmls[i], false)) {
            sError << "Failed to launch xml" << xmls[i];
            return false;
        }
    }

    // Initialize the model
    if (!model->initialize()) {
        sError << "Failed to initialize the model";
        return false;
    }

    pImpl->models[name] = model;
    return true;
}

// bool World::registerModel() {}

double World::time() const
{
    return ::yarp::os::Time::now();
}

std::string World::name() const
{
    return "default";
}

std::array<double, 3> World::gravity() const
{
    return {0, 0, -9.80};
}

std::vector<std::string> World::modelNames() const
{
    std::vector<std::string> names;
    names.reserve(pImpl->models.size());

    for (const auto& [name, _] : pImpl->models) {
        names.push_back(name);
    }

    return names;
}

scenario::base::ModelPtr World::getModel(const std::string& modelName) const
{
    if (pImpl->models.find(modelName) == pImpl->models.end()) {
        sError << "Model" << modelName << "does not exist";
        return nullptr;
    }

    return pImpl->models.at(modelName);
}

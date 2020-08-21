/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_YARP_WORLD_H
#define SCENARIO_YARP_WORLD_H

#include "scenario/base/World.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario::yarp {
    class World;
}

class scenario::yarp::World : public scenario::base::World,
                              public std::enable_shared_from_this<World> // TODO
{
public:
    World();
    ~World() override;

    // ==========
    // YARP World
    // ==========

    bool initialize();
    bool insertModel(const std::string& name,
                     const std::string& urdfFile,
                     const std::vector<std::string>& xmls,
                     const std::string& rcbrDevice);

    // ==========
    // World Core
    // ==========

    double time() const override;

    std::string name() const override;

    std::array<double, 3> gravity() const override;

    std::vector<std::string> modelNames() const override;

    scenario::base::ModelPtr
    getModel(const std::string& modelName) const override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_YARP_WORLD_H

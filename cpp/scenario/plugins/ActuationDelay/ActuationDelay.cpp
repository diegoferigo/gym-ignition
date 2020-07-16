/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This project is dual licensed under LGPL v2.1+ or Apache License.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ActuationDelay.h"
#include "scenario/gazebo/Joint.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/helpers.h"

#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

using namespace scenario::gazebo;
using namespace scenario::plugins::gazebo;

class ActuationDelay::Impl
{
public:
    size_t nrOfDelaySteps;
    scenario::gazebo::ModelPtr model;
    ignition::gazebo::Entity modelEntity;

    using LinkName = std::string;
    using JointForceCmdData = std::vector<double>;
    std::unordered_map<LinkName, std::deque<JointForceCmdData>> queues;
};

ActuationDelay::ActuationDelay()
    : System()
    , pImpl{std::make_unique<Impl>()}
{}

ActuationDelay::~ActuationDelay() = default;

void ActuationDelay::Configure(const ignition::gazebo::Entity& entity,
                               const std::shared_ptr<const sdf::Element>& sdf,
                               ignition::gazebo::EntityComponentManager& ecm,
                               ignition::gazebo::EventManager& eventMgr)
{
    // Store the model entity
    pImpl->modelEntity = entity;

    // Create a model that will be given to the controller
    pImpl->model = std::make_shared<Model>();

    // Create a model and check its validity
    if (!pImpl->model->initialize(entity, &ecm, &eventMgr)) {
        sError << "Failed to initialize model for controller" << std::endl;
        return;
    }

    if (!pImpl->model->valid()) {
        sError << "Failed to create a model from Entity [" << entity << "]"
               << std::endl;
        return;
    }

    if (utils::verboseFromEnvironment()) {
        sDebug << "Received SDF context:";
        std::cout << sdf->ToString("") << std::endl;
    }

    // This is the <plugin> element (with extra options stored in its children)
    sdf::ElementPtr pluginElement = sdf->Clone();

    // Check if it contains extra options stored in a <controller> child
    if (!pluginElement->HasElement("delay")) {
        sError << "Failed to find element 'delay' in the plugin's sdf context";
        return;
    }

    if (!pluginElement->Get<size_t>("delay", pImpl->nrOfDelaySteps, 0)) {
        sError << "Failed to get the 'delay' element";
        return;
    }

    sDebug << "Enabled the delay of joint force actuation for "
           << pImpl->nrOfDelaySteps << " physics steps" << std::endl;

    // Create the queues for all joints. Then, only the joints that use
    // JointForceCmd are actually processed.
    for (const auto& joint : pImpl->model->joints()) {
        pImpl->queues[joint->name()] = {};
        pImpl->queues[joint->name()].resize(
            pImpl->nrOfDelaySteps, Impl::JointForceCmdData(joint->dofs(), 0.0));
    }
}

void ActuationDelay::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                               ignition::gazebo::EntityComponentManager& ecm)
{
    if (pImpl->nrOfDelaySteps == 0) {
        return;
    }

    if (info.paused) {
        return;
    }

    if (!pImpl->model) {
        return;
    }

    // This plugin keep being called also after the model was removed
    try {
        pImpl->model->controllerPeriod();
    }
    catch (exceptions::ComponentNotFound) {
        return;
    }

    ecm.Each<ignition::gazebo::components::Joint,
             ignition::gazebo::components::Name,
             ignition::gazebo::components::JointForceCmd,
             ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity& /*entity*/,
            ignition::gazebo::components::Joint* /*jointComponent*/,
            ignition::gazebo::components::Name* nameComponent,
            ignition::gazebo::components::JointForceCmd* jointForceCmdComponent,
            ignition::gazebo::components::ParentEntity* parentEntityComponent)
            -> bool {
            if (parentEntityComponent->Data() != pImpl->modelEntity) {
                return true;
            }

            const std::string& name = nameComponent->Data();
            const std::vector<double>& forceCmd =
                jointForceCmdComponent->Data();

            auto& queue = pImpl->queues[name];
            queue.push_back(forceCmd);

            auto joint = pImpl->model->getJoint(name);
            if (!joint->setJointGeneralizedForceTarget(queue.front())) {
                sError << "Failed to set the force of joint " << name
                       << std::endl;
            }

            queue.pop_front();
            return true;
        });
}

IGNITION_ADD_PLUGIN(scenario::plugins::gazebo::ActuationDelay,
                    scenario::plugins::gazebo::ActuationDelay::System,
                    scenario::plugins::gazebo::ActuationDelay::ISystemConfigure,
                    scenario::plugins::gazebo::ActuationDelay::ISystemPreUpdate)

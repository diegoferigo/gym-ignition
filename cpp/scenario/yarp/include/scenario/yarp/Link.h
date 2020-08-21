/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_YARP_LINK_H
#define SCENARIO_YARP_LINK_H

#include "scenario/base/Link.h"

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace scenario::yarp {
    class Link;
    class YarpInterfaces;
} // namespace scenario::yarp

namespace scenario::yarp::utils {
    class FloatingBaseEstimator;
} // namespace scenario::yarp::utils

namespace iDynTree {
    class KinDynComputations;
} // namespace iDynTree

class scenario::yarp::Link : public scenario::base::Link
{
public:
    Link();
    ~Link() override;

    // =========
    // YARP Link
    // =========

    bool initialize(
        const std::shared_ptr<const base::Model> parentModel,
        const std::string& linkName,
        const std::shared_ptr<YarpInterfaces>& yarpInterfaces,
        const std::shared_ptr<iDynTree::KinDynComputations>& kinDynComputations,
        const std::shared_ptr<utils::FloatingBaseEstimator>& fbe);

    /////////////////

    std::string name(const bool scoped = false) const override;

    double mass() const override;

    std::array<double, 3> position() const override;

    std::array<double, 4> orientation() const override;

    std::array<double, 3> worldLinearVelocity() const override;
    std::array<double, 3> worldAngularVelocity() const override;
    std::array<double, 3> bodyLinearVelocity() const override;
    std::array<double, 3> bodyAngularVelocity() const override;

    std::array<double, 3> worldLinearAcceleration() const override;
    std::array<double, 3> worldAngularAcceleration() const override;
    std::array<double, 3> bodyLinearAcceleration() const override;
    std::array<double, 3> bodyAngularAcceleration() const override;

    bool contactsEnabled() const override;

    bool enableContactDetection(const bool enable) override;

    bool inContact() const override; // TODO: add optional "other" link?

    std::vector<scenario::base::Contact> contacts() const override;

    std::array<double, 6> contactWrench() const override;

    bool applyWorldForce(const std::array<double, 3>& force,
                         const double duration = 0.0) override;

    bool applyWorldTorque(const std::array<double, 3>& torque,
                          const double duration = 0.0) override;

    bool applyWorldWrench(const std::array<double, 3>& force,
                          const std::array<double, 3>& torque,
                          const double duration = 0.0) override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_YARP_LINK_H

/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_YARP_JOINT_H
#define SCENARIO_YARP_JOINT_H

#include "scenario/base/Joint.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace scenario::base {
    class Model;
}

namespace scenario::yarp {
    class Joint;
    class YarpInterfaces;
} // namespace scenario::yarp

namespace iDynTree {
    class KinDynComputations;
} // namespace iDynTree

class scenario::yarp::Joint : public scenario::base::Joint
{
public:
    Joint();
    ~Joint() override;

    // ==========
    // YARP Joint
    // ==========

    bool initialize(const std::shared_ptr<const base::Model> parentModel,
                    const int jointAxis,
                    const std::shared_ptr<YarpInterfaces>& yarpInterfaces,
                    const std::shared_ptr<iDynTree::KinDynComputations>&
                        kinDynComputations);

    /////////////

    bool historyOfAppliedJointForcesEnabled() const override;

    bool enableHistoryOfAppliedJointForces( //
        const bool enable = true,
        const size_t maxHistorySize = 100) override;

    std::vector<double> historyOfAppliedJointForces() const override;

    // ==========
    // Joint Core
    // ==========

    size_t dofs() const override;

    std::string name(const bool scoped = false) const override;

    scenario::base::JointType type() const override;

    scenario::base::JointControlMode controlMode() const override;

    bool setControlMode(const scenario::base::JointControlMode mode) override;

    double controllerPeriod() const override;

    scenario::base::PID pid() const override;

    bool setPID(const scenario::base::PID& pid) override;

    // ==================
    // Single DOF methods
    // ==================

    scenario::base::Limit positionLimit(const size_t dof = 0) const override;

    double maxGeneralizedForce(const size_t dof = 0) const override;

    bool setMaxGeneralizedForce(const double maxForce,
                                const size_t dof = 0) override;

    double position(const size_t dof = 0) const override;

    double velocity(const size_t dof = 0) const override;
    //    double acceleration(const size_t dof = 0) const;

    bool setPositionTarget(const double position,
                           const size_t dof = 0) override;

    bool setVelocityTarget(const double velocity,
                           const size_t dof = 0) override;

    bool setAccelerationTarget(const double acceleration,
                               const size_t dof = 0) override;

    bool setGeneralizedForceTarget(const double force,
                                   const size_t dof = 0) override;

    double positionTarget(const size_t dof = 0) const override;

    double velocityTarget(const size_t dof = 0) const override;

    double accelerationTarget(const size_t dof = 0) const override;

    double generalizedForceTarget(const size_t dof = 0) const override;

    bool resetPosition(const double position = 0,
                       const size_t dof = 0) override;

    bool resetVelocity(const double velocity = 0,
                       const size_t dof = 0) override;

    bool reset(const double position = 0,
               const double velocity = 0,
               const size_t dof = 0) override;

    // =================
    // Multi DOF methods
    // =================

    scenario::base::JointLimit jointPositionLimit() const override;

    std::vector<double> jointMaxGeneralizedForce() const override;

    bool
    setJointMaxGeneralizedForce(const std::vector<double>& maxForce) override;

    std::vector<double> jointPosition() const override;

    std::vector<double> jointVelocity() const override;
    //    std::vector<double> jointAcceleration() const;

    bool setJointPositionTarget(const std::vector<double>& position) override;

    bool setJointVelocityTarget(const std::vector<double>& velocity) override;

    bool setJointAccelerationTarget(
        const std::vector<double>& acceleration) override;

    bool
    setJointGeneralizedForceTarget(const std::vector<double>& force) override;

    std::vector<double> jointPositionTarget() const override;

    std::vector<double> jointVelocityTarget() const override;

    std::vector<double> jointAccelerationTarget() const override;

    std::vector<double> jointGeneralizedForceTarget() const override;

    bool resetJointPosition(const std::vector<double>& position = {0}) override;

    bool resetJointVelocity(const std::vector<double>& velocity = {0}) override;

    bool resetJoint(const std::vector<double>& position = {0},
                    const std::vector<double>& velocity = {0}) override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_YARP_JOINT_H

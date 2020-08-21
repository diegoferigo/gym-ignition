/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_YARP_MODEL_H
#define SCENARIO_YARP_MODEL_H

#include "scenario/base/Model.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario::yarp {
    class Link;
    class Model;
} // namespace scenario::yarp

// TODO: scenario::base::FloatingBase
class scenario::yarp::Model
    : public scenario::base::Model
    //    , public std::enable_shared_from_this<base::Model>
    , public std::enable_shared_from_this<Model>
{
public:
    Model();
    ~Model() override;

    // ==========
    // YARP Model
    // ==========

    bool initializeURDF(const std::string& urdfFile);
    bool initializeRCBR(const std::string& xml);

    bool initialize(
        //        const std::string& robotName,
        //                    const std::string& urdfFile,
        //                    const std::vector<std::string>& controlBoardNames
        //                    //,
        // const std::vector<std::string>& controlledJoints = {}
    );

    // qualcosa che si attacca al RCBR
    // TODO: attachDevice()

    // nel world invece attachDevice() normale

    // un po' come plugins di gazebo

    // puo' essere un modo per usare nostro urdf anche per YARP
    // Aggiungere anche device di sensori in questo modo
    // Troppo casino con gli XML e i gains? passare l'xml / ini installato da
    // superbuild come workaround? utils::FindYarpResource(yarpRobotName = "")?

    // TODO static?
    //    bool loadYarpDevice(const std::string& ini = "");
    //    std::vector<std::string>
    bool launchYarpRobotInterfaceXml(const std::string& xml,
                                     const bool join = true);

    // TODO
    // ============
    // Gazebo Model
    // ============

    bool historyOfAppliedJointForcesEnabled(
        const std::vector<std::string>& jointNames = {}) const override;
    bool enableHistoryOfAppliedJointForces( //
        const bool enable = true,
        const size_t maxHistorySizePerJoint = 100,
        const std::vector<std::string>& jointNames = {}) override;
    std::vector<double> historyOfAppliedJointForces(
        const std::vector<std::string>& jointNames = {}) const override;

    // ==========
    // Model Core
    // ==========

    bool valid() const override;

    /// \overload void scenario::base::Model::name
    size_t dofs(const std::vector<std::string>& jointNames = {}) const override;

    /**
     * @overload void scenario::base::Model::name
     */
    std::string name() const override;

    size_t nrOfLinks() const override;

    size_t nrOfJoints() const override;

    double
    totalMass(const std::vector<std::string>& linkNames = {}) const override;

    scenario::base::LinkPtr getLink(const std::string& linkName) const override;

    scenario::base::JointPtr
    getJoint(const std::string& jointName) const override;

    std::vector<std::string>
    linkNames(const bool scoped = false) const override;

    std::vector<std::string>
    jointNames(const bool scoped = false) const override;

    double controllerPeriod() const override;

    bool setControllerPeriod(const double period) override;

    // ========
    // Contacts
    // ========

    bool contactsEnabled() const override;

    bool enableContacts(const bool enable = true) override;

    bool selfCollisionsEnabled() const override;

    bool enableSelfCollisions(const bool enable = true) override;

    std::vector<std::string> linksInContact() const override;

    std::vector<scenario::base::Contact>
    contacts(const std::vector<std::string>& linkNames = {}) const override;

    // ==================
    // Vectorized Methods
    // ==================

    std::vector<double> jointPositions( //
        const std::vector<std::string>& jointNames = {}) const override;

    std::vector<double> jointVelocities( //
        const std::vector<std::string>& jointNames = {}) const override;

    //    std::vector<double> jointAccelerations(const std::vector<std::string>&
    //    jointNames = {}) const;

    scenario::base::JointLimit jointLimits( //
        const std::vector<std::string>& jointNames = {}) const override;

    bool setJointControlMode(
        const scenario::base::JointControlMode mode,
        const std::vector<std::string>& jointNames = {}) override;

    std::vector<scenario::base::LinkPtr> links( //
        const std::vector<std::string>& linkNames = {}) const override;

    std::vector<scenario::base::JointPtr> joints( //
        const std::vector<std::string>& jointNames = {}) const override;

    // =========================
    // Vectorized Target Methods
    // =========================

    bool setJointPositionTargets( //
        const std::vector<double>& positions,
        const std::vector<std::string>& jointNames = {}) override;

    bool setJointVelocityTargets( //
        const std::vector<double>& velocities,
        const std::vector<std::string>& jointNames = {}) override;

    bool setJointAccelerationTargets( //
        const std::vector<double>& accelerations,
        const std::vector<std::string>& jointNames = {}) override;

    bool setJointGeneralizedForceTargets( //
        const std::vector<double>& forces,
        const std::vector<std::string>& jointNames = {}) override;

    bool resetJointPositions( //
        const std::vector<double>& positions,
        const std::vector<std::string>& jointNames = {}) override;

    bool resetJointVelocities( //
        const std::vector<double>& velocities,
        const std::vector<std::string>& jointNames = {}) override;

    std::vector<double> jointPositionTargets( //
        const std::vector<std::string>& jointNames = {}) const override;
    std::vector<double> jointVelocityTargets( //
        const std::vector<std::string>& jointNames = {}) const override;
    std::vector<double> jointAccelerationTargets( //
        const std::vector<std::string>& jointNames = {}) const override;
    // This differs from the others... Should I return the Cmd?
    // Should we also remove JointForce component since it is wrong?
    std::vector<double> jointGeneralizedForceTargets( //
        const std::vector<std::string>& jointNames = {}) const override;

    // TODO: Pensare ad iCub vero, come instanziare un controllore?
    //       Su un thread separato che si fa la view dell'interfaccia Model?

    // =========
    // Base Link
    // =========

    std::string baseFrame() const override;
    //    bool setBaseFrame(const std::string& frameName) override; // TODO
    //    remove?

    // TODO: remove these?
    //    bool fixedBase() const override;
    //    bool setAsFixedBase(const bool fixedBase = true) override;

    std::array<double, 3> basePosition() const override;
    std::array<double, 4> baseOrientation() const override;
    std::array<double, 3> baseBodyLinearVelocity() const override;
    std::array<double, 3> baseBodyAngularVelocity() const override;
    std::array<double, 3> baseWorldLinearVelocity() const override;
    std::array<double, 3> baseWorldAngularVelocity() const override;

    bool resetBaseWorldLinearVelocity(
        const std::array<double, 3>& linear = {0, 0, 0}) override;
    bool resetBaseWorldAngularVelocity(
        const std::array<double, 3>& angular = {0, 0, 0}) override;
    bool resetBaseWorldVelocity( //
        const std::array<double, 3>& linear = {0, 0, 0},
        const std::array<double, 3>& angular = {0, 0, 0}) override;

    bool resetBasePose(
        const std::array<double, 3>& position = {0, 0, 0},
        const std::array<double, 4>& orientation = {0, 0, 0, 0}) override;
    bool resetBasePosition(
        const std::array<double, 3>& position = {0, 0, 0}) override;
    bool resetBaseOrientation(
        const std::array<double, 4>& orientation = {0, 0, 0, 0}) override;

    // =================
    // Base Link Targets
    // =================

    bool setBasePoseTarget(const std::array<double, 3>& position,
                           const std::array<double, 4>& orientation) override;
    bool setBasePositionTarget(const std::array<double, 3>& position) override;
    bool
    setBaseOrientationTarget(const std::array<double, 4>& orientation) override;

    bool
    setBaseWorldVelocityTarget(const std::array<double, 3>& linear,
                               const std::array<double, 3>& angular) override;
    bool setBaseWorldLinearVelocityTarget(
        const std::array<double, 3>& linear) override;
    bool setBaseWorldAngularVelocityTarget( //
        const std::array<double, 3>& angular) override;
    bool setBaseWorldLinearAccelerationTarget( //
        const std::array<double, 3>& linear) override;
    bool setBaseWorldAngularAccelerationTarget( //
        const std::array<double, 3>& angular) override;

    std::array<double, 3> basePositionTarget() const override;
    std::array<double, 4> baseOrientationTarget() const override;
    std::array<double, 3> baseWorldLinearVelocityTarget() const override;
    std::array<double, 3> baseWorldAngularVelocityTarget() const override;
    std::array<double, 3> baseWorldLinearAccelerationTarget() const override;
    std::array<double, 3> baseWorldAngularAccelerationTarget() const override;

private:
    friend Link;

    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_YARP_MODEL_H

/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/yarp/Joint.h"
#include "scenario/base/Model.h"
#include "scenario/yarp/Log.h"
#include "scenario/yarp/YarpInterfaces.h"

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <yarp/conf/numeric.h>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>

#import <cmath>
#include <thread>
#include <unordered_map>

using namespace scenario::yarp;

class Joint::Impl
{
public:
    int jointAxis = -1;
    std::shared_ptr<const base::Model> parentModel = nullptr;
    std::shared_ptr<YarpInterfaces> yarpInterfaces = nullptr;
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn = nullptr;

    base::JointControlMode initialControlMode = base::JointControlMode::Idle;

    static base::JointType FromYARP(const ::yarp::dev::JointTypeEnum yarpType);

    static ::yarp::conf::vocab32_t ToYARP(const base::JointControlMode mode);
    static base::JointControlMode
    FromYARP(const ::yarp::conf::vocab32_t yarpMode);

    static ::yarp::dev::Pid ToYARP(const base::PID& pid);
    static base::PID FromYARP(const ::yarp::dev::Pid& yarpPid);

    static ::yarp::dev::PidControlTypeEnum
    ToYARPPidControlType(const base::JointControlMode mode);
};

Joint::Joint()
    : pImpl{std::make_unique<Impl>()}
{}

Joint::~Joint()
{
    if (pImpl->initialControlMode != base::JointControlMode::Invalid
        && !this->setControlMode(pImpl->initialControlMode)) {
        sError << "Failed to restore the initial control mode";
    }
}

bool Joint::initialize(
    const std::shared_ptr<const base::Model> parentModel,
    const int jointAxis,
    const std::shared_ptr<YarpInterfaces>& yarpInterfaces,
    const std::shared_ptr<iDynTree::KinDynComputations>& kinDynComputations)
{
    if (!parentModel->valid()) {
        sError << "The parent model is not valid";
        return false;
    }

    pImpl->jointAxis = jointAxis;
    pImpl->parentModel = parentModel;
    pImpl->kinDyn = kinDynComputations;
    pImpl->yarpInterfaces = yarpInterfaces;

    pImpl->initialControlMode = this->controlMode();

    if (pImpl->initialControlMode == base::JointControlMode::Invalid) {
        return false;
    }

    // TODO Store initial PID

    const auto& model = pImpl->kinDyn->model();

    if (model.getJointName(jointAxis) != this->name()) {
        sError << "YARP joint name (" << this->name()
               << ") does not match with iDynTree joint name ("
               << model.getJointName(jointAxis) << ")";
        return false;
    }

    return static_cast<bool>(pImpl->yarpInterfaces);
}

bool Joint::historyOfAppliedJointForcesEnabled() const {}

bool Joint::enableHistoryOfAppliedJointForces(const bool enable,
                                              const size_t maxHistorySize)
{}

std::vector<double> Joint::historyOfAppliedJointForces() const {}

size_t Joint::dofs() const
{
    // YARP only supports revolute and prismatic joints
    return 1;
}

std::string Joint::name(const bool scoped) const
{
    ::yarp::dev::IAxisInfo* interface = nullptr;
    if (!pImpl->yarpInterfaces->getInterface(interface)) {
        sError << "Failed to get the IAxisInfo interface";
        return {}; // TODO
    }

    std::string yarpAxisName;
    if (!interface->getAxisName(pImpl->jointAxis, yarpAxisName)) {
        sError << "Failed to get the joint name from IAxisInfo";
        return {}; // TODO
    }

    if (scoped) {
        const std::string prefix = pImpl->parentModel->name() + "::";
        yarpAxisName = prefix + yarpAxisName;
    }

    return yarpAxisName;
}

scenario::base::JointType Joint::type() const
{
    ::yarp::dev::IAxisInfo* interface = nullptr;
    if (!pImpl->yarpInterfaces->getInterface(interface)) {
        sError << "Failed to get the IAxisInfo interface";
        return base::JointType::Invalid;
    }

    ::yarp::dev::JointTypeEnum yarpJointType;
    if (!interface->getJointType(pImpl->jointAxis, yarpJointType)) {
        sError << "Failed to get the joint type from IAxisInfo";
        return base::JointType::Invalid;
    }

    return Impl::FromYARP(yarpJointType);
}

scenario::base::JointControlMode Joint::controlMode() const
{
    ::yarp::dev::IControlMode* interface = nullptr;
    if (!pImpl->yarpInterfaces->getInterface(interface)) {
        sError << "Failed to get the IControlMode interface";
        return base::JointControlMode::Invalid;
    }

    ::yarp::conf::vocab32_t yarpControlMode;

    if (!interface->getControlMode(pImpl->jointAxis, &yarpControlMode)) {
        sError << "Failed to get the joint control mode from IControlMode";
        return base::JointControlMode::Invalid;
    }

    return Impl::FromYARP(yarpControlMode);
}

bool Joint::setControlMode(const scenario::base::JointControlMode mode)
{
    switch (mode) {
        case base::JointControlMode::Idle:
            break;
        case base::JointControlMode::PositionInterpolated: {
            ::yarp::dev::IPositionControl* iPositionControl = nullptr;
            if (!pImpl->yarpInterfaces->getInterface(iPositionControl)) {
                sError << "Failed to get the IPositionControl interface";
                return false;
            }

            if (!iPositionControl->setRefSpeed(pImpl->jointAxis, 100.0)) {
                sError << "Failed to set the reference joint position from "
                          "IPositionControl";
                return false;
            }

            // if
            // (!iPositionControl->setRefAcceleration(pImpl->jointAxis, 10.0)) {
            //     sError << "Failed to set the reference joint acceleration
            //     from "
            //               "IPositionControl";
            //     return false;
            // }

            // Initialize the targets with the current position
            //            if (!iPositionControl->positionMove(
            //                    pImpl->jointAxis, this->position() * 180.0 /
            //                    M_PI)) {
            //                sError
            //                    << "Failed to set the joint position from
            //                    IPositionDirect";
            //                return false;
            //            }

            break;
        }
        case base::JointControlMode::Position: {
            ::yarp::dev::IPositionDirect* iPositionDirect = nullptr;
            if (!pImpl->yarpInterfaces->getInterface(iPositionDirect)) {
                sError << "Failed to get the IPositionDirect interface";
                return false;
            }

            // Initialize the targets with the current position
            //            if (!iPositionDirect->setPosition(
            //                    pImpl->jointAxis, this->position() * 180.0 /
            //                    M_PI)) {
            //                sError
            //                    << "Failed to set the joint position from
            //                    IPositionDirect";
            //                return false;
            //            }

            break;
        }
        case base::JointControlMode::Force:
            break;
        case base::JointControlMode::Velocity:
            // TODO Initialize the targets with the current velocity
            break;
        case base::JointControlMode::Invalid:
            return false;
    }

    ::yarp::dev::IControlMode* interface = nullptr;
    if (!pImpl->yarpInterfaces->getInterface(interface)) {
        sError << "Failed to get the IControlMode interface";
        return false;
    }

    if (!interface->setControlMode(pImpl->jointAxis, Impl::ToYARP(mode))) {
        sError << "Failed to set the joint control mode from IControlMode";
        return false;
    }

    return true;
}

double Joint::controllerPeriod() const
{
    // TODO: custom controllers
}

scenario::base::PID Joint::pid() const
{
    const std::vector<base::JointControlMode> allowedControlModes = {
        base::JointControlMode::PositionInterpolated,
        base::JointControlMode::Velocity,
        base::JointControlMode::Force,
    };

    auto it = std::find(allowedControlModes.begin(),
                        allowedControlModes.end(),
                        this->controlMode());

    if (it == allowedControlModes.end()) {
        sError << "The current control mode does not support PID controllers";
        return {}; // TODO
    }

    ::yarp::dev::IPidControl* interface = nullptr;
    if (!pImpl->yarpInterfaces->getInterface(interface)) {
        sError << "Failed to get the IPidControl interface";
        return {}; // TODO
    }

    ::yarp::dev::Pid yarpPid;
    const auto controlType = Impl::ToYARPPidControlType(this->controlMode());

    if (!interface->getPid(controlType, pImpl->jointAxis, &yarpPid)) {
        sError << "Failed to get the joint PID from IPidControl";
        return {}; // TODO
    }

    return Impl::FromYARP(yarpPid);
}

bool Joint::setPID(const scenario::base::PID& pid)
{

    const std::vector<base::JointControlMode> allowedControlModes = {
        base::JointControlMode::PositionInterpolated,
        base::JointControlMode::Velocity,
        base::JointControlMode::Force,
    };

    auto it = std::find(allowedControlModes.begin(),
                        allowedControlModes.end(),
                        this->controlMode());

    if (it == allowedControlModes.end()) {
        sError << "The current control mode does not support PID controllers";
        return false;
    }

    ::yarp::dev::IPidControl* interface = nullptr;
    if (!pImpl->yarpInterfaces->getInterface(interface)) {
        sError << "Failed to get the IPidControl interface";
        return false;
    }

    const ::yarp::dev::Pid yarpPid = Impl::ToYARP(pid);
    const auto controlType = Impl::ToYARPPidControlType(this->controlMode());

    if (!interface->setPid(controlType, pImpl->jointAxis, yarpPid)) {
        sError << "Failed to get the joint PID from IPidControl";
        return false;
    }

    return true;
}

scenario::base::Limit Joint::positionLimit(const size_t dof) const
{
    const auto& model = pImpl->kinDyn->model();
    const auto& joint = model.getJoint(pImpl->jointAxis);
    return {joint->getMinPosLimit(dof), joint->getMaxPosLimit(dof)};

    // From YARP interfaces (disabled)
    //    ::yarp::dev::IControlLimits* iControlLimits2 = nullptr;

    //    if (!pImpl->yarpInterfaces->getInterface(iControlLimits2)) {
    //        sError << "Failed to get the IControlLimits interface";
    //        return {}; // TODO
    //    }

    //    double min, max;

    //    if (!iControlLimits2->getLimits(pImpl->jointAxis, &min, &max)) {
    //        sError << "Failed to get the joint position limit from
    //        IControlLimits"; return {}; // TODO
    //    }

    // TODO deg2rad
    //    return scenario::base::Limit(min, max);
}

double Joint::maxGeneralizedForce(const size_t dof) const {}

bool Joint::setMaxGeneralizedForce(const double maxForce, const size_t dof) {}

double Joint::position(const size_t dof) const
{
    ::yarp::dev::IEncoders* interface = nullptr;
    if (!pImpl->yarpInterfaces->getInterface(interface)) {
        sError << "Failed to get the IEncoders interface";
        return {}; // TODO
    }

    double jointPosition;

    if (!interface->getEncoder(pImpl->jointAxis, &jointPosition)) {
        sError << "Failed to get the joint position from IEncoders";
        return {}; // TODO
    }

    return jointPosition * M_PI / 180.0;
}

double Joint::velocity(const size_t dof) const
{
    ::yarp::dev::IEncoders* interface = nullptr;
    if (!pImpl->yarpInterfaces->getInterface(interface)) {
        sError << "Failed to get the IEncoders interface";
        return {}; // TODO
    }

    double jointVelocity;

    if (!interface->getEncoderSpeed(pImpl->jointAxis, &jointVelocity)) {
        sError << "Failed to get the joint velocity from IEncoders";
        return {}; // TODO
    }

    return jointVelocity * M_PI / 180.0;
}

bool Joint::setPositionTarget(const double position, const size_t dof)
{
    // TODO: what about custom controllers (active mode == force)?
    // TODO: only set the target and move YARP pids and interfaces
    //       into another thread? maybe called at a different frequency?

    const std::vector<base::JointControlMode> allowedControlModes = {
        base::JointControlMode::Position,
        base::JointControlMode::PositionInterpolated,
    };

    auto it = std::find(allowedControlModes.begin(),
                        allowedControlModes.end(),
                        this->controlMode());

    if (it == allowedControlModes.end()) {
        sError << "The active joint control mode does not accept a position "
                  "target";
        return false;
    }

    if (dof >= this->dofs()) {
        sError << "Joint" << this->name() << "does not have DoF#" << dof;
        return false;
    }

    switch (this->controlMode()) {
        case base::JointControlMode::Position: {
            ::yarp::dev::IPositionDirect* interface = nullptr;
            if (!pImpl->yarpInterfaces->getInterface(interface)) {
                sError << "Failed to get the IPositionDirect interface";
                return false;
            }

            if (!interface->setPosition(pImpl->jointAxis,
                                        position * 180.0 / M_PI)) {
                sError
                    << "Failed to set the joint position from IPositionDirect";
                return false;
            }
            break;
        }
        case base::JointControlMode::PositionInterpolated: {
            ::yarp::dev::IPositionControl* interface = nullptr;
            if (!pImpl->yarpInterfaces->getInterface(interface)) {
                sError << "Failed to get the IPositionControl interface";
                return false;
            }

            if (!interface->positionMove(pImpl->jointAxis,
                                         position * 180.0 / M_PI)) {
                sError
                    << "Failed to set the joint position from IPositionControl";
                return false;
            }
            break;
        }
        default:
            sError << "Control mode not recognized";
            return false;
    }

    return true;
}

bool Joint::setVelocityTarget(const double velocity, const size_t dof) {}

bool Joint::setAccelerationTarget(const double acceleration, const size_t dof)
{}

bool Joint::setGeneralizedForceTarget(const double force, const size_t dof) {}

double Joint::positionTarget(const size_t dof) const {}

double Joint::velocityTarget(const size_t dof) const {}

double Joint::accelerationTarget(const size_t dof) const {}

double Joint::generalizedForceTarget(const size_t dof) const {}

bool Joint::resetPosition(const double position, const size_t dof)
{
    // TODO: set position + sleep_for in a while loop? So it's compatible with
    //       the other backends?
}

bool Joint::resetVelocity(const double velocity, const size_t dof) {}

bool Joint::reset(const double position,
                  const double velocity,
                  const size_t dof)
{}

scenario::base::JointLimit Joint::jointPositionLimit() const
{
    assert(this->dofs() == 1);
    const auto& limit = this->positionLimit();
    return {{limit.min}, {limit.max}};
}

std::vector<double> Joint::jointMaxGeneralizedForce() const {}

bool Joint::setJointMaxGeneralizedForce(const std::vector<double>& maxForce) {}

std::vector<double> Joint::jointPosition() const
{
    return {this->position()};
}

std::vector<double> Joint::jointVelocity() const
{
    return {this->velocity()};
}

bool Joint::setJointPositionTarget(const std::vector<double>& position) {}

bool Joint::setJointVelocityTarget(const std::vector<double>& velocity) {}

bool Joint::setJointAccelerationTarget(const std::vector<double>& acceleration)
{}

bool Joint::setJointGeneralizedForceTarget(const std::vector<double>& force) {}

std::vector<double> Joint::jointPositionTarget() const
{
    return {this->positionTarget()};
}

std::vector<double> Joint::jointVelocityTarget() const
{
    return {this->velocityTarget()};
}

std::vector<double> Joint::jointAccelerationTarget() const
{
    return {this->accelerationTarget()};
}

std::vector<double> Joint::jointGeneralizedForceTarget() const
{
    return {this->generalizedForceTarget()};
}

bool Joint::resetJointPosition(const std::vector<double>& position) {}

bool Joint::resetJointVelocity(const std::vector<double>& velocity) {}

bool Joint::resetJoint(const std::vector<double>& position,
                       const std::vector<double>& velocity)
{}

scenario::base::JointType
Joint::Impl::FromYARP(const ::yarp::dev::JointTypeEnum yarpType)
{
    using namespace ::yarp::dev;

    const std::unordered_map<JointTypeEnum, base::JointType>
        yarpToScenarioJointType = {
            {JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE,
             base::JointType::Revolute},
            {JointTypeEnum::VOCAB_JOINTTYPE_PRISMATIC,
             base::JointType::Prismatic},
        };

    if (yarpToScenarioJointType.find(yarpType)
        == yarpToScenarioJointType.end()) {
        sError << "Joint type" << ::yarp::os::Vocab::decode(yarpType)
               << "not recognized";
        return base::JointType::Invalid;
    }

    return yarpToScenarioJointType.at(yarpType);
}

yarp::conf::vocab32_t
Joint::Impl::ToYARP(const scenario::base::JointControlMode mode)
{
    switch (mode) {
        case base::JointControlMode::Idle:
            return VOCAB_CM_IDLE;
        case base::JointControlMode::Position:
            return VOCAB_CM_POSITION_DIRECT;
        case base::JointControlMode::PositionInterpolated:
            return VOCAB_CM_POSITION;
        case base::JointControlMode::Force:
            return VOCAB_CM_TORQUE;
        case base::JointControlMode::Velocity:
            return VOCAB_CM_VELOCITY;
        case base::JointControlMode::Invalid:
            return VOCAB_CM_UNKNOWN;
    }
}

scenario::base::JointControlMode
Joint::Impl::FromYARP(const ::yarp::conf::vocab32_t yarpMode)
{
    const std::unordered_map<::yarp::conf::vocab32_t, base::JointControlMode>
        yarpToScenarioJointControlMode = {
            {VOCAB_CM_IDLE, base::JointControlMode::Idle},
            {VOCAB_CM_POSITION_DIRECT, base::JointControlMode::Position},
            {VOCAB_CM_POSITION, base::JointControlMode::PositionInterpolated},
            {VOCAB_CM_TORQUE, base::JointControlMode::Force},
            {VOCAB_CM_VELOCITY, base::JointControlMode::Velocity},
        };

    if (yarpToScenarioJointControlMode.find(yarpMode)
        == yarpToScenarioJointControlMode.end()) {
        sError << "Control mode" << ::yarp::os::Vocab::decode(yarpMode)
               << "not recognized";
        return base::JointControlMode::Invalid;
    }

    return yarpToScenarioJointControlMode.at(yarpMode);
}

::yarp::dev::Pid Joint::Impl::ToYARP(const scenario::base::PID& pid)
{
    ::yarp::dev::Pid yarpPid;
    yarpPid.setKp(pid.p);
    yarpPid.setKi(pid.p);
    yarpPid.setKd(pid.p);
    yarpPid.setMaxInt(pid.iMax);
    assert(pid.iMax == -pid.iMin);
    yarpPid.setMaxOut(pid.cmdMax);
    assert(pid.cmdMax == -pid.cmdMin);
    yarpPid.setOffset(pid.cmdOffset);

    return yarpPid;
}

scenario::base::PID Joint::Impl::FromYARP(const ::yarp::dev::Pid& yarpPid)
{
    scenario::base::PID pid;
    pid.p = yarpPid.kp;
    pid.i = yarpPid.ki;
    pid.d = yarpPid.kd;
    pid.iMin = -yarpPid.max_int;
    pid.iMax = yarpPid.max_int;
    pid.cmdMin = -yarpPid.max_output;
    pid.cmdMax = yarpPid.max_output;
    pid.cmdOffset = yarpPid.offset;

    return pid;
}

yarp::dev::PidControlTypeEnum
Joint::Impl::ToYARPPidControlType(const scenario::base::JointControlMode mode)
{
    switch (mode) {
        case base::JointControlMode::PositionInterpolated:
            return ::yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION;
        case base::JointControlMode::Velocity:
            return ::yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY;
        case base::JointControlMode::Force:
            return ::yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE;
        default:
            sError << "Control mode does not support PID";
            return {}; // TODO
    }
}

// scenario::base::JointControlMode
// Joint::Impl::FromYARP(const ::yarp::dev::PidControlTypeEnum pidType)
//{
//    switch (pidType) {
//        case ::yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION:
//            return base::JointControlMode::PositionInterpolated;
//        case ::yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY:
//            return base::JointControlMode::Velocity;
//        default:
//            return base::JointControlMode::Invalid;
//    }
//}

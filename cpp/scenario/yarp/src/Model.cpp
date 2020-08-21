/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/yarp/Model.h"
#include "scenario/yarp/Joint.h"
#include "scenario/yarp/Link.h"
#include "scenario/yarp/Log.h"
#include "scenario/yarp/YarpInterfaces.h"
#include "scenario/yarp/YarpRobots.h"
#include "scenario/yarp/helpers.h"
#include "scenario/yarp/utils.h"

#include <Eigen/Dense>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Link.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/robotinterface/Robot.h>

#include <cassert>
#include <cmath>
#include <functional>
#include <mutex>
#include <numeric>
#include <sstream>
#include <thread>
#include <unordered_map>

using namespace scenario::yarp;

class Model::Impl
{
public:
    std::string urdfFile;
    ::yarp::os::Network network;

    // TODO: far passare tutto attraverso kinDyn -> unico entrypoint?

    YarpRobots yarpRobots;
    std::shared_ptr<utils::FloatingBaseEstimator> fbe;
    std::shared_ptr<YarpInterfaces> interfaces = nullptr;
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn = nullptr;

    double controllerPeriod = 0.0;
    std::vector<double> historyJointForces;

    using JointIndex = int;
    using LinkName = std::string;
    using JointName = std::string;

    std::unordered_map<LinkName, scenario::base::LinkPtr> links;
    std::unordered_map<JointName, scenario::base::JointPtr> joints;

    std::unordered_map<JointName, JointIndex> jointNameToIndex;
    std::vector<int>
    toIndices(const std::vector<std::string>& jointNames) const;
};

Model::Model()
    : pImpl{std::make_unique<Impl>()}
{}

Model::~Model() = default;

bool Model::initializeURDF(const std::string& urdfFile)
{
    if (pImpl->kinDyn) {
        sError << "KinDynComputation was already initialized";
        return false;
    }

    // Instantiate KinDynComputations
    pImpl->kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // Explicitly set the velocity representation
    if (!pImpl->kinDyn->setFrameVelocityRepresentation(
            iDynTree::MIXED_REPRESENTATION)) {
        sError << "Failed to set the velocity representation";
        return false;
    }

    // Use ModelLoader to load the reduced model
    iDynTree::ModelLoader mdlLoader;

    if (!mdlLoader.loadModelFromFile(urdfFile)) {
        sError << "Impossible to load the model from '" << urdfFile
               << "'. Possible causes: file not found, or the joints "
               << "list contains an entry not present in the urdf model.";
        return false;
    }

    // Add the loaded model to the KinDynComputations object
    if (!pImpl->kinDyn->loadRobotModel(mdlLoader.model())) {
        sError << "Failed to load the robot model in KinDynComputations";
        return false;
    }

    // Store the map joint name -> index
    const auto& jointNames = this->jointNames();
    for (size_t i = 0; i < jointNames.size(); ++i) {
        pImpl->jointNameToIndex[jointNames[i]] = i;
    }

    pImpl->urdfFile = urdfFile;
    return true;
}

bool Model::initializeRCBR(const std::string& xml)
{
    if (pImpl->urdfFile.empty()) {
        sError << "URDF file has to be initialized first";
        return false;
    }

    if (!pImpl->yarpRobots.loadXml(xml, true)) {
        sError << "Failed to initialize the RemoteControlBoardRemapper";
        return false;
    }

    auto& robot = pImpl->yarpRobots.getRobots().back();

    if (robot.devicePtrs().size() != 1) {
        sError << "Something went wrong while initializing the "
                  "RemoteControlBoardRemapper";
        return false;
    }

    auto* rcbr = robot.devicePtrs()[0];

    if (!(rcbr && rcbr->isOpen())) {
        sError
            << "The RemoteControlBoardRemapper has not been opened correctly";
        return false;
    }

    pImpl->interfaces = std::make_shared<YarpInterfaces>(this->dofs());

    if (!pImpl->interfaces->storeRCBR(rcbr->driver())) {
        sError << "Failed to store the RCBR device";
        return false;
    }

    return true;
}

// HP and diff:
// - urdf model with links and joints that match the real robot
// - classes are not (almost) stateless -> they have the remconboarrem ->
//   expensive
// - base quantities are estimated
// - custom controllers?
// - network delay could create trouble? e.g. change cm and them update PID?
// - only 1 DoF joints are supported, and only revolute / prismatic
// - PID frequency cannot be modified
// - all joints are controlled by default -> use stiff position control for
//   those not controlled
// - xmls loading can be greatly improved
// - hardcoded feet frames for contact detection and contact forces

bool Model::initialize(
    /*const std::string& robotName,
                       const std::string& urdfFile,
                       const std::vector<std::string>& controlBoardNames*/ //,
                       //    const std::vector<std::string>& controlledJoints //
                       //    TODO: always full model
)
{
    //    return true;
    // TODOOOOOOOOOOOOOOOOOOOOOOOOOOOO
    if (pImpl->fbe) {
        sError
            << "The floating base estimator connector has been already created";
        return false;
    }

    pImpl->fbe = std::make_shared<utils::FloatingBaseEstimator>();

    if (!pImpl->fbe->connect()) {
        sError << "Failed to connect the floating base estimator";
        return false;
    }

    return true;
}

// https://github.com/robotology/yarp/pull/2288
// bool Model::loadYarpDevice(const std::string& xml) // ini
//{
//    const std::string xmlFile = utils::FindYarpResource(xml);

//    if (xmlFile.empty()) {
//        sError << "Failed to find file" << xml;
//        return false;
//    }

//    ::yarp::os::Property options = utils::parseConfig(xmlFile);
//    sMessage << options.toString();

//    // Create the device driver
//    auto device = std::make_unique<::yarp::dev::PolyDriver>();

//    // Open the device driver
//    if (!(device->open(options) && device->isValid())) {
//        sError << "Failed to open the device driver";
//        return false;
//    }

//    return true;
//}

bool Model::launchYarpRobotInterfaceXml(const std::string& xml, const bool join)
{
    if (!pImpl->yarpRobots.loadXml(xml, join)) {
        sError << "Failed to load xml" << xml;
        return false;
    }

    return true;
}

bool Model::historyOfAppliedJointForcesEnabled(
    const std::vector<std::string>& jointNames) const
{
    sWarning << "Getting the history of joint forces has not yet been "
                "implemented";
    return true;
}

bool Model::enableHistoryOfAppliedJointForces(
    const bool enable,
    const size_t maxHistorySizePerJoint,
    const std::vector<std::string>& jointNames)
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    if (enable) {
        pImpl->historyJointForces = std::vector<double>(
            jointSerialization.size() * maxHistorySizePerJoint);
        sWarning << "Getting the history of joint forces has not yet been "
                    "implemented";
        return true;
    }

    pImpl->historyJointForces.clear();
    return true;
}

std::vector<double> Model::historyOfAppliedJointForces(
    const std::vector<std::string>& jointNames) const
{
    return pImpl->historyJointForces;
}

bool Model::valid() const
{
    // TODOOOOOOOOOOOOOOOOOOOOOOOOOOO
    return pImpl->kinDyn->isValid() && pImpl->interfaces && pImpl->fbe;
    //    return pImpl->kinDyn->isValid() && pImpl->interfaces;
}

size_t Model::dofs(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    if (jointNames.empty()) {
        return pImpl->kinDyn->model().getNrOfDOFs();
    }

    size_t dofs = 0;

    for (const auto& joint : this->joints(jointSerialization)) {
        dofs += joint->dofs();
    }

    return dofs;
}

std::string Model::name() const
{
    return "icubSim"; // TODO
}

size_t Model::nrOfLinks() const
{
    return pImpl->kinDyn->getNrOfLinks();
}

size_t Model::nrOfJoints() const
{
    return pImpl->kinDyn->model().getNrOfJoints();
}

double Model::totalMass(const std::vector<std::string>& linkNames) const
{
    if (linkNames.empty()) {
        return pImpl->kinDyn->model().getTotalMass();
    }

    double mass = 0;

    for (const auto& link : links(linkNames)) {
        mass += link->mass();
    }

    return mass;
}

scenario::base::LinkPtr Model::getLink(const std::string& linkName) const
{
    if (pImpl->links.find(linkName) != pImpl->links.end()) {
        assert(pImpl->links.at(linkName));
        return pImpl->links.at(linkName);
    }

    //    if (linkEntity == ignition::gazebo::kNullEntity) {
    //        throw exceptions::LinkNotFound(linkName);
    //    }

    // Create the link
    auto link = std::make_shared<scenario::yarp::Link>();

    if (!link->initialize(this->shared_from_this(),
                          linkName,
                          pImpl->interfaces,
                          pImpl->kinDyn,
                          pImpl->fbe)) {
        //            throw exceptions::LinkError("Failed to initialize link",
        //            linkName);
        sError << "Failed to initialize link" << linkName;
        return nullptr;
    }

    // Cache the link instance
    pImpl->links[linkName] = link;

    return link;
}

scenario::base::JointPtr Model::getJoint(const std::string& jointName) const
{
    if (pImpl->joints.find(jointName) != pImpl->joints.end()) {
        assert(pImpl->joints.at(jointName));
        return pImpl->joints.at(jointName);
    }

    //    if (jointEntity == ignition::gazebo::kNullEntity) {
    //        throw exceptions::JointNotFound(jointName);
    //    }

    // Create the joint
    auto joint = std::make_shared<scenario::yarp::Joint>();
    const auto& model = pImpl->kinDyn->model();

    // TODO: parent model?
    if (!joint->initialize(this->shared_from_this(),
                           model.getJointIndex(jointName),
                           pImpl->interfaces,
                           pImpl->kinDyn)) {
        // throw exceptions::JointError("Failed to initialize joint",
        // jointName);
        sError << "Failed to initialize joint" << jointName;
        return nullptr;
    }

    // Cache the joint instance
    pImpl->joints[jointName] = joint;

    return joint;
}

std::vector<std::string> Model::linkNames(const bool scoped) const
{
    std::vector<std::string> linkNames;
    linkNames.reserve(this->nrOfLinks());

    const auto& model = pImpl->kinDyn->model();

    for (unsigned i = 0; i < model.getNrOfLinks(); ++i) {

        if (scoped) {
            linkNames.push_back(this->name() + "::" + model.getLinkName(i));
        }
        else {
            linkNames.push_back(model.getLinkName(i));
        }
    }

    return linkNames;
}

std::vector<std::string> Model::jointNames(const bool scoped) const
{
    std::vector<std::string> jointNames;
    jointNames.reserve(this->nrOfJoints());

    const auto& model = pImpl->kinDyn->model();

    for (unsigned i = 0; i < model.getNrOfJoints(); ++i) {
        if (model.getJoint(i)->getNrOfDOFs() < 1) {
            continue;
        }

        if (scoped) {
            jointNames.push_back(this->name() + model.getJointName(i));
        }
        else {
            jointNames.push_back(model.getJointName(i));
        }
    }

    return jointNames;
}

double Model::controllerPeriod() const
{
    //    sWarning << "The controller period is currently no-op on this
    //    backend";
    return pImpl->controllerPeriod;
}

bool Model::setControllerPeriod(const double period)
{
    //    sWarning << "The controller period is currently no-op on this
    //    backend";
    pImpl->controllerPeriod = period;
    return true;
}

bool Model::contactsEnabled() const
{
    return true;
}

bool Model::enableContacts(const bool enable)
{
    //    if (enable) {
    //        // TODO: use skin YARP interface?
    //        sError << "Contact detection is not yet supported";
    //        return false;
    //    }
    return true;
}

bool Model::selfCollisionsEnabled() const
{
    return true;
}

bool Model::enableSelfCollisions(const bool enable)
{
    if (!enable) {
        sError << "Real robots cannot avoid to self-collide";
        return false;
    }

    return true;
}

std::vector<std::string> Model::linksInContact() const
{
    std::vector<std::string> linksInContact;

    std::lock_guard lock(pImpl->fbe->contactState.mutex);

    if (pImpl->fbe->contactState.contacts.left_foot == 1) {
        // TODO hardcoded
        assert(this->getLink("l_foot") != nullptr);
        linksInContact.push_back("l_foot");
    }

    if (pImpl->fbe->contactState.contacts.right_foot == 1) {
        // TODO hardcoded
        assert(this->getLink("r_foot") != nullptr);
        linksInContact.push_back("r_foot");
    }

    return linksInContact;
}

std::vector<scenario::base::Contact>
Model::contacts(const std::vector<std::string>& /*linkNames*/) const
{
    sError << "Contact detection has not yet been implemented";
    return {};
}

std::vector<double>
Model::jointPositions(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    ::yarp::dev::IEncoders* interface = nullptr;
    if (!pImpl->interfaces->getInterface(interface)) {
        sError << "Failed to get the IEncoders interface";
        return {}; // TODO
    }

    std::vector<double> positions;

    if (jointNames.empty()) {

        positions.resize(jointSerialization.size());

        if (!interface->getEncoders(positions.data())) {
            sError << "Failed to get the joint positions from IEncoders";
            return {}; // TODO
        }

        std::transform(positions.begin(),
                       positions.end(),
                       positions.begin(),
                       std::bind1st(std::multiplies<double>(), M_PI / 180.0));
    }
    else {

        positions.reserve(jointSerialization.size());

        for (const auto& joint : this->joints(jointSerialization)) {
            assert(joint->dofs() == 1);
            positions.push_back(joint->position());
        }
    }

    return positions;
}

std::vector<double>
Model::jointVelocities(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    ::yarp::dev::IEncoders* interface = nullptr;
    if (!pImpl->interfaces->getInterface(interface)) {
        sError << "Failed to get the IEncoders interface";
        return {}; // TODO
    }

    std::vector<double> velocities;

    if (jointNames.empty()) {

        velocities.resize(jointSerialization.size());

        if (!interface->getEncoderSpeeds(velocities.data())) {
            sError << "Failed to get the joint velocities from IEncoders";
            return {}; // TODO
        }

        std::transform(velocities.begin(),
                       velocities.end(),
                       velocities.begin(),
                       std::bind1st(std::multiplies<double>(), M_PI / 180.0));
    }
    else {

        velocities.reserve(jointSerialization.size());

        for (const auto& joint : this->joints(jointSerialization)) {
            assert(joint->dofs() == 1);
            velocities.push_back(joint->velocity());
        }
    }

    return velocities;
}

scenario::base::JointLimit
Model::jointLimits(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    scenario::base::JointLimit limits;
    limits.min.reserve(jointSerialization.size());
    limits.max.reserve(jointSerialization.size());

    for (const auto& joint : this->joints(jointSerialization)) {
        const auto& limit = joint->positionLimit();
        limits.min.push_back(limit.min);
        limits.max.push_back(limit.max);
    }

    return limits;
}

bool Model::setJointControlMode(const scenario::base::JointControlMode mode,
                                const std::vector<std::string>& jointNames)
{
    // TODO vectorized

    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    bool ok = true;

    for (const auto& joint : this->joints(jointSerialization)) {
        ok = ok && joint->setControlMode(mode);
        sWarning << ok << " " << joint->name();
    }

    return ok;
}

std::vector<scenario::base::LinkPtr>
Model::links(const std::vector<std::string>& linkNames) const
{
    const std::vector<std::string>& linkSerialization =
        linkNames.empty() ? this->linkNames() : linkNames;

    std::vector<base::LinkPtr> links;
    links.reserve(linkSerialization.size());

    for (const auto& linkName : linkSerialization) {
        links.push_back(this->getLink(linkName));
    }

    return links;
}

std::vector<scenario::base::JointPtr>
Model::joints(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    std::vector<base::JointPtr> joints;
    joints.reserve(jointSerialization.size());

    for (const auto& jointName : jointSerialization) {
        joints.push_back(this->getJoint(jointName));
    }

    return joints;
}

bool Model::setJointPositionTargets(const std::vector<double>& positions,
                                    const std::vector<std::string>& jointNames)
{
    // TODO: do this also elsewhere
    // TODO: check joints exist
    if (!jointNames.empty() && positions.size() != jointNames.size()) {
        sError << "Wrong arguments";
        return false;
    }

    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    const auto& joints = this->joints(jointSerialization);

    auto sameControlMode = [&]() {
        const base::JointControlMode mode = joints.front()->controlMode();
        for (const auto& joint : joints) {
            if (joint->controlMode() != mode) {
                return false;
            }
        }
        return true;
    };

    if (!sameControlMode()) {
        sError << "Cannot set position targets to joints controlled in "
                  "different modes";
        return false;
    }

    //    std::vector<int> indices;
    //    indices.reserve(jointSerialization.size());
    //    indices = pImpl->toIndices(jointSerialization);

    std::vector<double> positionsInDeg = std::move(positions);
    std::transform(positionsInDeg.begin(),
                   positionsInDeg.end(),
                   positionsInDeg.begin(),
                   std::bind1st(std::multiplies<double>(), 180.0 / M_PI));

    switch (joints.front()->controlMode()) {
        case base::JointControlMode::Position: {
            ::yarp::dev::IPositionDirect* interface = nullptr;
            if (!pImpl->interfaces->getInterface(interface)) {
                sError << "Failed to get the IPositionDirect interface";
                return false;
            }

            if (!interface->setPositions(
                    positionsInDeg.size(),
                    pImpl->toIndices(jointSerialization).data(),
                    positionsInDeg.data())) {
                sError
                    << "Failed to set the joint position from IPositionDirect";
                return false;
            }
            break;
        }
        case base::JointControlMode::PositionInterpolated: {
            ::yarp::dev::IPositionControl* interface = nullptr;
            if (!pImpl->interfaces->getInterface(interface)) {
                sError << "Failed to get the IPositionControl interface";
                return false;
            }

            if (!interface->positionMove(
                    positionsInDeg.size(),
                    pImpl->toIndices(jointSerialization).data(),
                    positionsInDeg.data())) {
                sError
                    << "Failed to set the joint position from IPositionControl";
                return false;
            }
            break;
        }
        default:
            sError << "Control mode not supported";
            return false;
    }

    return true;

    //    bool ok = true;

    //    // TODO: vectorized
    //    for (unsigned i = 0; i < joints.size(); ++i) {
    //        const auto& joint = joints[i];
    //        const auto& position = positions[i];

    //        // TODO: assert 1DoF
    //        if (!joint->setPositionTarget(position)) {
    //            sError << "Failed to set the position of joint" <<
    //            joint->name(); ok = ok && false;
    //        }
    //    }

    //    return ok;
}

bool Model::setJointVelocityTargets(const std::vector<double>& velocities,
                                    const std::vector<std::string>& jointNames)
{
    return false;
}

bool Model::setJointAccelerationTargets(
    const std::vector<double>& accelerations,
    const std::vector<std::string>& jointNames)
{
    return false;
}

bool Model::setJointGeneralizedForceTargets(
    const std::vector<double>& forces,
    const std::vector<std::string>& jointNames)
{
    return false;
}

bool Model::resetJointPositions(const std::vector<double>& positions,
                                const std::vector<std::string>& jointNames)
{
    // TODO: do this also elsewhere
    // TODO: check joints exist
    if (!jointNames.empty() && positions.size() != jointNames.size()) {
        sError << "Wrong arguments";
        return false;
    }

    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    const auto& joints = this->joints(jointSerialization);

    // Store the active control modes so that we can restore them later
    std::vector<base::JointControlMode> initialModes;
    initialModes.reserve(this->dofs(jointSerialization));

    for (const auto& joint : joints) {
        initialModes.push_back(joint->controlMode());
    }

    // Restore the control mode when exiting this scope
    auto deleter = [&](void*) -> void {
        for (unsigned i = 0; i < joints.size(); ++i) {
            const auto& joint = joints[i];
            const auto& mode = initialModes[i];

            if (!joint->setControlMode(mode)) {
                sError << "Failed to restore the control mode of joint"
                       << joint->name();
            }
        }
    };

    // We use a dummy unique pointer with custom deleter to implement RAII
    std::unique_ptr<void, decltype(deleter)> raii{nullptr, deleter};

    // Control all the joints in PositionInterpolated
    if (!this->setJointControlMode(base::JointControlMode::PositionInterpolated,
                                   jointSerialization)) {
        sError << "Failed to set PositionInterpolated control mode";
        return false;
    }

    // Reset the joint positions
    if (!this->setJointPositionTargets(positions, jointSerialization)) {
        sError << "Failed to set the joint position targets";
        return false;
    }

    // Get the interface
    ::yarp::dev::IPositionControl* interface = nullptr;
    if (!pImpl->interfaces->getInterface(interface)) {
        sError << "Failed to get the IPositionControl interface";
        return false;
    }

    // Helper to check if the reset positions have been reached
    auto motionDone = [&]() -> bool {
        bool done;
        if (!interface->checkMotionDone(&done)) {
            sError << "Failed to check if motion is done";
            return false;
        }

        return done;
    };

    // Wait the trajectory transient to terminate
    while (!motionDone()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return true;
}

bool Model::resetJointVelocities(const std::vector<double>& velocities,
                                 const std::vector<std::string>& jointNames)
{
    sDebug << "NotImplemented";
    return false;
}

std::vector<double>
Model::jointPositionTargets(const std::vector<std::string>& jointNames) const
{}

std::vector<double>
Model::jointVelocityTargets(const std::vector<std::string>& jointNames) const
{}

std::vector<double> Model::jointAccelerationTargets(
    const std::vector<std::string>& jointNames) const
{}

std::vector<double> Model::jointGeneralizedForceTargets(
    const std::vector<std::string>& jointNames) const
{}

std::string Model::baseFrame() const {}

// bool Model::setBaseFrame(const std::string& frameName)
//{
//    return false;
//}

// bool Model::fixedBase() const {}

// bool Model::setAsFixedBase(const bool fixedBase) {}

std::array<double, 3> Model::basePosition() const
{
    std::unique_lock lock(pImpl->fbe->baseState.mutex);
    return {pImpl->fbe->baseState.position[0],
            pImpl->fbe->baseState.position[1],
            pImpl->fbe->baseState.position[2]};
}

std::array<double, 4> Model::baseOrientation() const
{
    std::unique_lock lock(pImpl->fbe->baseState.mutex);
    return {pImpl->fbe->baseState.quaternion.w(),
            pImpl->fbe->baseState.quaternion.x(),
            pImpl->fbe->baseState.quaternion.y(),
            pImpl->fbe->baseState.quaternion.z()};
}

std::array<double, 3> Model::baseBodyLinearVelocity() const
{
    // Conversion from mixed to body-fixed representation
    std::unique_lock lock(pImpl->fbe->baseState.mutex);

    const Eigen::Matrix3d& W_R_B =
        pImpl->fbe->baseState.quaternion.toRotationMatrix();

    std::array<double, 3> linearVelocity = this->baseWorldLinearVelocity();
    Eigen::Map<Eigen::Vector3d> linearVelocityMap(linearVelocity.data());

    const auto& bodyVelocity = W_R_B.inverse() * linearVelocityMap;
    return {bodyVelocity[0], bodyVelocity[1], bodyVelocity[2]};
}

std::array<double, 3> Model::baseBodyAngularVelocity() const
{
    // Conversion from mixed to body-fixed representation
    std::unique_lock lock(pImpl->fbe->baseState.mutex);

    const Eigen::Matrix3d& W_R_B =
        pImpl->fbe->baseState.quaternion.toRotationMatrix();

    std::array<double, 3> angularVelocity = this->baseWorldAngularVelocity();
    Eigen::Map<Eigen::Vector3d> angularVelocityMap(angularVelocity.data());

    const auto& bodyVelocity = W_R_B.inverse() * angularVelocityMap;
    return {bodyVelocity[0], bodyVelocity[1], bodyVelocity[2]};
}

std::array<double, 3> Model::baseWorldLinearVelocity() const
{
    std::unique_lock lock(pImpl->fbe->baseState.mutex);
    return {pImpl->fbe->baseState.linVel[0],
            pImpl->fbe->baseState.linVel[1],
            pImpl->fbe->baseState.linVel[2]};
}

std::array<double, 3> Model::baseWorldAngularVelocity() const
{
    std::unique_lock lock(pImpl->fbe->baseState.mutex);
    return {pImpl->fbe->baseState.angVel[0],
            pImpl->fbe->baseState.angVel[1],
            pImpl->fbe->baseState.angVel[2]};
}

bool Model::resetBaseWorldLinearVelocity(
    const std::array<double, 3>& /*linear*/)
{
    return false;
}

bool Model::resetBaseWorldAngularVelocity(
    const std::array<double, 3>& /*angular*/)
{
    return false;
}

bool Model::resetBaseWorldVelocity(const std::array<double, 3>& /*linear*/,
                                   const std::array<double, 3>& /*angular*/)
{
    return false;
}

bool Model::resetBasePose(const std::array<double, 3>& /*position*/,
                          const std::array<double, 4>& /*orientation*/)
{
    return false;
}

bool Model::resetBasePosition(const std::array<double, 3>& /*position*/)
{
    return false;
}

bool Model::resetBaseOrientation(const std::array<double, 4>& /*orientation*/)
{
    return false;
}

bool Model::setBasePoseTarget(const std::array<double, 3>& position,
                              const std::array<double, 4>& orientation)
{
    return false;
}

bool Model::setBasePositionTarget(const std::array<double, 3>& position)
{
    return false;
}

bool Model::setBaseOrientationTarget(const std::array<double, 4>& orientation)
{
    return false;
}

bool Model::setBaseWorldVelocityTarget(const std::array<double, 3>& linear,
                                       const std::array<double, 3>& angular)
{
    return false;
}

bool Model::setBaseWorldLinearVelocityTarget(
    const std::array<double, 3>& linear)
{
    return false;
}

bool Model::setBaseWorldAngularVelocityTarget(
    const std::array<double, 3>& angular)
{
    return false;
}

bool Model::setBaseWorldLinearAccelerationTarget(
    const std::array<double, 3>& linear)
{
    return false;
}

bool Model::setBaseWorldAngularAccelerationTarget(
    const std::array<double, 3>& angular)
{
    return false;
}

std::array<double, 3> Model::basePositionTarget() const {}

std::array<double, 4> Model::baseOrientationTarget() const {}

std::array<double, 3> Model::baseWorldLinearVelocityTarget() const {}

std::array<double, 3> Model::baseWorldAngularVelocityTarget() const {}

std::array<double, 3> Model::baseWorldLinearAccelerationTarget() const {}

std::array<double, 3> Model::baseWorldAngularAccelerationTarget() const {}

std::vector<int>
Model::Impl::toIndices(const std::vector<std::string>& jointNames) const
{
    std::vector<int> indices;
    indices.reserve(jointNames.size());

    for (const auto& name : jointNames) {
        assert(jointNameToIndex.find(name) != jointNameToIndex.end());
        indices.push_back(jointNameToIndex.at(name));
    }

    return indices;
}

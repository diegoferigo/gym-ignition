/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/yarp/Link.h"
#include "scenario/base/Model.h"
#include "scenario/yarp/Log.h"
#include "scenario/yarp/RobotInterface.h"
#include "scenario/yarp/helpers.h"

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Link.h>
#include <iDynTree/Model/Model.h>

using namespace scenario::yarp;

class Link::Impl
{
public:
    std::string linkName;
    iDynTree::LinkIndex linkIndex;
    std::shared_ptr<const base::Model> parentModel = nullptr;
    std::shared_ptr<YarpInterfaces> yarpInterfaces = nullptr;
    std::shared_ptr<utils::FloatingBaseEstimator> fbe = nullptr;
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn = nullptr;

    static bool
    Update(const std::shared_ptr<const base::Model>& model,
           std::shared_ptr<iDynTree::KinDynComputations> kinDynComputations);
};

Link::Link()
    : pImpl{std::make_unique<Impl>()}
{}

Link::~Link() = default;

bool Link::initialize(
    const std::shared_ptr<const scenario::base::Model> parentModel,
    const std::string& linkName,
    const std::shared_ptr<YarpInterfaces>& yarpInterfaces,
    const std::shared_ptr<iDynTree::KinDynComputations>& kinDynComputations,
    const std::shared_ptr<utils::FloatingBaseEstimator>& fbe)
{
    if (!parentModel->valid()) {
        sError << "The parent model is not valid";
        return false;
    }

    pImpl->linkIndex = kinDynComputations->model().getLinkIndex(linkName);

    if (pImpl->linkIndex == iDynTree::LINK_INVALID_INDEX) {
        sError << "Link " << linkName << "does not exist in the model";
        return false;
    }

    pImpl->fbe = fbe;
    pImpl->linkName = linkName;
    pImpl->parentModel = parentModel;
    pImpl->kinDyn = kinDynComputations;
    pImpl->yarpInterfaces = yarpInterfaces;

    return true;
}

std::string Link::name(const bool scoped) const
{
    return scoped ? pImpl->parentModel->name() + "::" + pImpl->linkName
                  : pImpl->linkName;
}

double Link::mass() const
{
    const auto& link = pImpl->kinDyn->model().getLink(pImpl->linkIndex);
    return link->getInertia().asVector()[0];
}

std::array<double, 3> Link::position() const
{
    if (!Impl::Update(pImpl->parentModel, pImpl->kinDyn)) {
        sError << "Failed to update KinDynComputations state";
        return {}; // TODO
    }

    const auto& tf = pImpl->kinDyn->getWorldTransform(pImpl->linkName);

    auto FromiDynTree =
        [](const iDynTree::Position& position) -> std::array<double, 3> {
        return {position[0], position[1], position[2]};
    };

    return FromiDynTree(tf.getPosition());
}

std::array<double, 4> Link::orientation() const
{
    if (!Impl::Update(pImpl->parentModel, pImpl->kinDyn)) {
        sError << "Failed to update KinDynComputations state";
        return {}; // TODO
    }

    const auto& tf = pImpl->kinDyn->getWorldTransform(pImpl->linkName);

    auto FromiDynTree =
        [](const iDynTree::Rotation& rotation) -> std::array<double, 4> {
        const auto& quaternion = rotation.asQuaternion();
        return {quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
    };

    return FromiDynTree(tf.getRotation());
}

std::array<double, 3> Link::worldLinearVelocity() const
{
    return {0, 0, 0};
}

std::array<double, 3> Link::worldAngularVelocity() const
{
    return {0, 0, 0};
}

std::array<double, 3> Link::bodyLinearVelocity() const
{
    return {0, 0, 0};
}

std::array<double, 3> Link::bodyAngularVelocity() const
{
    return {0, 0, 0};
}

std::array<double, 3> Link::worldLinearAcceleration() const
{
    return {0, 0, 0};
}

std::array<double, 3> Link::worldAngularAcceleration() const
{
    return {0, 0, 0};
}

std::array<double, 3> Link::bodyLinearAcceleration() const
{
    return {0, 0, 0};
}

std::array<double, 3> Link::bodyAngularAcceleration() const
{
    return {0, 0, 0};
}

bool Link::contactsEnabled() const
{
    if (this->name() == "l_foot" || this->name() == "r_foot") {
        return true;
    }

    return false;
}

bool Link::enableContactDetection(const bool enable)
{
    if (this->name() == "l_foot" || this->name() == "r_foot") {
        if (!enable) {
            sError << "Contact detection cannot be disabled";
            return false;
        }
        else {
            return true;
        }
    }

    return enable ? false : true;
}

bool Link::inContact() const
{

    if (this->name() == "l_foot") {
        std::lock_guard lock(pImpl->fbe->contactState.mutex);
        return bool(pImpl->fbe->contactState.contacts.left_foot);
    }

    if (this->name() == "r_foot") {
        std::lock_guard lock(pImpl->fbe->contactState.mutex);
        return bool(pImpl->fbe->contactState.contacts.right_foot);
    }

    sError << "Contact detection is not enabled for this link";
    return false;
}

std::vector<scenario::base::Contact> Link::contacts() const
{
    return {}; // TODO
}

std::array<double, 6> Link::contactWrench() const
{
    // TODO: the wrenches are expressed in the frame of the link
    //       (check the wbd config)

    auto ToiDynTreeTF =
        [](const std::array<double, 3>& position,
           const std::array<double, 4>& quaternion) -> iDynTree::Transform {
        iDynTree::Transform tf;
        iDynTree::Rotation rotation;

        rotation.fromQuaternion({quaternion.data(), 4});

        tf.setRotation(std::move(rotation));
        tf.setPosition({position[0], position[1], position[2]});

        return tf;
    };

    if (this->name() == "l_foot") {
        const auto& tf = ToiDynTreeTF(this->position(), this->orientation());
        std::lock_guard lock(pImpl->fbe->externalWrenches.left_foot.mutex);
        const auto& f = pImpl->fbe->externalWrenches.left_foot.wrench;
        const iDynTree::Wrench wrench({f[0], f[1], f[2]}, {f[3], f[4], f[5]});
        const iDynTree::Wrench worldWrench = tf * wrench;
        return {worldWrench(0),
                worldWrench(1),
                worldWrench(2),
                worldWrench(3),
                worldWrench(4),
                worldWrench(5)};
    }

    if (this->name() == "r_foot") {
        const auto& tf = ToiDynTreeTF(this->position(), this->orientation());
        std::lock_guard lock(pImpl->fbe->externalWrenches.right_foot.mutex);
        const auto& f = pImpl->fbe->externalWrenches.right_foot.wrench;
        const iDynTree::Wrench wrench({f[0], f[1], f[2]}, {f[3], f[4], f[5]});
        const iDynTree::Wrench worldWrench = tf * wrench;
        return {worldWrench(0),
                worldWrench(1),
                worldWrench(2),
                worldWrench(3),
                worldWrench(4),
                worldWrench(5)};
    }

    sWarning << "This link does not provide any contact wrench";
    return {0, 0, 0, 0, 0, 0};
}

bool Link::applyWorldForce(const std::array<double, 3>& force,
                           const double duration)
{
    return false;
}

bool Link::applyWorldTorque(const std::array<double, 3>& torque,
                            const double duration)
{
    return false;
}

bool Link::applyWorldWrench(const std::array<double, 3>& force,
                            const std::array<double, 3>& torque,
                            const double duration)
{
    return false;
}

bool Link::Impl::Update(
    const std::shared_ptr<const scenario::base::Model>& model,
    std::shared_ptr<iDynTree::KinDynComputations> kinDynComputations)
{
    if (!(kinDynComputations && kinDynComputations->isValid())) {
        sError << "KinDynComputations is not valid";
        return false;
    }

    auto ToiDynTreeTF =
        [](const std::array<double, 3>& position,
           const std::array<double, 4>& quaternion) -> iDynTree::Transform {
        iDynTree::Transform tf;
        iDynTree::Rotation rotation;

        rotation.fromQuaternion({quaternion.data(), 4});

        tf.setRotation(std::move(rotation));
        tf.setPosition({position[0], position[1], position[2]});

        return tf;
    };

    auto ToiDynTreeVec =
        [](const std::vector<double>& input) -> iDynTree::VectorDynSize {
        return iDynTree::VectorDynSize(input.data(), input.size());
    };

    // TODO: gravity
    auto gravity = iDynTree::Vector3();
    gravity.zero();
    gravity[2] = -9.80;

    auto ToiDynTreeGeom =
        [](const std::array<double, 3> input) -> iDynTree::GeomVector3 {
        return {input.data(), 3};
    };

    assert(kinDynComputations->getFrameVelocityRepresentation()
           == iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION);

    return kinDynComputations->setRobotState(
        ToiDynTreeTF(model->basePosition(), model->baseOrientation()),
        ToiDynTreeVec(model->jointPositions()),
        iDynTree::Twist(ToiDynTreeGeom(model->baseWorldLinearVelocity()),
                        ToiDynTreeGeom(model->baseWorldAngularVelocity())),
        ToiDynTreeVec(model->jointVelocities()),
        gravity);
}

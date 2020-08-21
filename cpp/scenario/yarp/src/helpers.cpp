/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/yarp/helpers.h"
#include "scenario/yarp/Log.h"
#include "scenario/yarp/utils.h"

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcClient.h>

#include <thread>

using namespace scenario::yarp::utils;

yarp::os::Property scenario::yarp::utils::parseConfig(const std::string& config)
{
    // TODO: assert config file exists
    ::yarp::os::Property property;

    if (!property.fromConfigFile(config)) {
        sError << "Failed to parse" << config;
        return {};
    }

    return property;
}

void BaseState::onRead(::yarp::sig::VectorOf<double>& datum)
{
    if (datum.size() != 12) {
        sError << "Received misformed BaseState data";
        return;
    }

    std::unique_lock lock(mutex);

    position << datum[0], datum[1], datum[2];
    linVel << datum[6], datum[7], datum[8];
    angVel << datum[9], datum[10], datum[11];

    quaternion = Eigen::AngleAxisd(datum[3], Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(datum[4], Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(datum[5], Eigen::Vector3d::UnitZ());
}

void ContactState::onRead(::yarp::os::Bottle& datum)
{
    if (datum.size() != 5) {
        sError << "Received malformed ContactState data";
        return;
    }

    auto getInt = [](const ::yarp::os::Value& value) -> int {
        if (value.isInt()) {
            return value.asInt();
        }
        sError << "Expected Int but got something else";
        return 0;
    };

    auto getFloat64 = [](const ::yarp::os::Value& value) -> double {
        if (value.isFloat64()) {
            return value.asFloat64();
        }
        sError << "Expected Float64 but got something else";
        return 0.0;
    };

    std::unique_lock lock(mutex);

    forces.left_foot = getFloat64(datum.get(0));
    forces.right_foot = getFloat64(datum.get(1));
    contacts.left_foot = getInt(datum.get(2));
    contacts.right_foot = getInt(datum.get(2));
}

void ExternalWrenches::ExternalWrench::onRead(
    ::yarp::sig::VectorOf<double>& datum)
{
    if (datum.size() != 6) {
        sError << "Received misformed BaseState data";
        return;
    }

    std::unique_lock lock(mutex);
    wrench = {datum[0], datum[1], datum[2], datum[3], datum[4], datum[5]};
}

bool FloatingBaseEstimator::connect()
{
    if (!this->isConnected()) {
        using namespace ::yarp;

        os::RpcClient rpc;
        if (!rpc.open("...")) {
            sError << "Failed to open RPC port for starting the base "
                      "estimator";
            return false;
        }

        if (!os::Network::connect(rpc.getName(), rpcPort)) {
            sError << "Failed to connect to RPC server";
            return false;
        }

        os::Bottle in, out;
        out.fromString("startFloatingBaseFilter");

        size_t attempts = 0;
        while (attempts++ < 20 && !rpc.write(out, in)) {
            sError << "... waiting RPC reply";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        if (in.toString() != "[ok]") {
            sError << "Failed to start the base estimator";
            return false;
        }

        baseState.port.useCallback(baseState);
        contactState.port.useCallback(contactState);
        externalWrenches.left_foot.port.useCallback(externalWrenches.left_foot);
        externalWrenches.right_foot.port.useCallback(
            externalWrenches.right_foot);

        if (!baseState.port.open("...") || !contactState.port.open("...")
            || !externalWrenches.left_foot.port.open("...")
            || !externalWrenches.right_foot.port.open("...")) {
            sError << "Failed to open local ports";
            return false;
        }

        if (!os::Network::connect(floatingBasePort, baseState.port.getName())
            || !os::Network::connect(feetContactPort,
                                     contactState.port.getName())
            || !os::Network::connect(lFootExtWrenchPort,
                                     externalWrenches.left_foot.port.getName())
            || !os::Network::connect(
                rFootExtWrenchPort,
                externalWrenches.right_foot.port.getName())) {
            sError << "Failed to connect base estimator ports";
            return false;
        }

        while (baseState.port.getPendingReads() < 1
               && contactState.port.getPendingReads() < 1
               && externalWrenches.left_foot.port.getPendingReads() < 1
               && externalWrenches.right_foot.port.getPendingReads() < 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        return true;
    }

    sError << "The estimator is already connected";
    return false;
}

bool FloatingBaseEstimator::isConnected() const
{
    if (!::yarp::os::Network::exists(baseState.port.getName())
        || !::yarp::os::Network::exists(contactState.port.getName())) {
        return false;
    }

    return ::yarp::os::Network::isConnected(floatingBasePort,
                                            baseState.port.getName())
           && ::yarp::os::Network::isConnected(feetContactPort,
                                               contactState.port.getName());
}

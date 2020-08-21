/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_YARP_HELPERS_H
#define SCENARIO_YARP_HELPERS_H

#include <Eigen/Dense>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/os/TypedReaderCallback.h>
#include <yarp/sig/Vector.h>

#include <memory>
#include <string>

namespace scenario::yarp::utils {
    struct BaseState;
    struct ContactState;
    struct ExternalWrenches;
    struct FloatingBaseEstimator;
    ::yarp::os::Property parseConfig(const std::string& config);
} // namespace scenario::yarp::utils

struct scenario::yarp::utils::BaseState
    : public ::yarp::os::TypedReaderCallback<::yarp::sig::VectorOf<double>>
{
    mutable std::recursive_mutex mutex;
    ::yarp::os::BufferedPort<::yarp::sig::VectorOf<double>> port;

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
    Eigen::Vector3d linVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d angVel = Eigen::Vector3d::Zero();

    void onRead(::yarp::sig::VectorOf<double>& datum) override;
};

struct scenario::yarp::utils::ContactState
    : public ::yarp::os::TypedReaderCallback<::yarp::os::Bottle>
{
    mutable std::recursive_mutex mutex;
    ::yarp::os::BufferedPort<::yarp::os::Bottle> port;

    struct
    {
        double left_foot = 0.0;
        double right_foot = 0.0;
    } forces;

    struct
    {
        int left_foot = 0;
        int right_foot = 0;
    } contacts;

    void onRead(::yarp::os::Bottle& datum) override;
};

struct scenario::yarp::utils::ExternalWrenches
    : public ::yarp::os::TypedReaderCallback<::yarp::sig::VectorOf<double>>
{
    //    mutable std::mutex mutex;
    //    ::yarp::os::BufferedPort<::yarp::sig::VectorOf<double>> port;

    struct ExternalWrench
        : public ::yarp::os::TypedReaderCallback<::yarp::sig::VectorOf<double>>
    {
        mutable std::recursive_mutex mutex;
        ::yarp::os::BufferedPort<::yarp::sig::VectorOf<double>> port;
        std::array<double, 6> wrench;

        void onRead(::yarp::sig::VectorOf<double>& datum) override;
    };

    ExternalWrench left_foot;
    ExternalWrench right_foot;
};

struct scenario::yarp::utils::FloatingBaseEstimator
{
    std::string rpcPort = "/base-estimator/rpc";
    std::string feetContactPort = "/base-estimator/feet_contact/state:o";
    std::string floatingBasePort = "/base-estimator/floating_base/state:o";
    std::string lFootExtWrenchPort =
        "/wholeBodyDynamics/left_foot/cartesianLinkWrench:o";
    std::string rFootExtWrenchPort =
        "/wholeBodyDynamics/right_foot/cartesianLinkWrench:o";

    BaseState baseState;
    ContactState contactState;
    ExternalWrenches externalWrenches;

    bool connect();
    bool isConnected() const;
};

#endif // SCENARIO_YARP_HELPERS_H

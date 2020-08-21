/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_YARP_ROBOTINTERFACE_H
#define SCENARIO_YARP_ROBOTINTERFACE_H

#include <memory>
#include <string>
#include <vector>

namespace yarp::dev {
    class IPositionControl;
    class IPositionDirect;
    class IVelocityControl;
    class ITorqueControl;
    class IPWMControl;
    class IControlMode;
    class ICurrentControl;
    class IEncoders;
    class IMotorEncoders;
    class IControlLimits;
    class IPidControl;
    class IAxisInfo;
} // namespace yarp::dev

namespace iDynTree {
    class KinDynComputations;
}

namespace scenario::yarp {
    class RobotInterface;
    struct Configuration;

    using JointIndex_Yarp = int;
    using JointIndex_iDynTree = int;
    using JointName = std::string;
} // namespace scenario::yarp

struct scenario::yarp::Configuration
{
    std::string robotName;
    std::string urdfFile;
    std::vector<std::string> jointNames;
    std::vector<std::string> controlBoardNames;
    std::string localName;
    std::array<double, 3> gravityVector = {0, 0, -9.80};

    inline size_t dofs() const { return jointNames.size(); }
};

class scenario::yarp::RobotInterface
{
public:
    //    RobotInterface() = delete;
    RobotInterface();
    RobotInterface(const Configuration& config);
    ~RobotInterface();

    /**
     * @brief Get the stored configuration
     *
     * @return A reference of the configuration this object refers to.
     */
    const Configuration& getConfiguration() const;

    /**
     * @brief Get the object to operate on the configured model
     *
     * @return A `shared_ptr` to the KinDynComputations object.
     */
    const std::shared_ptr<iDynTree::KinDynComputations> getKinDynComputations();

    bool startBaseEstimatorDevice();

    /**
     * @brief Get a Yarp interface
     *
     * The interface is lazy-evaluated. The handling of the memory is not
     * responsibility of the caller. It is handled internally.
     *
     * @param[out] interface The object that will contain the pointer to the
     * interface.
     * @return True for success, false otherwise.
     */
    template <typename T>
    bool getInterface(T*& interface);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

// Specialize the getInterface template
namespace scenario::yarp {
    template <>
    bool RobotInterface::getInterface(::yarp::dev::IControlMode*& interface);
    template <>
    bool
    RobotInterface::getInterface(::yarp::dev::IPositionControl*& interface);
    template <>
    bool RobotInterface::getInterface(::yarp::dev::IPositionDirect*& interface);
    template <>
    bool
    RobotInterface::getInterface(::yarp::dev::IVelocityControl*& interface);
    template <>
    bool RobotInterface::getInterface(::yarp::dev::ITorqueControl*& interface);
    template <>
    bool RobotInterface::getInterface(::yarp::dev::IPWMControl*& interface);
    template <>
    bool RobotInterface::getInterface(::yarp::dev::ICurrentControl*& interface);
    template <>
    bool RobotInterface::getInterface(::yarp::dev::IEncoders*& interface);
    template <>
    bool RobotInterface::getInterface(::yarp::dev::IMotorEncoders*& interface);
    template <>
    bool RobotInterface::getInterface(::yarp::dev::IControlLimits*& interface);
    template <>
    bool RobotInterface::getInterface(::yarp::dev::IPidControl*& interface);
    template <>
    bool RobotInterface::getInterface(::yarp::dev::IAxisInfo*& interface);
} // namespace scenario::yarp

#endif // SCENARIO_YARP_ROBOTINTERFACE_H

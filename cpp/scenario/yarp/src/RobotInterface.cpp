/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/yarp/RobotInterface.h"
#include "scenario/yarp/Log.h"

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingMatrices.h>
#include <iDynTree/Model/Indices.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <cassert>
#include <chrono>
#include <functional>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

using namespace scenario::yarp;

struct YarpInterfaces
{
    ::yarp::dev::IControlMode* iControlMode = nullptr;
    ::yarp::dev::IPositionControl* iPositionControl = nullptr;
    ::yarp::dev::IPositionDirect* iPositionDirect = nullptr;
    ::yarp::dev::IVelocityControl* iVelocityControl = nullptr;
    ::yarp::dev::ITorqueControl* iTorqueControl = nullptr;
    ::yarp::dev::IPWMControl* iPWMControl = nullptr;
    ::yarp::dev::ICurrentControl* iCurrentControl = nullptr;
    ::yarp::dev::IEncoders* iEncoders = nullptr;
    ::yarp::dev::IMotorEncoders* iMotorEncoders = nullptr;
    ::yarp::dev::IControlLimits* iControlLimits = nullptr;
    ::yarp::dev::IPidControl* iPidControl = nullptr;
    ::yarp::dev::IAxisInfo* iAxisInfo = nullptr;
};

class RobotInterface::Impl
{
public:
    ::yarp::os::Network network;
    YarpInterfaces yarpInterfaces;
    std::unique_ptr<::yarp::dev::PolyDriver> robotDevice;
    std::shared_ptr<iDynTree::KinDynComputations> kinDynComp;

    const Configuration config;

    Impl() = delete;
    Impl(const Configuration& configuration)
        : config(configuration)
    {}

    template <typename T>
    auto getInterfaceLazyEval(
        T*& interface,
        const std::unique_ptr<::yarp::dev::PolyDriver>& cbRemapper) -> T*;

    template <typename T>
    auto checkInterface(std::function<bool(T*)> getMeasurement) -> bool;

    bool initializeModel();
    bool initializeRemoteControlBoardRemapper();
};

RobotInterface::RobotInterface(const Configuration& config)
    : pImpl{std::make_unique<Impl>(config)}
{}

RobotInterface::~RobotInterface()
{
    if (pImpl->robotDevice) {
        if (!pImpl->robotDevice->close()) {
            sError << "Failed to close the RemoteControlBoardRemapper device";
        }
    }
}

const Configuration& RobotInterface::getConfiguration() const
{
    return pImpl->config;
}

const std::shared_ptr<iDynTree::KinDynComputations>
RobotInterface::getKinDynComputations()
{
    if (pImpl->kinDynComp) {
        return pImpl->kinDynComp;
    }

    if (!pImpl->initializeModel()) {
        sError << "Failed to initialize the KinDynComputations object";
        return nullptr;
    }

    return pImpl->kinDynComp;
}

bool RobotInterface::startBaseEstimatorDevice()
{
    // Initialize the RCBR
    if (!pImpl->robotDevice && !pImpl->initializeRemoteControlBoardRemapper()) {
        return false;
    }

    // Start WBD?

    // baseEstimatorV1 options
    ::yarp::os::Property options;

    ::yarp::dev::PolyDriverList devices;
    //    devices.push()

    // Allocate the interface driver for the baseEstimatorV1
    auto baseEstimatorV1 = std::make_unique<::yarp::dev::PolyDriver>();

    // Fill a PolyDriverList
    // View the yarp::dev::IMultipleWrapper
    // Manually attach

    // Open the interface driver
    if (!(baseEstimatorV1->open(options) && baseEstimatorV1->isValid())) {
        sError << "Failed to open the baseEstimatorV1 with the options passed";
        return false;
    }

    // TODO: RPC calls

    return true;
}

// =======================
// Template specialization
// =======================

template <>
bool RobotInterface::getInterface(::yarp::dev::IControlMode*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iControlMode;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));

        //        auto getMeas = [&](int* input) -> bool {
        //            return storedInterface->getControlModes(input);
        //        };

        //        // Check if it works fine
        //        if (!pImpl->checkInterface<int>(getMeas)) {
        //            return false;
        //        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(::yarp::dev::IPositionControl*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iPositionControl;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }

        auto getMeas = [&](int* input) -> bool {
            return storedInterface->getAxes(input);
        };

        // Check if it works fine
        if (!pImpl->checkInterface<int>(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(::yarp::dev::IPositionDirect*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iPositionDirect;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }

        auto getMeas = [&](double* input) -> bool {
            return storedInterface->getRefPositions(input);
        };

        // Check if it works fine
        if (!pImpl->checkInterface<double>(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(::yarp::dev::IVelocityControl*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iVelocityControl;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }

        auto getMeas = [&](double* input) -> bool {
            return storedInterface->getRefVelocities(input);
        };

        // Check if it works fine
        if (!pImpl->checkInterface<double>(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(::yarp::dev::ITorqueControl*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iTorqueControl;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }

        auto getMeas = [&](double* input) -> bool {
            return storedInterface->getTorques(input);
        };

        // Check if it works fine
        if (!pImpl->checkInterface<double>(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(::yarp::dev::IPWMControl*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iPWMControl;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }

        auto getMeas = [&](double* input) -> bool {
            return storedInterface->getDutyCycles(input);
        };

        // Check if it works fine
        if (!pImpl->checkInterface<double>(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(::yarp::dev::ICurrentControl*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iCurrentControl;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }

        auto getMeas = [&](double* input) -> bool {
            return storedInterface->getCurrents(input);
        };

        // Check if it works fine
        if (!pImpl->checkInterface<double>(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(::yarp::dev::IEncoders*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iEncoders;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }

        auto getMeas = [&](double* input) -> bool {
            return storedInterface->getEncoders(input);
        };

        // Check if it works fine
        if (!pImpl->checkInterface<double>(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(::yarp::dev::IMotorEncoders*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iMotorEncoders;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }

        auto getMeas = [&](double* input) -> bool {
            return storedInterface->getMotorEncoders(input);
        };

        // Check if it works fine
        if (!pImpl->checkInterface<double>(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(::yarp::dev::IControlLimits*& interface)
{
    interface = pImpl->getInterfaceLazyEval(
        pImpl->yarpInterfaces.iControlLimits, pImpl->robotDevice);
    return interface;
}

template <>
bool RobotInterface::getInterface(::yarp::dev::IPidControl*& interface)
{
    interface = pImpl->getInterfaceLazyEval(pImpl->yarpInterfaces.iPidControl,
                                            pImpl->robotDevice);
    return interface;
}

template <>
bool RobotInterface::getInterface(::yarp::dev::IAxisInfo*& interface)
{
    interface = pImpl->getInterfaceLazyEval(pImpl->yarpInterfaces.iAxisInfo,
                                            pImpl->robotDevice);
    return interface;
}

// ====
// Impl
// ====

template <typename T>
auto RobotInterface::Impl::getInterfaceLazyEval(
    T*& interface,
    const std::unique_ptr<::yarp::dev::PolyDriver>& cbRemapper) -> T*
{
    if (!interface) {
        // Lazy-initialize the RemoteControlBoardRemapper device
        if (!cbRemapper) {
            if (!this->initializeRemoteControlBoardRemapper()) {
                sError << "Failed to initialize the "
                          "RemoteControlBoardRemapper";
                return nullptr;
            }
        }
        // Ask the interface from the device
        if (!robotDevice->view(interface)) {
            sError << "Failed to view the interface";
            return nullptr;
        }
    }

    // Return the raw pointer
    return interface;
}

template <typename T>
auto RobotInterface::Impl::checkInterface(
    std::function<bool(T*)> getMeasurement) -> bool
{
    // This method is used only on interfaces which get measurements. There
    // is an interval right after the allocation of the
    // RemoteControlBoardRemapper when a get*() calls from a viewed
    // interface will return false. This failure is not due to a wrong usage
    // of the interface, but rather to a call before the interface actually
    // receives data.
    unsigned counter = 0;
    constexpr unsigned maxIter = 2000;
    std::vector<T> buffer(config.dofs(), 0.0);

    while (!getMeasurement(buffer.data())) {
        if (++counter == maxIter) {
            sError << "Failed to get a measurement while initializing the "
                      "interface ";
            return false;
        }

        // Sleep for some while
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    return true;
}

bool RobotInterface::Impl::initializeModel()
{
    if (kinDynComp) {
        sWarning << "KinDynComputations has been already initialized";
        return true;
    }

    // Allocate the object
    kinDynComp = std::make_shared<iDynTree::KinDynComputations>();

    // Explicitly set the velocity representation
    if (!kinDynComp->setFrameVelocityRepresentation(
            iDynTree::MIXED_REPRESENTATION)) {
        sError << "Failed to set the velocity representation";
        return false;
    }

    // Use ModelLoader to load the reduced model
    bool ok = false;
    iDynTree::ModelLoader mdlLoader;

    if (config.jointNames.empty()) {
        ok = mdlLoader.loadModelFromFile(config.urdfFile);
    }
    else {
        ok = mdlLoader.loadReducedModelFromFile(config.urdfFile,
                                                config.jointNames);
    }

    if (!ok) {
        sError << "Impossible to load the model from '" << config.urdfFile
               << "'. Possible causes: file not found, or the joints "
               << "list contains an entry not present in the urdf model.";
        return false;
    }

    // Add the loaded model to the KinDynComputations object
    return kinDynComp->loadRobotModel(mdlLoader.model());
}

bool RobotInterface::Impl::initializeRemoteControlBoardRemapper()
{
    // Initialize the network
    if (!::yarp::os::Network::initialized()
        || !::yarp::os::Network::checkNetwork(5.0)) {
        sError << "YARP server wasn't found active.";
        return false;
    }

    // RemoteControlBoardRemapper options
    ::yarp::os::Property options;

    // Name of the device
    options.put("device", "remotecontrolboardremapper");

    // Setup the joint serialization
    const std::vector<std::string> jointSerialization =
        [&]() -> std::vector<std::string> {
        if (!config.jointNames.empty()) {
            return config.jointNames;
        }

        // Use the model to get the joint serialization
        if (!kinDynComp && !initializeModel()) {
            return {};
        }

        std::vector<std::string> serialization;
        serialization.reserve(kinDynComp->model().getNrOfJoints());

        for (size_t idx = 0; idx < kinDynComp->model().getNrOfJoints(); ++idx) {
            if (kinDynComp->model().getJoint(idx)->getNrOfDOFs() > 0) {
                serialization.push_back(kinDynComp->model().getJointName(idx));
            }
        }

        return serialization;
    }();

    if (jointSerialization.empty()) {
        sError << "Failed to get the joint serialization";
        return false;
    }

    // Controlled joints (axes)
    options.put("axesNames", [&]() -> ::yarp::os::Value {
        ::yarp::os::Bottle axesNames;
        ::yarp::os::Bottle& axesList = axesNames.addList();

        for (const auto& name : jointSerialization) {
            axesList.addString(name);
        }

        sMessage << axesList.toString();

        return axesNames.get(0);
    }());

    // ControlBoard names
    options.put("remoteControlBoards", [&]() -> ::yarp::os::Value {
        ::yarp::os::Bottle remoteControlBoards;
        ::yarp::os::Bottle& remoteControlBoardsList =
            remoteControlBoards.addList();

        for (const auto& cb : config.controlBoardNames) {
            remoteControlBoardsList.addString("/" + config.robotName + "/"
                                              + cb);
        }
        return remoteControlBoards.get(0);
    }());

    // Prefix of the openened ports
    // In this case appending the unique id is necessary, since multiple
    // configuration can share some ControlBoard in their
    // RemoteControlBoardRemappers. In this case, it is not possible
    // using the same prefix for all the RemoteControlBoardRemapper
    // devices.
    options.put("localPortPrefix", config.localName);
    // TODO
    //                config.localName + "/" + config.getUniqueId());

    // Misc options
    ::yarp::os::Property& rcbOpts =
        options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    rcbOpts.put("writeStrict", "on");

    sDebug << options.toString();

    // Allocate the interface driver for the RemoteControlBoardRemappper
    auto rcbr = std::make_unique<::yarp::dev::PolyDriver>();

    // Open the interface driver
    if (!(rcbr->open(options) && rcbr->isValid())) {
        sError << "Failed to open the RemoteControlBoardRemapper with the "
                  "options passed";
        return false;
    }

    robotDevice = std::move(rcbr);
    return true;
}

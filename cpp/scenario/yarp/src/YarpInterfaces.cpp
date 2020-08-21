/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/yarp/YarpInterfaces.h"
#include "scenario/yarp/Log.h"

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

class YarpInterfaces::Impl
{
public:
    size_t dofs;
    ::yarp::os::Network network;
    ::yarp::dev::PolyDriver* robotDevice;

    struct
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
    } yarpInterfaces;

    template <typename T>
    auto getInterfaceLazyEval(T*& interface,
                              ::yarp::dev::PolyDriver* cbRemapper) -> T*;

    template <typename T>
    auto checkInterface(std::function<bool(T*)> getMeasurement) -> bool;

    //    bool initializeModel();
    //    bool initializeRemoteControlBoardRemapper();
};

YarpInterfaces::YarpInterfaces(const size_t dofs)
    : pImpl{std::make_unique<Impl>()}
{
    pImpl->dofs = dofs;
}

YarpInterfaces::~YarpInterfaces() = default;

bool YarpInterfaces::storeRCBR(::yarp::dev::PolyDriver* rcbr)
{
    if (!(rcbr && rcbr->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    pImpl->robotDevice = rcbr; // TODO rename
    return true;
}

// =======================
// Template specialization
// =======================

template <>
bool YarpInterfaces::getInterface(::yarp::dev::IControlMode*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iControlMode;

    if (!storedInterface) {
        // Get the interface
        //        if (!pImpl->getInterfaceLazyEval(storedInterface,
        //        pImpl->robotDevice)) {
        //            return false;
        //        }

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
            return false;
        }

        //        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto getMeas = [&](int* input) -> bool {
            return storedInterface->getControlModes(input);
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
bool YarpInterfaces::getInterface(::yarp::dev::IPositionControl*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iPositionControl;

    if (!storedInterface) {
        // Get the interface
        //        if (!pImpl->getInterfaceLazyEval(storedInterface,
        //        pImpl->robotDevice)) {
        //            return false;
        //        }

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
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
bool YarpInterfaces::getInterface(::yarp::dev::IPositionDirect*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iPositionDirect;

    if (!storedInterface) {
        // Get the interface
        //        if (!pImpl->getInterfaceLazyEval(storedInterface,
        //        pImpl->robotDevice)) {
        //            return false;
        //        }

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
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
bool YarpInterfaces::getInterface(::yarp::dev::IVelocityControl*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iVelocityControl;

    if (!storedInterface) {
        // Get the interface
        //        if (!pImpl->getInterfaceLazyEval(storedInterface,
        //        pImpl->robotDevice)) {
        //            return false;
        //        }

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
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
bool YarpInterfaces::getInterface(::yarp::dev::ITorqueControl*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iTorqueControl;

    if (!storedInterface) {
        // Get the interface
        //        if (!pImpl->getInterfaceLazyEval(storedInterface,
        //        pImpl->robotDevice)) {
        //            return false;
        //        }

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
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
bool YarpInterfaces::getInterface(::yarp::dev::IPWMControl*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iPWMControl;

    if (!storedInterface) {
        // Get the interface
        //        if (!pImpl->getInterfaceLazyEval(storedInterface,
        //        pImpl->robotDevice)) {
        //            return false;
        //        }

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
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
bool YarpInterfaces::getInterface(::yarp::dev::ICurrentControl*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iCurrentControl;

    if (!storedInterface) {
        // Get the interface
        //        if (!pImpl->getInterfaceLazyEval(storedInterface,
        //        pImpl->robotDevice)) {
        //            return false;
        //        }

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
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
bool YarpInterfaces::getInterface(::yarp::dev::IEncoders*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iEncoders;

    if (!storedInterface) {
        // Get the interface
        //        if (!pImpl->getInterfaceLazyEval(storedInterface,
        //        pImpl->robotDevice)) {
        //            return false;
        //        }

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
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
bool YarpInterfaces::getInterface(::yarp::dev::IMotorEncoders*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iMotorEncoders;

    if (!storedInterface) {

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
            return false;
        }

        //        // Get the interface
        //        if (!pImpl->getInterfaceLazyEval(storedInterface,
        //        pImpl->robotDevice)) {
        //            return false;
        //        }

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
bool YarpInterfaces::getInterface(::yarp::dev::IControlLimits*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iControlLimits;

    if (!storedInterface) {

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
            return false;
        }
    }

    //    interface = pImpl->getInterfaceLazyEval(
    //        pImpl->yarpInterfaces.iControlLimits, pImpl->robotDevice);
    //    return interface;

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool YarpInterfaces::getInterface(::yarp::dev::IPidControl*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iPidControl;

    if (!storedInterface) {

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);

    //    interface =
    //    pImpl->getInterfaceLazyEval(pImpl->yarpInterfaces.iPidControl,
    //                                            pImpl->robotDevice);
    //    return interface;
}

template <>
bool YarpInterfaces::getInterface(::yarp::dev::IAxisInfo*& interface)
{
    if (!(pImpl->robotDevice && pImpl->robotDevice->isValid())) {
        sError << "The pointer to the RCBR is not valid";
        return false;
    }

    auto& storedInterface = pImpl->yarpInterfaces.iAxisInfo;

    if (!storedInterface) {

        if (!pImpl->robotDevice->view(storedInterface)) {
            sError << "Failed to view the interface";
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);

    //    interface =
    //    pImpl->getInterfaceLazyEval(pImpl->yarpInterfaces.iAxisInfo,
    //                                            pImpl->robotDevice);
    //    return interface;
}

// ====
// Impl
// ====

template <typename T>
auto YarpInterfaces::Impl::getInterfaceLazyEval(
    T*& interface,
    ::yarp::dev::PolyDriver* cbRemapper) -> T*
{
    if (!interface) {
        // Lazy-initialize the RemoteControlBoardRemapper device
        //        if (!cbRemapper) {
        //            if (!this->initializeRemoteControlBoardRemapper()) {
        //                sError << "Failed to initialize the "
        //                          "RemoteControlBoardRemapper";
        //                return nullptr;
        //            }
        //        }
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
auto YarpInterfaces::Impl::checkInterface(
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
    std::vector<T> buffer(dofs, 0.0);

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

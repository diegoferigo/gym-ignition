/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_YARP_LOG
#define SCENARIO_YARP_LOG

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#define sError ::yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).error()
#define sWarning ::yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).warning()
#define sMessage ::yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).info()
#define sDebug ::yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).debug()
#define sLog ::yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).trace()

#endif // SCENARIO_YARP_LOG

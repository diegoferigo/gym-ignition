//%module(moduleimport="import $module") scenario_bindings
//%module(package="scenario.gazebo") scenario_yarp
//%module(package="scenario.bindings", moduleimport="from scenario import gazebo") gazebo
%module(moduleimport="import $module") scenario_yarp

%{
#define SWIG_FILE_WITH_INIT
// base
//#include "scenario/base/Joint.h"
//#include "scenario/base/Link.h"
//#include "scenario/base/Model.h"
//#include "scenario/base/World.h"
// yarp
#include "scenario/yarp/Joint.h"
#include "scenario/yarp/Link.h"
#include "scenario/yarp/Model.h"
#include "scenario/yarp/utils.h"
#include "scenario/yarp/World.h"
%}

%naturalvar;

%import "../base/base.i"

// STL classes
%include <stdint.i>
%include <std_pair.i>
%include <std_array.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_shared_ptr.i>

// Convert python list to std::vector
//%template(VectorI) std::vector<int>;
//%template(VectorU) std::vector<size_t>;
//%template(VectorF) std::vector<float>;
//%template(VectorD) std::vector<double>;
//%template(VectorS) std::vector<std::string>;

// Convert python list to std::array
//%template(Array3d) std::array<double, 3>;
//%template(Array4d) std::array<double, 4>;
//%template(Array6d) std::array<double, 6>;

// Pair instantiation
//%template(PosePair) std::pair<std::array<double, 3>, std::array<double, 4>>;

// ScenarI/O templates
//%template(VectorOfLinks) std::vector<scenario::base::LinkPtr>;
//%template(VectorOfJoints) std::vector<scenario::base::JointPtr>;
//%template(Vector_contact) std::vector<scenario::base::Contact>;
//%template(Vector_contact_point) std::vector<scenario::base::ContactPoint>;

// List of Doxygen typemaps
// TODO: se usiamo vector double -> tuple, problemi con inputs? Union[Tuple[float, ...], List[float]]?
// TODO: inject from typing import Tuple above otherwise methods like Link.position do not recognize
// TODO: matrices?
// TODO: inject to_numpy() from_numpy() methods?
//%typemap(doctype) std::array<double, 3> "Tuple[float, float, float]";
//%typemap(doctype) std::array<double, 4> "Tuple[float, float, float, float]";
//%typemap(doctype) std::aBaserray<double, 6> "Tuple[float, float, float, float, float, float,]";
//%typemap(doctype) std::vector<double> "Tuple[float, ...]";
//%typemap(doctype) std::vector<std::string> "Tuple[str]";  // TODO
//%typemap(doctype) std::vector<scenario::base::Contact> "Tuple[Contact, ...]";
//%typemap(doctype) std::vector<scenario::base::ContactPoint> "Tuple[ContactPoint, ...]";

//%pythonbegin %{
//from typing import Tuple
//%}

// Rename all methods to undercase with _ separators excluding the classes.
// Keep all template instantations above.
// TODO: fare uguale per gympp
//%rename("%(undercase)s") "";
//%rename("") PID;
//%rename("") Pose;
//%rename("") Link;
//%rename("") Joint;
//%rename("") Model;
//%rename("") World;
//%rename("") Limit;
//%rename("") Contact;
//%rename("") JointType;
//%rename("") JointLimit;
//%rename("") ContactPoint;
//%rename("") ECMSingleton;
//%rename("") JointControlMode;
//%rename("") GazeboClassicRunner;

// Public helpers
%include "scenario/yarp/utils.h"

// Other templates for ScenarI/O APIs
//%shared_ptr(scenario::base::Joint)
//%shared_ptr(scenario::base::Link)
//%shared_ptr(scenario::base::Model)
//%shared_ptr(scenario::base::World)

%shared_ptr(scenario::yarp::Joint)
%shared_ptr(scenario::yarp::Link)
%shared_ptr(scenario::yarp::Model)
%shared_ptr(scenario::yarp::World)

////%rename("JointYarpBase") scenario::base::Joint;
////%rename("LinkYarpBase") scenario::base::Link;
////%rename("ModelYarpBase") scenario::base::Model;
////%rename("WorldYarpBase") scenario::base::World;
//%rename("JointBase") scenario::base::Joint;
//%rename("LinkBase") scenario::base::Link;
//%rename("ModelBase") scenario::base::Model;
//%rename("WorldBase") scenario::base::World;

// Ignored methods
%ignore scenario::yarp::Joint::initialize;
%ignore scenario::yarp::Link::initialize;
//%ignore scenario::gazebo::Model::initialize;
//%ignore scenario::gazebo::World::initialize;

// ScenarI/O base headers
//%include "scenario/base/Joint.h"
//%include "scenario/base/Link.h"
//%include "scenario/base/Model.h"
//%include "scenario/base/World.h"

// ScenarI/O headers
%include "scenario/yarp/Joint.h"
%include "scenario/yarp/Link.h"
%include "scenario/yarp/Model.h"
%include "scenario/yarp/World.h"

//%pythoncode %{
//    JointBase = JointYarpBase
//    LinkBase = LinkYarpBase
//    ModelBase = ModelYarpBase
//    WorldBase = WorldYarpBase
//%}

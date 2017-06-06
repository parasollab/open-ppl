#include "Robot.h"

#include <algorithm>
#include <sstream>

#include "Actuator.h"
#include "DynamicsModel.h"
#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Environment/Environment.h"
#include "Utilities/XMLNode.h"

// Temporaries for hard-coded stuff
#include "Behaviors/Controllers/ControlSetGenerators.h"
#include "Behaviors/Controllers/SimpleController.h"
#include "nonstd/io.h"

/*------------------------------ Construction --------------------------------*/

Robot::
Robot(MPProblem* const _p, XMLNode& _node) : m_problem(_p) {
  // Get the unique robot label.
  m_label = _node.Read("label", true, "", "Unique robot label");

  // Get the (optional) holonomicness, assuming holononmic.
  m_nonholonomic = _node.Read("nonholonomic", false, false, "Is the robot "
      "nonholonomic?");

  // Get the (optional) car-likeness, assuming not car-like.
  m_carlike = _node.Read("carlike", false, false, "Is the robot car-like?");

  // Get the multibody file name and make sure it exists.
  const std::string path = GetPathName(_node.Filename());
  const std::string file = _node.Read("filename", true, "", "Robot file name");
  const std::string filename = path + file;
  if(!FileExists(filename))
    throw ParseException(_node.Where(), "File '" + filename + "' does not exist");

  // If we got an XML file, use that parsing mechanism.
  if(filename.find(".xml") != string::npos)
    ReadXMLFile(filename);
  // Otherwise we got a multibody file, which cannot specify dynamics options
  // like actuators and controls. Assume some defaults for these.
  else {
    ReadMultibodyFile(filename);

    // Set up a single, velocity-based actuator for all DOF. As this robot is
    // holonomic, we will only use the actuator in simulation.
    std::vector<double> reverse(m_multibody->DOF(), -1),
                        forward(m_multibody->DOF(),  1);
    m_actuators["default"] = new Actuator(this, "default",
          IsNonholonomic() ? Actuator::Force : Actuator::Velocity);
    m_actuators["default"]->SetLimits(reverse, forward);
    m_actuators["default"]->SetMaxForce(1);

    // Use the simplest controller.
    m_controller = new SimpleController(this, 1, 1);

    // If the robot is nonholonomic, we need a discrete control set for now to
    // get the kinodynamic extender working.
    if(IsNonholonomic())
      m_controller->SetControlSet(SimpleControlSetGenerator(this));
  }

  // Create dynamics model if the robot is nonholonomic.
  if(IsNonholonomic())
    m_dynamicsModel = new DynamicsModel(this, nullptr);

  // The agent should be initialized by the Simulation object to avoid long
  // compile times for this object.
  m_agentLabel = _node.Read("agent", false, "", "Label for the agent type");
}


Robot::
Robot(MPProblem* const _p, ActiveMultiBody* const _mb, const std::string& _label)
    : m_problem(_p), m_label(_label), m_multibody(_mb) {
  m_multibody->InitializeDOFs(m_problem->GetEnvironment()->GetBoundary());
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));

  InitializePlanningSpaces();
}


Robot::
~Robot() noexcept {
  delete m_multibody;
  delete m_agent;
  delete m_controller;
  delete m_dynamicsModel;

  for(auto& a : m_actuators)
    delete a.second;
}

/*---------------------------------- I/O -------------------------------------*/

void
Robot::
ReadXMLFile(const std::string& _filename) {
  XMLNode node(_filename, "Robot");

  // Read attributes of the robot node.
  m_maxLinearVelocity = node.Read("maxLinearVelocity", false, 10., 0.,
      std::numeric_limits<double>::max(), "The robot's maximum linear velocity");
  m_maxAngularVelocity = node.Read("maxAngularVelocity", false, 1., 0.,
      std::numeric_limits<double>::max(), "The robot's maximum angular velocity");

  for(auto& child : node) {
    if(child.Name() == "Multibody") {
      // Read the multibody file. Eventually we'll go full XML and pass the
      // child node directly to the multibody instead.
      const std::string mbFile = child.Read("filename", true, "", "Name of the "
          "robot's multibody file");
      ReadMultibodyFile(GetPathName(_filename) + mbFile);
    }
    else if(child.Name() == "Actuator") {
      // Parse the actuator.
      auto actuator = new Actuator(this, child);
      m_actuators[actuator->GetLabel()] = actuator;
    }
    else if(child.Name() == "Controller") {
      // Read downcased controller type.
      std::string controllerType = child.Read("type", true, "", "Label "
          "of the controller type to use");
      std::transform(controllerType.begin(), controllerType.end(),
          controllerType.begin(), ::tolower);

      // Setup the appropriate controller type.
      if(controllerType == "simple")
        m_controller = new SimpleController(this, child);
      else
        throw ParseException(child.Where(), "Unknown controller label '" +
            controllerType + "'. Currently only 'simple' is supported.");
    }
  }

  // Throw errors for any unrequested attributes or nodes.
  node.WarnAll(true);
}


void
Robot::
ReadMultibodyFile(const std::string& _filename) {
  // Open the file.
  CountingStreamBuffer cbs(_filename);
  std::istream ifs(&cbs);

  // Parse the file to get the robot's geometry.
  m_multibody = new ActiveMultiBody;
  m_multibody->Read(ifs, cbs);

  // Initialize the DOF limits and set the robot to a zero starting configuration.
  m_multibody->InitializeDOFs(m_problem->GetEnvironment()->GetBoundary());
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));

  InitializePlanningSpaces();
}


void
Robot::
InitializePlanningSpaces() {
  const size_t dof = m_multibody->DOF();
  const auto& dofInfo = m_multibody->GetDofInfo();

  // Create the configuration space boundary.
  delete m_cspace;
  m_cspace = new CSpaceBoundingBox(dof);

  for(size_t i = 0; i < dof; ++i)
    m_cspace->SetRange(i, dofInfo[i].range);

  // If this is a holonomic robot, we are done.
  if(!m_nonholonomic)
    return;

  // Create the velocity space boundary.
  delete m_vspace;
  m_vspace = new CSpaceBoundingBox(dof);

  const size_t pos = m_multibody->PosDOF(),
               ori = m_multibody->OrientationDOF();

  for(size_t i = 0; i < pos; ++i)
    m_vspace->SetRange(i, -m_maxLinearVelocity, m_maxLinearVelocity);

  for(size_t i = pos; i < pos + ori; ++i)
    m_vspace->SetRange(i, -m_maxAngularVelocity, m_maxAngularVelocity);

  /// @TODO Set up a way to specify velocity limits for each joint.
  for(size_t i = pos + ori; i < dof; ++i)
    m_vspace->SetRange(i, -1, 1);
}

/*--------------------------- Planning Interface -----------------------------*/

const CSpaceBoundingBox*
Robot::
GetCSpace() const noexcept {
  return m_cspace;
}


const CSpaceBoundingBox*
Robot::
GetVSpace() const noexcept {
  return m_vspace;
}

/*------------------------- Simulation Interface -----------------------------*/

void
Robot::
Step(const double _dt) {
  // Update the agent's perception of the world.
  /// @TODO Add percept model
  //if(m_percept)
  //  m_percept->Update();


  // Run the agent's decision-making routine. The agent will apply controls as
  // required to execute its decision.
  if(m_agent)
    m_agent->Step(_dt);
}


MPProblem*
Robot::
GetMPProblem() const noexcept {
  return m_problem;
}

/*--------------------------- Geometry Accessors -----------------------------*/

ActiveMultiBody*
Robot::
GetMultiBody() noexcept {
  return m_multibody;
}


const ActiveMultiBody*
Robot::
GetMultiBody() const noexcept {
  return m_multibody;
}

/*------------------------------ Agent Accessors -----------------------------*/

Agent*
Robot::
GetAgent() noexcept {
  return m_agent;
}


void
Robot::
SetAgent(Agent* const _a) noexcept {
  delete m_agent;
  m_agent = _a;
}

/*---------------------------- Control Accessors -----------------------------*/

ControllerMethod*
Robot::
GetController() noexcept {
  return m_controller;
}


void
Robot::
SetController(ControllerMethod* const _c) noexcept {
  delete m_controller;
  m_controller = _c;
}

/*---------------------------- Actuator Accessors ----------------------------*/

Actuator*
Robot::
GetActuator(const std::string& _label) noexcept {
  return m_actuators[_label];
}


const std::unordered_map<std::string, Actuator*>&
Robot::
GetActuators() noexcept {
  return m_actuators;
}

/*---------------------------- Dynamics Accessors ----------------------------*/

DynamicsModel*
Robot::
GetDynamicsModel() noexcept {
  return m_dynamicsModel;
}


void
Robot::
SetDynamicsModel(btMultiBody* const _m) {
  delete m_dynamicsModel;
  m_dynamicsModel = new DynamicsModel(this, _m);
}

/*------------------------------- Other --------------------------------------*/

bool
Robot::
IsNonholonomic() const noexcept {
  return m_nonholonomic;
}


bool
Robot::
IsCarlike() const noexcept {
  return m_carlike;
}


double
Robot::
GetMaxLinearVelocity() const noexcept {
  return m_maxLinearVelocity;
}


double
Robot::
GetMaxAngularVelocity() const noexcept {
  return m_maxAngularVelocity;
}


const std::string&
Robot::
GetLabel() const noexcept {
  return m_label;
}

/*---------------------------------- Debug -----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Robot& _r) {
  _os << "Robot '" << _r.GetLabel() << "'"
      << "\n\tDOF:      " << _r.GetMultiBody()->DOF()
      << "\n\tPosDOF:   " << _r.GetMultiBody()->PosDOF()
      << "\n\tOriDOF:   " << _r.GetMultiBody()->OrientationDOF()
      << "\n\tJointDOF: " << _r.GetMultiBody()->JointDOF()
      << "\nActuators:\n";
  for(const auto& a : _r.m_actuators)
    _os << *a.second << "\n";
  return _os << std::endl;
}

/*----------------------------------------------------------------------------*/

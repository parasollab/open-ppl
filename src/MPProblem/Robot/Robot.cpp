#include "Robot.h"

#include "Actuator.h"
#include "DynamicsModel.h"
#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Boundary.h"

// Temporaries for hard-coded stuff
#include "ControlGenerators.h"
#include "Behaviors/Agents/PathFollowingAgent.h"
#include "Behaviors/Controllers/SimpleController.h"


/*------------------------------ Construction --------------------------------*/

Robot::
Robot(MPProblem* const _p, XMLNode& _node, const Boundary* const _b) :
    m_problem(_p) {
  // Get the unique robot label.
  m_label = _node.Read("label", true, "", "Unique robot label");

  // Get the (optional) holonomicness, assuming holononmic.
  m_nonholonomic = _node.Read("nonholonomic", false, false, "Is the robot "
      "nonholonomic?");

  // Get the geometry file name and make sure it exists.
  const std::string path = GetPathName(_node.Filename());
  const std::string file = _node.Read("filename", true, "", "Robot file name");
  const std::string filename = path + file;
  if(!FileExists(filename))
    throw ParseException(filename, "File does not exist");

  // Open the file.
  CountingStreamBuffer cbs(filename);
  std::istream ifs(&cbs);

  // Parse the file to get the robot's geometry.
  m_multibody = new ActiveMultiBody;
  m_multibody->Read(ifs, cbs);
  m_multibody->InitializeDOFs(_b);

  // Set up actuators.
  /// @TODO Read in actuators instead of hard-coding. Currently we assume a
  ///       single actuator that can generate thrust for any DOF of a rigid body.
  std::vector<double> reverse(m_multibody->DOF()), forward(m_multibody->DOF());
  for(size_t i = 0; i < m_multibody->PosDOF(); ++i) {
    reverse[i] = -.5;
    forward[i] = .5;
  }
  for(size_t i = m_multibody->PosDOF(); i < m_multibody->DOF(); ++i) {
    reverse[i] = -.3;
    forward[i] = .3;
  }
  m_actuators.push_back(new Actuator(this));
  m_actuators.back()->SetLimits(reverse, forward);
  m_actuators.back()->SetMaxForce(.5);

  // Create control set.
  /// @TODO Read in control generation method instead of hard-coding. Currently
  ///       we assume a simple control set.
  SimpleControlGenerator simple;
  SetControlSet(simple.GenerateDiscreteSet(this));

  // Create controller.
  /// @TODO Read in controller type instead of hard-coding. Currently we assume
  ///       the simplest controller.
  m_controller = new SimpleController(this, .1);

  // Create agent.
  /// @TODO Read in agent type instead of hard-coding. Currently we assume a
  ///       path-following agent.
  m_agent = new PathFollowingAgent(this);

  // Create default (unattached to any simulator) dynamics model if the robot is
  // nonholonomic.
  if(IsNonholonomic())
    m_dynamicsModel = new DynamicsModel(this, nullptr);

  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));
}


Robot::
~Robot() noexcept {
  delete m_multibody;
  delete m_agent;
  delete m_controlSet;
  delete m_controlSpace;
  delete m_controller;
  delete m_dynamicsModel;

  for(auto a : m_actuators)
    delete a;
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

ControlSet*
Robot::
GetControlSet() noexcept {
  return m_controlSet;
}


void
Robot::
SetControlSet(ControlSet* const _c) noexcept {
  delete m_controlSet;
  m_controlSet = _c;
}


ControlSpace*
Robot::
GetControlSpace() noexcept {
  return m_controlSpace;
}


void
Robot::
SetControlSpace(ControlSpace* const _c) noexcept {
  delete m_controlSpace;
  m_controlSpace = _c;
}


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
GetActuator(const size_t _i) noexcept {
  return m_actuators[_i];
}


const std::vector<Actuator*>&
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

/*----------------------------------------------------------------------------*/

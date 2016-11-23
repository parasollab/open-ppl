#include "Robot.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Boundary.h"

#include "MPProblem/Robot/Agent/NullAgent.h"
#include "MPProblem/Robot/Control/Controller.h"
#include "MPProblem/Robot/Control/ControlGenerators.h"
#include "MPProblem/Robot/Control/SteeringFunctions.h"


/*------------------------------ Construction --------------------------------*/

Robot::
Robot() = default;


Robot::
Robot(XMLNode& _node, Boundary* const _b) {
  // Get the unique robot label.
  m_label = _node.Read("label", true, "", "Unique robot label");

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
  m_actuators.emplace_back(this);
  m_actuators.back().SetLimits(reverse, forward);
  m_actuators.back().SetMaxForce(.5);

  // Create a controller.
  /// @TODO Read in control generation method instead of hard-coding. Currently
  ///       we assume a very simple control model and PID feedback steering.
  m_controller = new Controller(this);
  m_controller->ComputeControls(SimpleControlGenerator());
  // Use gains of .5, .01, .1 for the P, I, and D terms.
  m_controller->SetSteeringFunction(new PIDFeedback(.5, .01 , .1));

  // Create an agent for the robot.
  /// @TODO Parse agent type rather than hard-coding.
  m_agent = new NullAgent(this);
}


Robot::
~Robot() {
  delete m_multibody;
  delete m_agent;
  delete m_controller;
}

/*------------------------- Simulation Interface -----------------------------*/

void
Robot::
Step() {
  /// @TODO Add percept model
  //m_percept->Update();
  m_agent->Step();
}

/*--------------------------- Geometry Accessors -----------------------------*/

ActiveMultiBody*
Robot::
GetMultiBody() {
  return m_multibody;
}


const ActiveMultiBody*
Robot::
GetMultiBody() const {
  return m_multibody;
}

/*--------------------------- Controller Accessors ---------------------------*/

Agent*
Robot::
GetAgent() {
  return m_agent;
}


void
Robot::
SetAgent(Agent* const _a) {
  delete m_agent;
  m_agent = _a;
}

/*--------------------------- Controller Accessors ---------------------------*/

Controller*
Robot::
GetController() {
  return m_controller;
}


void
Robot::
SetController(Controller* const _c) {
  delete m_controller;
  m_controller = _c;
}

/*---------------------------- Actuator Accessors ----------------------------*/

Actuator&
Robot::
GetActuator(const size_t _i) {
  return m_actuators[_i];
}


std::vector<Actuator>&
Robot::
GetActuators() {
  return m_actuators;
}

/*---------------------------- Dynamics Accessors ----------------------------*/

btMultiBody*
Robot::
GetDynamicsModel() {
  return m_dynamicsModel;
}


void
Robot::
SetDynamicsModel(btMultiBody* const _m) {
  m_dynamicsModel = _m;
}

/*----------------------------------------------------------------------------*/

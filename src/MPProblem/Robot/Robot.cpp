#include "Robot.h"

#include "Actuator.h"
#include "ControlGenerators.h"

#include "Behaviors/Agent/Agent.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Boundary.h"


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
  m_actuators.push_back(new Actuator(this));
  m_actuators.back()->SetLimits(reverse, forward);
  m_actuators.back()->SetMaxForce(.5);

  // Create control set.
  /// @TODO Read in control generation method instead of hard-coding. Currently
  ///       we assume a simple control set.
  SimpleControlGenerator simple;
  SetControlSet(simple.GenerateDiscreteSet(this));
}


Robot::
~Robot() {
  delete m_multibody;
  delete m_agent;
  delete m_controlSet;
  delete m_controlSpace;

  for(auto a : m_actuators)
    delete a;
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

/*------------------------------ Agent Accessors -----------------------------*/

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

/*---------------------------- Control Accessors -----------------------------*/

ControlSet*
Robot::
GetControlSet() {
  return m_controlSet;
}


void
Robot::
SetControlSet(ControlSet* const _c) {
  delete m_controlSet;
  m_controlSet = _c;
}


ControlSpace*
Robot::
GetControlSpace() {
  return m_controlSpace;
}


void
Robot::
SetControlSpace(ControlSpace* const _c) {
  delete m_controlSpace;
  m_controlSpace = _c;
}

/*---------------------------- Actuator Accessors ----------------------------*/

Actuator*
Robot::
GetActuator(const size_t _i) {
  return m_actuators[_i];
}


const std::vector<Actuator*>&
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

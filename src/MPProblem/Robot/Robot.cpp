#include "Robot.h"

#include <algorithm>
#include <sstream>

#include "Actuator.h"
#include "DynamicsModel.h"
#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Utilities/XMLNode.h"

// Temporaries for hard-coded stuff
#include "ControlGenerators.h"
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

  // Get the robot file name and make sure it exists.
  const std::string path = GetPathName(_node.Filename());
  const std::string file = _node.Read("filename", true, "", "Robot file name");
  const std::string filename = path + file;
  if(!FileExists(filename))
    throw ParseException(filename, "File does not exist");

  // If we got an XML file, use that parsing mechanism.
  if(filename.find(".xml") != string::npos)
    ReadXMLFile(filename, _b);
  // Otherwise, we got a multibody file. Parse it and assume some settings for
  // the dynamics.
  else {
    ReadMultibodyFile(filename, _b);

    // Also assume some default settings for nonholonomic robots for now.

    // Set up actuators. Assume a single actuator that can generate thrust for
    // any DOF of a rigid body.
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

    // Create control set. Assume a simple discrete control set.
    SimpleControlGenerator simple;
    SetControlSet(simple.GenerateDiscreteSet(this));

    // Create controller. Assume the simplest controller.
    m_controller = new SimpleController(this, .1);
  }

  // Create dynamics model if the robot is nonholonomic.
  if(IsNonholonomic())
    m_dynamicsModel = new DynamicsModel(this, nullptr);
}


Robot::
Robot(MPProblem* const _p, ActiveMultiBody* _mb, const std::string& _label,
    const Boundary* const _b) :
    m_problem(_p), m_multibody(_mb), m_label(_label) {
  m_multibody->InitializeDOFs(_b);
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

/*---------------------------------- I/O -------------------------------------*/

void
Robot::
ReadXMLFile(const std::string& _filename, const Boundary* const _b) {
  XMLNode node(_filename, "Robot");

  // Read attributes of the robot node.
  m_maxLinearVelocity = node.Read("maxLinearVelocity", false, 10., 0.,
      std::numeric_limits<double>::max(), "The robot's maximum linear velocity");
  m_maxAngularVelocity = node.Read("maxAngularVelocity", false, 1., 0.,
      std::numeric_limits<double>::max(), "The robot's maximum angular velocity");

  bool readActuators = false;

  for(auto& child : node) {
    if(child.Name() == "Multibody") {
      // Read the multibody file. Eventually we'll go full XML and pass the
      // child node directly to the multibody instead.
      const std::string mbFile = child.Read("filename", true, "", "Name of the "
          "robot's multibody file");
      ReadMultibodyFile(GetPathName(_filename) + mbFile, _b);
    }
    else if(child.Name() == "Actuators") {
      // If we read an actuators section then we must have controls. We should
      // not already have any controls as there can only be one actuators
      // section.
      if(readActuators)
        throw ParseException(child.Where(), "Already have a control set - are "
            "there two Actuators nodes?");
      readActuators = true;

      for(auto& grandChild : child) {
        // Parse the actuator.
        auto actuator = new Actuator(this, grandChild);
        m_actuators.push_back(actuator);

        // Parse its controls.
        for(auto& greatGrandChild : grandChild) {
          // Read downcased type.
          std::string type = greatGrandChild.Read("type", true, "",
              "The type of control (currently only fixed supported)");
          std::transform(type.begin(), type.end(), type.begin(), ::tolower);

          // Setup the appropriate type of control.
          if(type == "fixed") {
            // Ensure we have a control set.
            if(!m_controlSet)
              SetControlSet(new ControlSet);

            Control c{actuator};

            // Read the control signal.
            const std::string signalString = greatGrandChild.Read("signal", true,
                "", "The control signal.");
            istringstream signal(signalString);
            double temp;
            while(signal >> temp)
              c.signal.push_back(temp);

            // Assert that we read the right number of signal values.
            if(c.signal.size() != m_multibody->DOF())
              throw ParseException(greatGrandChild.Where(), "Read control signal "
                  "with " + std::to_string(c.signal.size()) + " DOFs, but Robot "
                  "has " + std::to_string(m_multibody->DOF()) + " DOFs.");

            m_controlSet->push_back(c);
          }
        }
      }
    }
    else if(child.Name() == "Controller") {
      // Read downcased controller label.
      std::string controllerLabel = child.Read("label", true, "", "Label "
          "of the controller type to use");
      std::transform(controllerLabel.begin(), controllerLabel.end(),
          controllerLabel.begin(), ::tolower);

      // Setup the appropriate controller type.
      if(controllerLabel == "simple") {
        // Read the one parameter.
        const double coefficient = child.Read("coefficient", true, 0.,
            std::numeric_limits<double>::min(),
            std::numeric_limits<double>::max(), "Force coefficient");

        m_controller = new SimpleController(this, coefficient);
      }
      else
        throw ParseException(child.Where(), "Unknown controller label.");
    }
    else if(child.Name() == "Agent") {
      // The agent should be initialized by the Simulation object to avoid long
      // compile times for this object.
      m_agentLabel = child.Read("label", false, "", "Label of the agent type to "
          "use");
    }
  }

  // Throw errors for any unrequested attributes or nodes.
  node.WarnAll(true);
}


void
Robot::
ReadMultibodyFile(const std::string& _filename, const Boundary* const _b) {
  // Open the file.
  CountingStreamBuffer cbs(_filename);
  std::istream ifs(&cbs);

  // Parse the file to get the robot's geometry.
  m_multibody = new ActiveMultiBody;
  m_multibody->Read(ifs, cbs);

  // Initialize the DOF limits and set the robot to a zero starting configuration.
  m_multibody->InitializeDOFs(_b);
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));
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

/*---------------------------------- Debug -----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Robot& _r) {
  _os << "Robot '" << _r.GetLabel() << "'"
      << "\n\tDOF:      " << _r.GetMultiBody()->DOF()
      << "\n\tPosDOF:   " << _r.GetMultiBody()->PosDOF()
      << "\n\tOriDOF:   " << _r.GetMultiBody()->OrientationDOF()
      << "\n\tJointDOF: " << _r.GetMultiBody()->JointDOF()
      << "\nActuators:\n";
  for(const auto a : _r.m_actuators) {
    _os << *a
        << "\tControls";
    for(const auto& c : *_r.m_controlSet) {
      if(c.actuator != a)
        continue;
      _os << "\n\t\t";
      for(const auto v : c.signal)
        _os << std::setw(2) << v << " ";
    }
    _os << "\n";
  }
  return _os << std::endl;
}

/*----------------------------------------------------------------------------*/

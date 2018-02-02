#include "Robot.h"

#include <algorithm>
#include <sstream>

#include "Actuator.h"
#include "DynamicsModel.h"
#include "HardwareInterfaces/HardwareInterface.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Environment/Environment.h"
#include "Utilities/XMLNode.h"

// Temporaries for hard-coded stuff
#include "Behaviors/Controllers/ControlSetGenerators.h"
#include "Behaviors/Controllers/SimpleController.h"
#include "Behaviors/Controllers/ICreateController.h"
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

  // Check if this robot is flagged as virtual.
  m_virtual = _node.Read("virtual", false, false, "Virtual robots are imaginary "
      "and will not be included in the simulation or CD checks.");

  //std::string color = _node.Read("color", false, "1 0 0 1", "Color of the robot in simulation");
  //this->GetMultiBody()->GetBody(i)->SetBodyColor(color);

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
    m_actuators["default"] = std::unique_ptr<Actuator>(
        new Actuator(this, "default",
          IsNonholonomic() ? Actuator::Force : Actuator::Velocity));
    m_actuators["default"]->SetLimits(reverse, forward);
    m_actuators["default"]->SetMaxForce(1);

    // Use the simplest controller.
    m_controller = std::unique_ptr<SimpleController>(
        new SimpleController(this, 1, 1)
    );

    // If the robot is nonholonomic, we need a discrete control set for now to
    // get the kinodynamic extender working.
    if(IsNonholonomic())
      m_controller->SetControlSet(SimpleControlSetGenerator(this));
  }

  // Create dynamics model if the robot is nonholonomic.
  if(IsNonholonomic())
    SetDynamicsModel(nullptr);

  // Parse hardware and agent child nodes.
  for(auto& child : _node) {
    if(child.Name() == "HardwareInterface") {
      // Make sure we don't allow duplicate labels.
      const std::string label = child.Read("label", true, "",
          "A unique label for this hardware.");
      const bool duplicate = m_hardware.count(label);
      if(duplicate)
        throw RunTimeException(WHERE, "Hardware labels must be unique for a "
            "given robot.");

      SetHardwareInterface(label, HardwareInterface::Factory(child));
    }
    else if(child.Name() == "Agent") {
      SetAgent(Agent::Factory(this, child));
    }
  }

  // Initialize the emulated battery.
  m_battery = std::unique_ptr<Battery>(new Battery());
}


Robot::
Robot(MPProblem* const _p, std::unique_ptr<MultiBody>&& _mb,
    const std::string& _label)
  : m_problem(_p), m_label(_label), m_multibody(std::move(_mb)) {
  InitializePlanningSpaces();
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));

  m_battery = std::unique_ptr<Battery>(new Battery());
}


Robot::
Robot(MPProblem* const _p, const Robot& _r)
  : m_problem(_p),
    m_nonholonomic(_r.m_nonholonomic),
    m_carlike(_r.m_carlike),
    m_maxLinearVelocity(_r.m_maxLinearVelocity),
    m_maxAngularVelocity(_r.m_maxAngularVelocity)
{
  // Copy the robot label. If the robot was copied to the same problem, append
  // _copy to the end of the label to make sure it is unique.
  const bool sameProblem = _p == _r.GetMPProblem();
  m_label = _r.m_label + (sameProblem ? "_copy" : "");

  // Copy multibody.
  m_multibody = std::unique_ptr<MultiBody>(new MultiBody(*_r.m_multibody));

  // Copy actuators.
  for(const auto& labelPointer : _r.m_actuators) {
    auto label    = labelPointer.first;
    auto actuator = labelPointer.second.get();
    m_actuators[label] = std::unique_ptr<Actuator>(new Actuator(this, *actuator));
  }

  // Copy the planning spaces.
  m_cspace = std::unique_ptr<CSpaceBoundingBox>(
      new CSpaceBoundingBox(*_r.m_cspace)
  );
  if(_r.m_vspace)
    m_vspace = std::unique_ptr<CSpaceBoundingBox>(
        new CSpaceBoundingBox(*_r.m_vspace)
    );

  // Copy controller.
  if(_r.m_controller)
    SetController(_r.m_controller->Clone(this));

  // We can't copy the bullet model directly - it must be recreated by adding
  // this robot to a simulation because some of the related data is stored in
  // the Simulation object. This will initialize the internal micro-sim only.
  if(_r.m_dynamicsModel)
    SetDynamicsModel(nullptr);

  // TODO
  // We will not copy the agent because each one must be created for a specific
  // robot object. We will only copy the agent label and require the Simulation
  // to handle the rest.
  if(_r.m_agent)
    std::cerr << "WARNING: copying a robot does not copy the agent object "
              << "because each must be created for a terface object driving a "
              << "given piece of hardware at a time."
              << std::endl;

  // We will not copy the hardware interfaces because there should only be one
  // such object for a given piece of hardware. Warn the user in this case.
  if(_r.m_hardware.size())
    std::cerr << "WARNING: copying a robot does not copy the hardware interface "
              << "because there should only be one interface object driving a "
              << "given piece of hardware at a time."
              << std::endl;

  if(_r.m_battery.get())
    m_battery = std::unique_ptr<Battery>(new Battery(*_r.m_battery));
}


Robot::
~Robot() = default;

Robot::
Robot(Robot&&) = default;

Robot& Robot::operator=(Robot&&) = default;

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
    if(child.Name() == "MultiBody") {
      // Read the multibody file. Eventually we'll go full XML and pass the
      // child node directly to the multibody instead.
      const std::string mbFile = child.Read("filename", false, "", "Name of the "
          "robot's multibody file");

      // If there is no filename then the multibody information is in the XML
      // child node.
      if(mbFile == "")
        ReadMultiBodyXML(child);
      else
        ReadMultibodyFile(GetPathName(_filename) + mbFile);
    }
    else if(child.Name() == "Actuator") {
      // Parse the actuator.
      std::unique_ptr<Actuator> actuator(new Actuator(this, child));
      m_actuators[actuator->GetLabel()] = std::move(actuator);
    }
    else if(child.Name() == "Controller") {
      auto controller = ControllerMethod::Factory(this, child);
      SetController(std::move(controller));
    }
  }

  // Throw errors for any unrequested attributes or nodes.
  node.WarnAll(true);
}


void
Robot::
ReadMultiBodyXML(XMLNode& _node) {
  m_multibody = std::unique_ptr<MultiBody>(new MultiBody(_node));

  // Initialize the DOF limits and set the robot to a zero starting configuration.
  InitializePlanningSpaces();
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));
}


void
Robot::
ReadMultibodyFile(const std::string& _filename) {
  // Open the file.
  CountingStreamBuffer cbs(_filename);
  std::istream ifs(&cbs);

  // Parse the file to get the robot's geometry.
  m_multibody = std::unique_ptr<MultiBody>(new MultiBody(MultiBody::Type::Active));
  m_multibody->Read(ifs, cbs);

  // Initialize the DOF limits and set the robot to a zero starting configuration.
  InitializePlanningSpaces();
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));
}

/*--------------------------- Planning Interface -----------------------------*/

void
Robot::
InitializePlanningSpaces() {
  m_multibody->InitializeDOFs(m_problem->GetEnvironment()->GetBoundary());

  const size_t dof = m_multibody->DOF();
  const auto& dofInfo = m_multibody->GetDofInfo();

  // Create the configuration space boundary.
  m_cspace = std::unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(dof));

  for(size_t i = 0; i < dof; ++i)
    m_cspace->SetRange(i, dofInfo[i].range);

  // If this is a holonomic robot, we are done.
  if(!m_nonholonomic)
    return;

  // Create the velocity space boundary.
  m_vspace = std::unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(dof));

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


const CSpaceBoundingBox*
Robot::
GetCSpace() const noexcept {
  return m_cspace.get();
}


const CSpaceBoundingBox*
Robot::
GetVSpace() const noexcept {
  return m_vspace.get();
}


MPProblem*
Robot::
GetMPProblem() const noexcept {
  return m_problem;
}

/*------------------------- Simulation Interface -----------------------------*/

void
Robot::
Step(const double _dt) {
  // Run the agent's decision-making routine. The agent will apply controls as
  // required to execute its decision.
  if(m_agent)
    m_agent->Step(_dt);
}


void
Robot::
SynchronizeModels() noexcept {
  auto dynamics = GetDynamicsModel();
  if(dynamics)
    GetMultiBody()->Configure(dynamics->GetSimulatedState());
}

/*--------------------------- Geometry Accessors -----------------------------*/

MultiBody*
Robot::
GetMultiBody() noexcept {
  return m_multibody.get();
}


const MultiBody*
Robot::
GetMultiBody() const noexcept {
  return m_multibody.get();
}

/*------------------------------ Agent Accessors -----------------------------*/

Agent*
Robot::
GetAgent() noexcept {
  return m_agent.get();
}


void
Robot::
SetAgent(std::unique_ptr<Agent>&& _a) noexcept {
  m_agent = std::move(_a);
}

/*---------------------------- Control Accessors -----------------------------*/

ControllerMethod*
Robot::
GetController() noexcept {
  return m_controller.get();
}


void
Robot::
SetController(std::unique_ptr<ControllerMethod>&& _c) noexcept {
  m_controller = std::move(_c);
}

/*---------------------------- Actuator Accessors ----------------------------*/

Actuator*
Robot::
GetActuator(const std::string& _label) noexcept {
  return m_actuators[_label].get();
}


const std::unordered_map<std::string, std::unique_ptr<Actuator>>&
Robot::
GetActuators() const noexcept {
  return m_actuators;
}

/*---------------------------- Dynamics Accessors ----------------------------*/

DynamicsModel*
Robot::
GetDynamicsModel() noexcept {
  return m_dynamicsModel.get();
}


void
Robot::
SetDynamicsModel(btMultiBody* const _m) {
  if(m_dynamicsModel and _m == m_dynamicsModel->Get())
    return;
  m_dynamicsModel = std::unique_ptr<DynamicsModel>(new DynamicsModel(this, _m));
}

/*---------------------------- Hardware Interface ----------------------------*/

HardwareInterface*
Robot::
GetHardwareInterface(const std::string& _label) const noexcept {
  if(m_hardware.count(_label))
    return m_hardware.at(_label).get();
  return nullptr;
}


void
Robot::
SetHardwareInterface(const std::string& _label,
    std::unique_ptr<HardwareInterface>&& _i) noexcept {
  m_hardware[_label] = std::move(_i);
}


Battery*
Robot::
GetBattery() const noexcept {
  return m_battery.get();
}

/*------------------------------- Other --------------------------------------*/

bool
Robot::
IsVirtual() const noexcept {
  return m_virtual;
}


void
Robot::
SetVirtual(bool _v) noexcept {
  m_virtual = _v;
}


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
  for(const auto& a : _r.GetActuators())
    _os << *a.second.get() << "\n";
  return _os << std::endl;
}

/*----------------------------------------------------------------------------*/

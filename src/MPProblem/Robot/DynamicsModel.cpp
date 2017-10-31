#include "DynamicsModel.h"

#include "Control.h"
#include "Robot.h"
#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "Utilities/PMPLExceptions.h"

#include "Simulator/BulletEngine.h"
#include "Simulator/Conversions.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Local Helpers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/// Extract the configuration from a simulated robot.
/// @param _robot A PMPL robot.
/// @param _model A bullet model of _robot.
/// @return The configuration data of _model in its simulation.
Cfg ExtractSimulatedState(Robot* const _robot, const btMultiBody* const _model);

/// Configure a simulated robot.
/// @param _c The configuration to set.
/// @param _model A bullet model of _c's robot.
void ConfigureSimulatedState(const Cfg& _c, btMultiBody* const _model);

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Micro Simulator ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------ Construction --------------------------------*/

MicroSimulator::
MicroSimulator(Robot* const _robot) :
    m_robot(_robot),
    m_engine(new BulletEngine(m_robot->GetMPProblem())),
    m_model(m_engine->AddObject(m_robot->GetMultiBody())) {
  if(m_robot->IsCarlike())
    m_engine->CreateCarlikeCallback(m_model);
}


MicroSimulator::
~MicroSimulator() {
  delete m_engine;
}

/*-------------------------------- Interface ---------------------------------*/

Cfg
MicroSimulator::
Test(const Cfg& _start, const Control& _control, const double _dt) {
  // Set up the simulated robot at _start.
  if(_start.GetRobot() != m_robot)
    throw RunTimeException(WHERE, "Can't test dynamics model on a configuration "
        "for a different robot.");
  ConfigureSimulatedState(_start, m_model);

#if 0
  // Verify our conversions. Use this test to assert that we get back the same
  // Cfg that we put in to bullet.

  const Cfg test = ExtractSimulatedState(m_robot, m_model);
  if(test != _start) {
    ostringstream oss;
    oss << "Cfg changed after configure/extract:"
        << "\n\t_start: " << _start
        << "\n\ttest:   " << test
        << "\n\tdelta:  " << _start - test;
    throw RunTimeException(WHERE, oss.str());
  }
#endif

  // Set the force on the internal model.
  _control.Execute(m_model);

  // Apply.
  m_engine->Step(_dt);

  // Return the resulting state.
  return ExtractSimulatedState(m_robot, m_model);
}


Cfg
MicroSimulator::
Test(const Cfg& _start, const ControlSet& _controlSet, const double _dt) {
  // Set up the simulated robot at _start.
  if(_start.GetRobot() != m_robot)
    throw RunTimeException(WHERE, "Can't test dynamics model on a configuration "
        "for a different robot.");
  ConfigureSimulatedState(_start, m_model);

  // Set the force on the internal model.
  for(const auto& c : _controlSet)
    c.Execute(m_model);

  // Apply.
  m_engine->Step(_dt);

  // Return the resulting state.
  return ExtractSimulatedState(m_robot, m_model);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Dynamics Model ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------ Construction --------------------------------*/

DynamicsModel::
DynamicsModel(Robot* const _r, btMultiBody* const _b) : m_robot(_r), m_model(_b)
{ }


DynamicsModel::
~DynamicsModel() noexcept {
  delete m_simulator;
}

/*------------------------------ Conversion ----------------------------------*/

btMultiBody*
DynamicsModel::
Get() const noexcept {
  return m_model;
}


DynamicsModel::
operator btMultiBody*() const noexcept {
  return m_model;
}

/*------------------------------ Interface -----------------------------------*/

Cfg
DynamicsModel::
GetSimulatedState() const {
  return ExtractSimulatedState(m_robot, m_model);
}

void
DynamicsModel::
SetSimulatedState(const Cfg& _cfg) {
  ConfigureSimulatedState(_cfg, m_model);
}


std::vector<double>
DynamicsModel::
GetSimulatedForces() const {
  const auto& f = m_model->getBaseForce();
  const auto& t = m_model->getBaseTorque();

  std::vector<double> out{f[0], f[1], f[2], t[0], t[1], t[2]};

  for(size_t i = 0; i < m_robot->GetMultiBody()->JointDOF(); ++i)
    out.push_back(m_model->getJointTorque(i));

  return out;
}


Cfg
DynamicsModel::
Test(const Cfg& _start, const Control& _c, const double _dt) const {
  // Initialze the internal simulator if it isn't already available.
  if(!m_simulator)
    m_simulator = new MicroSimulator(m_robot);
  return m_simulator->Test(_start, _c, _dt);
}


void
DynamicsModel::
LocalToWorld(std::vector<double>& _force, btMultiBody* const _model) const {
  Transform(_force,
      [_model](btVector3& _v) {_v = _model->localDirToWorld(-1, _v);});
}


void
DynamicsModel::
LocalToWorld(std::vector<double>& _force) const {
  LocalToWorld(_force, m_model);
}


void
DynamicsModel::
WorldToLocal(std::vector<double>& _force, btMultiBody* const _model) const {
  Transform(_force,
      [_model](btVector3& _v) {_v = _model->worldDirToLocal(-1, _v);});
}


void
DynamicsModel::
WorldToLocal(std::vector<double>& _force) const {
  WorldToLocal(_force, m_model);
}

/*-------------------------------- Helpers -----------------------------------*/

void
DynamicsModel::
Transform(std::vector<double>& _force, std::function<void(btVector3&)>&& _f)
    const {
  auto read = _force.begin(),
       write = _force.begin();

  auto mb = m_robot->GetMultiBody();
  btVector3 scratch(0, 0, 0);

  // Convert base force.
  const size_t numPos = mb->PosDOF();
  for(size_t i = 0; i < numPos; ++i, ++read)
    scratch[i] = *read;

  _f(scratch);

  for(size_t i = 0; i < numPos; ++i, ++write)
    *write = scratch[i];


  // Convert base torque.
  scratch.setValue(0, 0, 0);

  switch(mb->OrientationDOF()) {
    case 3:
      // This is a volumetric rotational robot. We need all three torque
      // directions.
      for(int i = 0; i < 2; ++i, ++read)
        scratch[i] = *read;

      _f(scratch);

      for(int i = 0; i < 2; ++i, ++write)
        *write = scratch[i];

      break;
    case 1:
      // This is a planar rotational robot. We only want a torque in the Z
      // direction.
      /// @TODO In simulation, it is possible that a planar rotational robot
      /// will collide with something and be knocked off its plane. If this
      /// happens, the transformations between our 3-dof Cfgs and the bullet
      /// world will be thrown off with no way to recover. The only resolution is
      /// to treat these as 6-dof robots in simulation.
      scratch[2] = *read;
      _f(scratch);
      *write = scratch[2];
      break;
    default:
      // This robot does not rotate.
      ;
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Internal Helpers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

Cfg
ExtractSimulatedState(Robot* const _robot, const btMultiBody* const _model) {
  auto mb = _robot->GetMultiBody();
  Cfg out(_robot);

  // First get the base state.
  switch(mb->GetBaseType()) {
    case FreeBody::BodyType::Fixed:
      break;
    case FreeBody::BodyType::Planar:
      {
        // Get the base transform.
        auto transform = ToPMPL(_model->getBaseWorldTransform());
        auto cfg = transform.GetCfg();

        switch(mb->GetBaseMovementType()) {
          case FreeBody::MovementType::Rotational:
            out[2] = cfg[5];
          case FreeBody::MovementType::Translational:
            out[0] = cfg[0];
            out[1] = cfg[1];
            break;
          default:
            throw RunTimeException(WHERE, "Unrecognized base movement type.");
        }
      }
      break;
    case FreeBody::BodyType::Volumetric:
      {
        // Get the base transform.
        auto transform = ToPMPL(_model->getBaseWorldTransform());
        auto cfg = transform.GetCfg();

        switch(mb->GetBaseMovementType()) {
          case FreeBody::MovementType::Translational:
            out[0] = cfg[0];
            out[1] = cfg[1];
            out[2] = cfg[2];
            break;
          case FreeBody::MovementType::Rotational:
            for(size_t i = 0; i < 6; ++i)
              out[i] = cfg[i];
            break;
          default:
            throw RunTimeException(WHERE, "Unrecognized base movement type.");
        }
      }
      break;
    default:
      throw RunTimeException(WHERE, "Unrecognized base type.");
  }
  out.NormalizeOrientation();

  // Get base velocities.
  const bool getVelocity = _robot->IsNonholonomic();
  if(getVelocity)
  {
    out.SetLinearVelocity(ToPMPL(_model->getBaseVel()));
    out.SetAngularVelocity(ToPMPL(_model->getBaseOmega()));
  }

  // Get joint positions.
  {
    const size_t firstJointIndex = mb->DOF() - mb->JointDOF();
    const size_t lastJointIndex = mb->DOF();

    int count = 0;
    for(size_t i = firstJointIndex; i < lastJointIndex; ++i, ++count) {
      // Bullet uses [ -PI : PI ].
      out[i] = _model->getJointPos(count) / PI;
      if(getVelocity)
        out.Velocity(i) = _model->getJointVel(count) / PI;
    }
  }

  return out;
}


void
ConfigureSimulatedState(const Cfg& _c, btMultiBody* const _model) {
  auto mb = _c.GetMultiBody();
  const bool getVelocity = _c.GetRobot()->IsNonholonomic();

  // Set the base DOFs.
  /// @TODO Make a more efficient routine for this, that doesn't need to
  ///       configure the PMPL robot. Ideally we should grab the base's world
  ///       transform directly from the _c cfg.
  _c.ConfigureRobot();
  _model->setBaseWorldTransform(ToBullet(mb->GetFreeBody(0)->
        GetWorldTransformation()));
  if(getVelocity) {
    _model->setBaseVel(ToBullet(_c.GetLinearVelocity()));
    _model->setBaseOmega(ToBullet(_c.GetAngularVelocity()));
  }

  // Set the joint DOFs.
  const size_t firstJointIndex = mb->DOF() - mb->JointDOF();
  const size_t lastJointIndex = mb->DOF();
  for(size_t i = firstJointIndex, index = 0; i < lastJointIndex; ++i, ++index) {
    // Bullet uses [ -PI : PI ].
    _model->setJointPos(index, _c[i] * PI);
    if(getVelocity)
      _model->setJointVel(index, _c.Velocity(i) * PI);
  }
}

/*----------------------------------------------------------------------------*/

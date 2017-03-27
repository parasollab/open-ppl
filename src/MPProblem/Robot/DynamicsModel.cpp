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


/*------------------------------ Local Helpers -------------------------------*/

/// Extract the configuration from a simulated robot.
/// @param _robot A PMPL robot.
/// @param _model A bullet model of _robot.
/// @return The configuration data of _model in its simulation.
Cfg ExtractSimulatedState(Robot* const _robot, btMultiBody* const _model);

/// Configure a simulated robot.
/// @param _c The configuration to set.
/// @param _model A bullet model of _c's robot.
void ConfigureSimulatedState(const Cfg& _c, btMultiBody* const _model);

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~ Internal Simulator ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

////////////////////////////////////////////////////////////////////////////////
/// A micro simulator for a single robot.
///
/// @details This object outsources forward dynamics computations to the bullet
///          physics engine.
////////////////////////////////////////////////////////////////////////////////
class InternalSimulator final {

  ///@name Internal State
  ///@{

  Robot* const m_robot;   ///< Our pmpl robot.
  BulletEngine m_engine;  ///< The engine for this micro-simulator.
  btMultiBody* m_model;   ///< The bullet body for our robot.

  const btScalar m_timestep; ///< The smallest timestep to use.

  ///@}

  public:

    ///@name Construction
    ///@{

    InternalSimulator(Robot* const _robot);

    ///@}
    ///@name Interface
    ///@{

    Cfg Test(const Cfg& _start, const Control& _c, const double _dt);

    ///@}

};

/*------------------------------ Construction --------------------------------*/

InternalSimulator::
InternalSimulator(Robot* const _robot) :
    m_robot(_robot), m_engine(m_robot->GetMPProblem()),
    m_timestep(m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes()) {
  //m_engine = new BulletEngine(m_robot->GetMPProblem());
  m_model = m_engine.AddObject(m_robot->GetMultiBody());

  // Add gravity in the model.
  // Note that using the dynamicsWorld->setGravity, it loops through and
  // updates all bodies in the world, and should update future bodies added.
  //m_engine.SetGravity(
  //    ToBullet(m_robot->GetMPProblem()->GetEnvironment()->GetGravity()));
  // Can get InternalSimulator gravity using m_engine.GetGravity()
}

/*-------------------------------- Interface ---------------------------------*/

Cfg
InternalSimulator::
Test(const Cfg& _start, const Control& _c, const double _dt) {
  // Set up the simulated robot at _start.
  if(_start.GetRobot() != m_robot)
    throw RunTimeException(WHERE, "Can't test dynamics model on a configuration "
        "for a different robot.");
  ConfigureSimulatedState(_start, m_model);

  // Set the force on the internal model.
  _c.Execute(m_model);

  // Apply.
  // Advance by _dt ...
  // Using tics m_timestep long...
  const int maxSubSteps = std::ceil(_dt / m_timestep); // Up to this many ticks.
  m_engine.Step(_dt, maxSubSteps, m_timestep);

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


Cfg
DynamicsModel::
Test(const Cfg& _start, const Control& _c, const double _dt) const {
  // Initialze the internal simulator if it isn't already available.
  if(!m_simulator)
    m_simulator = new InternalSimulator(m_robot);
  return m_simulator->Test(_start, _c, _dt);
}

/*--------------------------------- Helpers ----------------------------------*/

Cfg
ExtractSimulatedState(Robot* const _robot, btMultiBody* const _model) {
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
          case FreeBody::MovementType::Translational:
            out[0] = cfg[0];
            out[1] = cfg[1];
            break;
          case FreeBody::MovementType::Rotational:
            out[0] = cfg[0];
            out[1] = cfg[1];
            out[2] = cfg[3];
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
    const btVector3& linear = _model->getBaseVel();
    for(size_t i = 0; i < mb->PosDOF(); ++i)
      out.Velocity(i) = linear[i];

    const btVector3 angular = _model->getBaseOmega();
    const size_t firstJointIndex = mb->DOF() - mb->JointDOF();
    int count = -1;
    for(size_t i = mb->PosDOF(); i < firstJointIndex; ++i)
      out.Velocity(i) = angular[++count];
  }

  // Get joint positions.
  {
    const size_t firstJointIndex = mb->DOF() - mb->JointDOF();
    const size_t lastJointIndex = mb->DOF();

    int count = -1;
    for(size_t i = firstJointIndex; i < lastJointIndex; ++i) {
      // Bullet uses [ -PI : PI ].
      out[i] = _model->getJointPos(++count) / PI;
      if(getVelocity)
        out.Velocity(i) = _model->getJointVel(++count) / PI;
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
    _model->setBaseVel(ToBullet(_c.LinearVelocity()));
    _model->setBaseOmega(ToBullet(_c.AngularVelocity()));
  }

  // Set the joint DOFs.
  int index = 0;
  const size_t firstJointIndex = mb->DOF() - mb->JointDOF();
  const size_t lastJointIndex = mb->DOF();
  for(size_t i = firstJointIndex; i < lastJointIndex; ++i, ++index) {
    // Bullet uses [ -PI : PI ].
    _model->setJointPos(index, _c[i] * PI);
    if(getVelocity)
      _model->setJointVel(index, _c.Velocity(i) * PI);
  }
}

/*----------------------------------------------------------------------------*/

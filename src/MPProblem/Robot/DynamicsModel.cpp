#include "DynamicsModel.h"

#include "Robot.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Simulator/Conversions.h"
#include "Utilities/PMPLExceptions.h"

#include "BulletDynamics/Featherstone/btMultiBody.h"

/*------------------------------ Construction --------------------------------*/

DynamicsModel::
DynamicsModel(Robot* const _r, btMultiBody* const _b) : m_robot(_r), m_model(_b)
{ }

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

std::vector<double>
DynamicsModel::
GetSimulatedState() const {
  std::vector<double> out;
  out.reserve(m_robot->GetMultiBody()->DOF());

  // First get the base state.
  switch(m_robot->GetMultiBody()->GetBaseType()) {
    case FreeBody::BodyType::Fixed:
      break;
    case FreeBody::BodyType::Planar:
      {
        // Get the base transform.
        auto transform = ToPMPL(m_model->getBaseWorldTransform());
        auto cfg = transform.GetCfg();

        switch(m_robot->GetMultiBody()->GetBaseMovementType()) {
          case FreeBody::MovementType::Translational:
            out = {cfg[0], cfg[1]};
            break;
          case FreeBody::MovementType::Rotational:
            out = {cfg[0], cfg[1], cfg[5]};
            break;
          default:
            throw RunTimeException(WHERE, "Unrecognized base movement type.");
        }
        break;
      }
    case FreeBody::BodyType::Volumetric:
      {
        // Get the base transform.
        auto transform = ToPMPL(m_model->getBaseWorldTransform());
        auto cfg = transform.GetCfg();

        switch(m_robot->GetMultiBody()->GetBaseMovementType()) {
          case FreeBody::MovementType::Translational:
            out = {cfg[0], cfg[1], cfg[2]};
            break;
          case FreeBody::MovementType::Rotational:
            out = std::move(cfg);
            break;
          default:
            throw RunTimeException(WHERE, "Unrecognized base movement type.");
        }
        break;
      }
    default:
      throw RunTimeException(WHERE, "Unrecognized base type.");
  }

  while(out.size() < m_robot->GetMultiBody()->DOF())
    out.push_back(0);

  return out;
}

/*----------------------------------------------------------------------------*/

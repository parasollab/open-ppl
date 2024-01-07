#include "DynamicDomainRRT.h"

#include "MPLibrary/MPLibrary.h"

/*---------------------------- Construction ----------------------------------*/

DynamicDomainRRT::DynamicDomainRRT() : BasicRRTStrategy() {
  this->SetName("DynamicDomainRRT");
}

DynamicDomainRRT::DynamicDomainRRT(XMLNode& _node) : BasicRRTStrategy(_node) {
  this->SetName("DynamicDomainRRT");

  m_r = _node.Read(
      "r", true, m_r, 0., std::numeric_limits<double>::max(),
      "Dynamic domain factor. This is a multiple of extender max distance.");

  m_dmLabel =
      _node.Read("dmLabel", true, "",
                 "Distance metric for measuring dynamic domain inclusion.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void DynamicDomainRRT::Print(std::ostream& _os) const {
  BasicRRTStrategy::Print(_os);
  _os << "\tDynamic Domain radius: " << m_r
      << "\n\tDynamic Domain distance metric: " << m_dmLabel << std::endl;
}

/*-------------------------- MPStrategy Overrides ----------------------------*/

void DynamicDomainRRT::Initialize() {
  BasicRRTStrategy::Initialize();

  // Label any input vertices with unrestricted domain.
  for (auto v : *this->GetRoadmap())
    v.property().SetStat(RLabel(), std::numeric_limits<double>::max());
}

/*----------------------------- RRT Overrides --------------------------------*/

typename DynamicDomainRRT::VID DynamicDomainRRT::Extend(
    const VID _nearVID,
    const Cfg& _qRand,
    LPOutput& _lp,
    const bool _requireNew) {
  const VID newVID =
      BasicRRTStrategy::Extend(_nearVID, _qRand, _lp, _requireNew);

  if (newVID != INVALID_VID) {
    Cfg& cfg = this->GetRoadmap()->GetVertex(newVID);
    cfg.SetStat(RLabel(), std::numeric_limits<double>::max());
  } else {
    auto e = this->GetMPLibrary()->GetExtender(this->m_exLabel);
    Cfg& cfg = this->GetRoadmap()->GetVertex(_nearVID);
    cfg.SetStat(RLabel(), m_r * e->GetMaxDistance());
  }
  return newVID;
}

typename DynamicDomainRRT::VID DynamicDomainRRT::ExpandTree(
    const VID _nearestVID,
    const Cfg& _target) {
  const Cfg& cfg = this->GetRoadmap()->GetVertex(_nearestVID);

  // If the nearest node's radius is too small, return failure.
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  if (dm->Distance(cfg, _target) >= cfg.GetStat(RLabel()))
    return INVALID_VID;
  else
    return BasicRRTStrategy::ExpandTree(_nearestVID, _target);
}

/*----------------------------------------------------------------------------*/

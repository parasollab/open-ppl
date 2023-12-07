#include "ExtenderMethod.h"

#include <limits>

/*------------------------------ Construction --------------------------------*/

ExtenderMethod::
ExtenderMethod(XMLNode& _node) : MPBaseObject(_node) {
  // We do not require these to be specified because some extenders don't use
  // them (like KinodynamicExtender).
  m_maxDist = _node.Read("maxDist", false, m_maxDist,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "The maximum allowed distance to expand from the starting node to the "
      "target node.");

  m_minDist = _node.Read("minDist", false, m_minDist,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "The minimum valid distance when expanding from the starting node to the "
      "target node (shorter extensions are considered invalid).");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

void
ExtenderMethod::
Print(std::ostream& _os) const {
  MPBaseObject::Print(_os);
  _os << "\tMin distance: " << m_minDist
      << "\n\tMax distance: " << m_maxDist
      << std::endl;
}

/*------------------------- ExtenderMethod Interface -------------------------*/

double
ExtenderMethod::
GetMinDistance() const {
  return m_minDist;
}


double
ExtenderMethod::
GetMaxDistance() const {
  return m_maxDist;
}


bool
ExtenderMethod::
Extend(const Cfg& _start, const Cfg& _end, Cfg& _new,
    LPOutput& _lp, CDInfo& _cdInfo) {
  throw NotImplementedException(WHERE);
}


bool
ExtenderMethod::
Extend(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupCfgType& _new, GroupLPOutput& _lp,
    const std::vector<size_t>& _robotIndexes) {
  throw NotImplementedException(WHERE);
}


bool
ExtenderMethod::
Extend(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupCfgType& _new, GroupLPOutput& _lp, CDInfo& _cdInfo,
    const std::vector<size_t>& _robotIndexes) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

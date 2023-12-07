#include "NeighborhoodFinderMethod.h"

#include <boost/mpl/list.hpp>
#include <boost/mpl/for_each.hpp>


/*------------------------------ Construction --------------------------------*/

NeighborhoodFinderMethod::
NeighborhoodFinderMethod(const Type _type) : MPBaseObject(),
    m_nfType(_type) {
}


NeighborhoodFinderMethod::
NeighborhoodFinderMethod(XMLNode& _node, const Type _type,
    const bool _requireDM) : MPBaseObject(_node) {
  if(_requireDM)
    m_dmLabel = _node.Read("dmLabel", true, "", "Distance Metric Method");

  m_unconnected = _node.Read("unconnected", false, m_unconnected,
      "Require neighbors to be non-adjacent to the query configuration");

  m_nfType = _type;
  switch(_type) {
    case Type::K:
    {
      m_k = _node.Read("k", true,
          m_k, size_t(0), std::numeric_limits<size_t>::max(),
          "The number of neighbors to find. Zero for all.");
      break;
    }
    case Type::RADIUS:
    {
      m_radius = _node.Read("radius", true,
          m_radius, 0., std::numeric_limits<double>::max(),
          "Include all neighbors within this metric radius.");
      break;
    }
    default:
      break;
  }
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
NeighborhoodFinderMethod::
Print(std::ostream& _os) const {
  MPBaseObject::Print(_os);
  _os << "\tdmLabel: " << m_dmLabel
      << "\n\tunconnected: " << m_unconnected
      << std::endl;
}

/*-------------------------------- Accessors ---------------------------------*/

inline
typename NeighborhoodFinderMethod::Type
NeighborhoodFinderMethod::
GetType() const noexcept {
  return m_nfType;
}


inline
size_t&
NeighborhoodFinderMethod::
GetK() noexcept {
  return m_k;
}


inline
double&
NeighborhoodFinderMethod::
GetRadius() noexcept {
  return m_radius;
}


inline
void
NeighborhoodFinderMethod::
SetDMLabel(const std::string& _label) noexcept {
  m_dmLabel = _label;
}


inline
const std::string&
NeighborhoodFinderMethod::
GetDMLabel() const noexcept {
  return m_dmLabel;
}

/*----------------------------------------------------------------------------*/

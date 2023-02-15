/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_LOC_QUAL_HPP
#define STAPL_UTILITY_LOC_QUAL_HPP

#include <ostream>

#include <stapl/runtime.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Qualifies the locality information a view or container returns
/// about an element.
///
/// LQ_CERTAIN indicates that the element is stored on the specified location.
/// LQ_LOOKUP indicates that the current location wasn't able to determine the
/// element's locality, and that the locality query should be reexecuted on the
/// specified location.
/// LQ_DONTCARE indicates that the storage of the location is irrelevant.
//////////////////////////////////////////////////////////////////////
enum loc_qual
{
  LQ_CERTAIN,
  LQ_LOOKUP,
  LQ_DONTCARE
};


//////////////////////////////////////////////////////////////////////
/// @brief Output operator overload for the location qualifier enum
///        @ref loc_qual.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
inline std::ostream& operator<<(std::ostream& s, const loc_qual q)
{
  switch (q)
  {
    case LQ_CERTAIN:
      s << " certain ";
      break;
    case LQ_LOOKUP:
      s << " lookup ";
      break;
    case LQ_DONTCARE:
      s << " don't care ";
      break;
    default:
      s << " unknown ";
      break;
  }
  return s;
}


//////////////////////////////////////////////////////////////////////
/// @brief Locality information class used by the containers and views
///   to coarsen data and provide information to tasks placement policies
///   of a given PARAGRAPH's scheduler.
/// @todo handle() methods need to be updated in light of const qualification
///   fixes in rmi_handle class hierarchy.
/// @todo Default constructor should be unnecessary.  Track down usage in
///   coarsening and resolve any anti-patterns requiring it.
//////////////////////////////////////////////////////////////////////
class locality_info
{
private:
  loc_qual                m_qualifier;
  affinity_tag            m_affinity;
  rmi_handle::reference   m_handle;
  location_type           m_location;

public:
  constexpr locality_info()
    : m_qualifier(LQ_DONTCARE),
      m_affinity(invalid_affinity_tag),
      m_handle(rmi_handle::reference()),
      m_location(invalid_location_id)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Shorthand constructor for use when desiring to report
  ///   LQ_DONTCARE (placement policy ignores all other fields).
  /// @todo reenable assert when C++14 (n3597) support available.
  //////////////////////////////////////////////////////////////////////
  constexpr locality_info(loc_qual qualifier)
    : m_qualifier(qualifier),
      m_affinity(invalid_affinity_tag),
      m_location(invalid_location_id)
  {
#if 0
    stapl_assert(qualifier == LQ_DONTCARE,
      "qualifier only, locality_info ctor requires LQ_DONTCARE");
#endif
  }

  locality_info(loc_qual qualifier, affinity_tag affinity,
                rmi_handle::reference const& handle, location_type location)
     : m_qualifier(qualifier), m_affinity(affinity),
       m_handle(handle), m_location(location)
  { }

  loc_qual qualifier(void) const
  {
    return m_qualifier;
  }

  affinity_tag affinity(void) const
  {
    return m_affinity;
  }

  location_type location(void) const
  {
    return m_location;
  }

  rmi_handle::reference& handle(void)
  {
    return m_handle;
  }

  rmi_handle::reference handle(void) const
  {
    return m_handle;
  }

  void define_type(typer &t)
  {
    t.member(m_qualifier);
    t.member(m_affinity);
    t.member(m_handle);
    t.member(m_location);
  }
}; // class locality_info

} // namespace stapl

#endif

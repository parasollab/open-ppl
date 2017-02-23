#ifndef EXTENDER_METHOD_H_
#define EXTENDER_METHOD_H_

#include <limits>

#include "MPLibrary/LocalPlanners/LPOutput.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Base algorithm abstraction for \ref Extenders.
///
/// ExtenderMethod has one main method, @c Extend, to grow a simple path from a
/// starting node in some input direction - note that not all expansion
/// methods go in straight lines through @cspace.
///
/// @usage
/// @code
/// ExtenderPointer e = this->GetExtender(m_exLabel);
/// CfgType start, goal, new;
/// LPOutput<MPTraits> lp;
/// bool pass = e->Extend(start, goal, new, lp);
/// @endcode
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class ExtenderMethod : public MPBaseObject<MPTraits> {

  protected:

    ///@name Extender Properties
    ///@{

    double m_minDist;  ///< The minimum valid extension distance.
    double m_maxDist;  ///< The maximum valid extension distance.

    ///@}

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    ExtenderMethod(const double _min = .001, const double _max = 1);

    ExtenderMethod(XMLNode& _node);

    virtual ~ExtenderMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Required Interface
    ///@{

    /// Get the minimum extension distance.
    virtual double GetMinDistance() const;

    /// Get the maximum extension distance.
    virtual double GetMaxDistance() const;

    /// Extends a local plan from a starting configuration towards a target
    /// configuration.
    /// @param _start Initial configuration to grow from.
    /// @param _end   Target configuration to grow towards.
    /// @param _new   Placeholder for resulting configuration.
    /// @param _lp    Placeholder for polygonal chain configurations for
    ///               non-straight-line extention operations and associated
    ///               weight.
    /// @return True if the extension produced a valid configuration that was
    ///         at least the minimum distance away from the starting point.
    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) = 0;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ExtenderMethod<MPTraits>::
ExtenderMethod(const double _min, const double _max) :
    m_minDist(_min), m_maxDist(_max) { }


template <typename MPTraits>
ExtenderMethod<MPTraits>::
ExtenderMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  m_maxDist = _node.Read("maxDist", false, 1., 0.,
      std::numeric_limits<double>::max(), "The maximum allowed distance to "
      "expand from the starting node to the target node.");
  m_minDist = _node.Read("minDist", false, 0., 0.,
      std::numeric_limits<double>::max(), "The minimum valid distance when "
      "expanding from the starting node to the target node (shorter extensions "
      "are considered invalid)");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
ExtenderMethod<MPTraits>::
Print(ostream& _os) const {
  MPBaseObject<MPTraits>::Print(_os);
  _os << "\tMin distance: " << m_minDist << endl
      << "\tMax distance: " << m_maxDist << endl;
}

/*------------------------- ExtenderMethod Interface -------------------------*/

template <typename MPTraits>
double
ExtenderMethod<MPTraits>::
GetMinDistance() const {
  return m_minDist;
}


template <typename MPTraits>
double
ExtenderMethod<MPTraits>::
GetMaxDistance() const {
  return m_maxDist;
}

/*----------------------------------------------------------------------------*/

#endif

#ifndef EXTENDER_METHOD_H_
#define EXTENDER_METHOD_H_

#include "LocalPlanners/LPOutput.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Base algorithm abstraction for \ref Extenders.
/// @tparam MPTraits Motion planning universe
///
/// ExtenderMethod has one main method, @c Extend, to grow a simple path from a
/// starting node in some input direction - note that not all expansion
/// methods go in straight lines through @cspace.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class ExtenderMethod : public MPBaseObject<MPTraits> {

  protected:

    ///\name Extender Properties
    ///@{

    double m_minDist;
    double m_maxDist;

    ///@}

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ///@}
    ///\name Construction
    ///@{

    ExtenderMethod(double _min = .001, double _max = 1) :
        m_minDist(_min), m_maxDist(_max) { }

    ExtenderMethod(MPProblemType* _problem, XMLNode& _node) :
        MPBaseObject<MPTraits>(_problem, _node) {
      ParseXML(_node);
    }

    virtual ~ExtenderMethod() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override {
      MPBaseObject<MPTraits>::Print(_os);
      _os << "\tMin distance: " << m_minDist << endl
          << "\tMax distance: " << m_maxDist << endl;
    }

    void ParseXML(XMLNode& _node) {
      m_maxDist = _node.Read("maxDist", false, 1., 0., MAX_DBL, "Maximum "
          "extension distance");
      m_minDist = _node.Read("minDist", false, .001, 0., MAX_DBL, "Minimum "
          "extension distance.");
    }

    ///@}
    ///\name Required Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Maximum extension distance
    virtual double GetMinDistance() const {return m_minDist;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Maximum extension distance
    virtual double GetMaxDistance() const {return m_maxDist;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Extends a path from an input configuration towards a given
    ///        direction
    /// @param _start Initial configuration to grow from.
    /// @param _end   Direction configuration to grow to.
    /// @param _new   Placeholder for resulting configuration.
    /// @param _lp    Placeholder for polygonal chain configurations for
    ///               non-straight-line extention operations and associated
    ///               weight.
    /// @return Success/fail for extention operation
    ///
    /// @usage
    /// @code
    /// ExtenderPointer e = this->GetExtender(m_exLabel);
    /// CfgType start, goal, new;
    /// LPOutput<MPTraits> lp;
    /// bool pass = e->Extend(start, goal, new, lp);
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) = 0;

    ///@}
};

#endif

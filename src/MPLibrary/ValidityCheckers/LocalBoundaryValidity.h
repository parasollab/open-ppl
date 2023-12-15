#ifndef LOCAL_BOUNDARY_VALIDITY__H
#define LOCAL_BOUNDARY_VALIDITY__H

#include "ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// A configuration is considered valid if it is within a given local
/// boundary.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LocalBoundaryValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    LocalBoundaryValidity();
    LocalBoundaryValidity(XMLNode& _node);
    virtual ~LocalBoundaryValidity() = default;

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const string& _callName) override;

    virtual void SetLocalBoundary(Boundary* _boundary) override;

    virtual void SetLocalBoundaries(std::map<Robot*,Boundary*> _boundaries) override;

    ///@}
  private:

    std::map<Robot*,Boundary*> m_localBoundaries;

    Boundary* m_localBoundary{nullptr};

    std::string m_vcLabel;       ///< The VC label, must point to pqp solid.

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
LocalBoundaryValidity<MPTraits>::
LocalBoundaryValidity() {
  this->SetName("LocalBoundaryValidity");
}


template <typename MPTraits>
LocalBoundaryValidity<MPTraits>::
LocalBoundaryValidity(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("LocalBoundaryValidity");
}

/*------------------------- ValidityChecker Interface ------------------------*/

template <typename MPTraits>
bool
LocalBoundaryValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo&, const string&) {
  //if(m_localBoundary->InBoundary(_cfg))
  //std::cout << "LocalValidityChecker using boundary: " << *m_localBoundary <<std::endl;
  //return m_localBoundary->InBoundary(_cfg);

  auto robot = _cfg.GetRobot();
  auto boundary = m_localBoundaries[robot];

  return boundary->InBoundary(_cfg);
}

template<typename MPTraits>
void
LocalBoundaryValidity<MPTraits>::
SetLocalBoundary(Boundary* _boundary) {
  m_localBoundary = _boundary;
}


template<typename MPTraits>
void
LocalBoundaryValidity<MPTraits>::
SetLocalBoundaries(std::map<Robot*,Boundary*> _boundaries) {
  m_localBoundaries = _boundaries;
}
/*----------------------------------------------------------------------------*/

#endif

#ifndef ACTIVE_BODY_EUCLIDEAN_DISTANCE_H_
#define ACTIVE_BODY_EUCLIDEAN_DISTANCE_H_

#include "MinkowskiDistance.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief Measure the standard Euclidean distance between two composite
///        configurations. This is intended for assembly planning, and weights
///        any configuration that moves a body not in the active body list to
///        be the double infinity value.
///
/// Euclidean Distance is Minkowski Distance where r1=r2=2, r3=0.5
/// This allows us to use Minkowski Distance to calculate Euclidean Distance
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ActiveBodyEuclideanDistance : public MinkowskiDistance<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef std::vector<unsigned int> Subassembly;

    ///@name Construction
    ///@{

    ActiveBodyEuclideanDistance(bool _normalize = false);
    ActiveBodyEuclideanDistance(XMLNode& _node);

    ///@}

    void SetActiveBodies(const Subassembly& _sub);

//    virtual double PositionDistance(const CfgType& _c) override;
//    virtual double OrientationDistance(const CfgType& _c) override;

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

  private:

    Subassembly m_activeBodies;
    Subassembly m_inactiveBodies;
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ActiveBodyEuclideanDistance<MPTraits>::
ActiveBodyEuclideanDistance(bool _normalize) :
    MinkowskiDistance<MPTraits>(2, 2, 1. / 2, _normalize) {
  this->SetName("ActiveBodyEuclidean");
}


template <typename MPTraits>
ActiveBodyEuclideanDistance<MPTraits>::
ActiveBodyEuclideanDistance(XMLNode& _node) : MinkowskiDistance<MPTraits>(_node) {
  this->SetName("ActiveBodyEuclidean");

  this->m_r1 = 2;
  this->m_r2 = 2;
  this->m_r3 = 1.0/2;
  this->m_normalize = _node.Read("normalize", false, false,
      "flag if position dof should be normalized by environment diagonal");
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
void
ActiveBodyEuclideanDistance<MPTraits>::
SetActiveBodies(const Subassembly& _sub) {
  if(_sub != m_activeBodies) {
    const MultiBody* const mb = this->GetTask()->GetRobot()->GetMultiBody();
    const unsigned int numBodies = mb->GetNumBodies();
    m_activeBodies = _sub;
    m_inactiveBodies.resize(numBodies);

    for(unsigned int i = 0; i < numBodies; ++i)
      m_inactiveBodies[i] = i;
    for(const unsigned int body : _sub)
      m_inactiveBodies.erase(remove(m_inactiveBodies.begin(),
          m_inactiveBodies.end(), body), m_inactiveBodies.end());
  }
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
ActiveBodyEuclideanDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  // Initialize a cfg to have max cfg values, divided by the number of Dofs,(so
  // we don't overflow the double's value).
  const unsigned int dofsPerBody = _c1.PosDOF() + _c1.OriDOF();
  const CfgType diff = _c1 - _c2;

  //First check for bad vals:
  for(const unsigned int body : m_inactiveBodies) {
    for(unsigned int i = 0; i < dofsPerBody; ++i) {
      const unsigned int inactiveInd = body*dofsPerBody + i;
      if(fabs(diff[inactiveInd]) > 1e-10)
        return std::numeric_limits<double>::infinity();//Short circuit, invalid connection.
    }
  }
  //If a valid moving body, then accumulate the difference:
  double distance = 0.;
  for(const unsigned int body : m_activeBodies) {
    for(unsigned int i = 0; i < dofsPerBody; ++i) {
      const unsigned int activeInd = body*dofsPerBody + i;
      distance += pow(fabs(diff[activeInd]), this->m_r2);//r1 == r2 for us.
    }
  }

  return pow(distance, this->m_r3);
}

/*----------------------------------------------------------------------------*/

#endif

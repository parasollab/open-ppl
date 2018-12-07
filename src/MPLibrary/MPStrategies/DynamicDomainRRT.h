#ifndef PMPL_DYNAMIC_DOMAIN_RRT_H_
#define PMPL_DYNAMIC_DOMAIN_RRT_H_

#include "BasicRRTStrategy.h"


////////////////////////////////////////////////////////////////////////////////
/// Implementation of the Dynamic Domain RRT algorithm.
///
/// Dynamic Domain RRT is identical to regular RRT, except that the accepted
/// Voronoi region for extending a node is diminished to some 'dynamic domain
/// radius' after the first failed extension. I.e., after one failed extension
/// from a node q, further extensions will be rejected unless the growth target
/// is within the dynamic domain radius of q.
///
/// Reference:
///   Yershova, Anna, Sim, Thierry, and Lavalle, Steven M., "Dynamic-Domain RRTs:
///   Efficient Exploration by Controlling the Sampling Domain," ICRA 2005.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DynamicDomainRRT : public BasicRRTStrategy<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    DynamicDomainRRT();

    DynamicDomainRRT(XMLNode& _node);

    virtual ~DynamicDomainRRT() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name RRT Overrides
    ///@{

    virtual VID Extend(const VID _nearVID, const CfgType& _qRand,
        LPOutput<MPTraits>& _lp, const bool _requireNew = false) override;

    virtual VID ExpandTree(const VID _nearestVID, const CfgType& _target)
        override;

    ///@}
    ///@name Helpers
    ///@{

    /// @return String for accessing Cfg::GetStat for radius value
    static constexpr std::string RLabel() {return "DDRRT::R";}

    ///@}
    ///@name Internal State
    ///@{

    /// The dynamic domain radius factor. When we fail to extend from a
    /// configuration, its dynamic domain is set to this times the extender max
    /// distance. Future extensions from this node are aborted if the target is
    /// outside its dynamic domain.
    double m_r{2.};

    std::string m_dmLabel; ///< DM for measuring dynamic domain.

    ///@}

};

/*---------------------------- Construction ----------------------------------*/

template <typename MPTraits>
DynamicDomainRRT<MPTraits>::
DynamicDomainRRT() : BasicRRTStrategy<MPTraits>() {
  this->SetName("DynamicDomainRRT");
}


template <typename MPTraits>
DynamicDomainRRT<MPTraits>::
DynamicDomainRRT(XMLNode& _node) : BasicRRTStrategy<MPTraits>(_node) {
  this->SetName("DynamicDomainRRT");

  m_r = _node.Read("r", true, m_r, 0., std::numeric_limits<double>::max(),
      "Dynamic domain factor. This is a multiple of extender max distance.");

  m_dmLabel = _node.Read("dmLabel", true, "",
      "Distance metric for measuring dynamic domain inclusion.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
DynamicDomainRRT<MPTraits>::
Print(std::ostream& _os) const {
  BasicRRTStrategy<MPTraits>::Print(_os);
  _os << "\tDynamic Domain radius: " << m_r
      << "\n\tDynamic Domain distance metric: " << m_dmLabel
      << std::endl;
}

/*-------------------------- MPStrategy Overrides ----------------------------*/

template <typename MPTraits>
void
DynamicDomainRRT<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();
  for(auto v : *this->GetRoadmap())
    v.property().SetStat(RLabel(), std::numeric_limits<double>::max());
}

/*----------------------------- RRT Overrides --------------------------------*/

template <typename MPTraits>
typename DynamicDomainRRT<MPTraits>::VID
DynamicDomainRRT<MPTraits>::
Extend(const VID _nearVID, const CfgType& _qRand, LPOutput<MPTraits>& _lp,
    const bool _requireNew) {
  // As basic RRT's extend, but do some stuff on failure.
  const VID newVID = BasicRRTStrategy<MPTraits>::Extend(_nearVID, _qRand, _lp,
      _requireNew);

  if(newVID != INVALID_VID) {
    CfgType& cfg = this->GetRoadmap()->GetVertex(newVID);
    cfg.SetStat(RLabel(), std::numeric_limits<double>::max());
  }
  else {
    auto e = this->GetExtender(this->m_exLabel);
    CfgType& cfg = this->GetRoadmap()->GetVertex(_nearVID);
    cfg.SetStat(RLabel(), m_r * e->GetMaxDistance());
  }
  return newVID;
}


template <typename MPTraits>
typename DynamicDomainRRT<MPTraits>::VID
DynamicDomainRRT<MPTraits>::
ExpandTree(const VID _nearestVID, const CfgType& _target) {
  const CfgType& cfg = this->GetRoadmap()->GetVertex(_nearestVID);

  // If the nearest node's radius is too small, return failure.
  auto dm = this->GetDistanceMetric(m_dmLabel);
  if(dm->Distance(cfg, _target) >= cfg.GetStat(RLabel()))
    return INVALID_VID;
  else
    return BasicRRTStrategy<MPTraits>::ExpandTree(_nearestVID, _target);
}

/*----------------------------------------------------------------------------*/

#endif

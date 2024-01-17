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
///   Yershova, Anna, Sim, Thierry, and Lavalle, Steven M., "Dynamic-Domain
///   RRTs: Efficient Exploration by Controlling the Sampling Domain," ICRA
///   2005.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class DynamicDomainRRT : public BasicRRTStrategy {
 public:
  ///@name Motion Planning Types
  ///@{

  typedef typename MPBaseObject::WeightType WeightType;
  typedef typename MPBaseObject::RoadmapType RoadmapType;
  typedef typename RoadmapType::VID VID;

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

  /// As basic RRT, but set a limiting dynamic domain on the source node
  /// whenever the extension fails.
  virtual VID Extend(const VID _nearVID,
                     const Cfg& _qRand,
                     LPOutput& _lp,
                     const bool _requireNew = false) override;

  /// As basic RRT, but reject attempts to extend towards targets beyond the
  /// dynamic domain.
  virtual VID ExpandTree(const VID _nearestVID, const Cfg& _target) override;

  ///@}
  ///@name Helpers
  ///@{

  /// @return String for accessing Cfg::GetStat for radius value
  static std::string RLabel() { return "DDRRT::R"; }

  ///@}
  ///@name Internal State
  ///@{

  /// The dynamic domain radius factor. When we fail to extend from a
  /// configuration, its dynamic domain is set to this times the extender max
  /// distance. Future extensions from this node are aborted if the target is
  /// outside its dynamic domain.
  double m_r{2.};

  std::string m_dmLabel;  ///< DM for measuring dynamic domain.

  ///@}
};

#endif

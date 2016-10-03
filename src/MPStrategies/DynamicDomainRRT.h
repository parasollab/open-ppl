#ifndef DYNAMIC_DOMAIN_RRT_H_
#define DYNAMIC_DOMAIN_RRT_H_

#include "BasicRRTStrategy.h"

#include "Environment/BoundingBox.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Dynamic Domain RRT adapts the sampling space of RRTs to better
///        approximate boundary information
/// @tparam MPTraits Motion planning universe
///
/// Dynamic Domain RRT considers a bounding sphere around each configuration.
/// During RRT sampling only nodes within some \f$q_{near}\f$'s radius. If
/// extension is unsuccessful then \f$q_{near}\f$'s radius is set to \f$R\f$. If
/// successful then \f$q_{new}\f$'s radius is set to \f$\inf\f$.
///
/// Yershova, Anna, Sim, Thierry, and Lavalle, Steven M., "Dynamic-Domain RRTs:
/// Efficient Exploration by Controlling the Sampling Domain," Proc. of the Int.
/// Conf. on Robotics and Automation (ICRA), pp. 3856-3861, Barcelona, Spain,
/// Apr. 2005.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class DynamicDomainRRT : public BasicRRTStrategy<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::CfgRef           CfgRef;
    typedef typename MPTraits::WeightType       WeightType;
    typedef typename MPTraits::MPProblemType    MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType   GraphType;
    typedef typename MPProblemType::VID         VID;

    ///}@
    ///\name Construction
    ///@{

    DynamicDomainRRT(string _dm = "euclidean", string _nf = "bfnf",
        string _vc = "rapid", string _nc = "kClosest", string _ex = "BERO",
        vector<string> _evaluators = vector<string>(),
        string _gt = "UNDIRECTED_TREE",  bool _growGoals = false,
        double _growthFocus = .05, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3, double _r = 10.);

    DynamicDomainRRT(MPProblemType* _problem, XMLNode& _node);

    virtual ~DynamicDomainRRT() = default;

    ///@}
    ///\name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const;

    ///@}

  protected:

    ///\name RRT Overrides
    ///@{

    virtual VID ExpandTree(const VID _nearestVID, const CfgType& _target)
        override;
    virtual VID Extend(const VID _nearVID, const CfgType& _qRand,
        const bool _lp = false) override;

    ///@}
    ///\name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return String for accessing Cfg::GetStat for radius value
    static constexpr string RLabel() {return "DDRRT::R";}

    ///@}
    ///\name Internal State
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief The dynamic domain radius factor. When we fail to extend from a
    ///        configuration, its dynamic domain is set to this times the
    ///        extender max distance. Future extensions from this node are
    ///        aborted if the target is outside its dynamic domain.
    double m_r;

    ///@}
};

/*---------------------------- Construction ----------------------------------*/

template<class MPTraits>
DynamicDomainRRT<MPTraits>::
DynamicDomainRRT(string _dm, string _nf, string _vc, string _nc,
    string _ex, vector<string> _evaluators, string _gt, bool _growGoals,
    double _growthFocus, size_t _numRoots, size_t _numDirections,
    size_t _maxTrial, double _r) :
    BasicRRTStrategy<MPTraits>(_dm, _nf, _vc, _nc, _ex, _evaluators, _gt,
        _growGoals, _growthFocus, _numRoots, _numDirections, _maxTrial),
    m_r(_r) {
  this->SetName("DynamicDomainRRT");
}


template<class MPTraits>
DynamicDomainRRT<MPTraits>::
DynamicDomainRRT(MPProblemType* _problem, XMLNode& _node) :
    BasicRRTStrategy<MPTraits>(_problem, _node) {
  this->SetName("DynamicDomainRRT");
  m_r = _node.Read("r", true, 2., 0., numeric_limits<double>::max(),
      "Dynamic domain factor. This is a multiple of extender max distance.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template<class MPTraits>
void
DynamicDomainRRT<MPTraits>::
Print(ostream& _os) const {
  BasicRRTStrategy<MPTraits>::Print(_os);
  _os << "\tr:: " << m_r << endl;
}

/*-------------------------- MPStrategy Overrides ----------------------------*/

template<class MPTraits>
void
DynamicDomainRRT<MPTraits>::
Initialize() {
  BasicRRTStrategy<MPTraits>::Initialize();
  for(auto v  : *this->GetRoadmap()->GetGraph())
    v.property().SetStat(RLabel(), numeric_limits<double>::max());
}

/*----------------------------- RRT Overrides --------------------------------*/

template <typename MPTraits>
typename DynamicDomainRRT<MPTraits>::VID
DynamicDomainRRT<MPTraits>::
Extend(const VID _nearVID, const CfgType& _qRand, const bool _lp) {
  // As basic RRT's extend, but do some stuff on failure.
  VID newVID = BasicRRTStrategy<MPTraits>::Extend(_nearVID, _qRand, _lp);

  if(newVID != INVALID_VID) {
    CfgType& cfg = this->GetRoadmap()->GetGraph()->GetVertex(newVID);
    cfg.SetStat(RLabel(), numeric_limits<double>::max());
  }
  else {
    CfgType& cfg = this->GetRoadmap()->GetGraph()->GetVertex(_nearVID);
    cfg.SetStat(RLabel(), m_r);
  }
  return newVID;
}


template<class MPTraits>
typename DynamicDomainRRT<MPTraits>::VID
DynamicDomainRRT<MPTraits>::
ExpandTree(const VID _nearestVID, const CfgType& _target) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  const CfgType& cfg = this->GetRoadmap()->GetGraph()->GetVertex(_nearestVID);

  // If the nearest node's radius is too small, return failure.
  if(dm->Distance(cfg, _target) >= cfg.GetStat(RLabel()))
    return INVALID_VID;
  else
    return BasicRRTStrategy<MPTraits>::ExpandTree(_nearestVID, _target);
}

/*----------------------------------------------------------------------------*/

#endif

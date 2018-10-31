#ifndef PMPL_MEDIAL_AXIS_EXTENDER_H_
#define PMPL_MEDIAL_AXIS_EXTENDER_H_

#include "ExtenderMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Extends along medial axis of @cfree.
///
/// @todo This method needs to have its dedicated MA tool removed and replaced
///       with an MA label (fetch from MPTools).
///
/// Extend along the medial axis of @cfree from \f$q_{near}\f$ towards
/// \f$q_{dir}\f$ until either \f$q_{dir}\f$ is reached, a distance of
/// \f$\Delta q\f$ is extended, or no progress is made.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MedialAxisExtender : public ExtenderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    MedialAxisExtender();

    MedialAxisExtender(XMLNode& _node);

    virtual ~MedialAxisExtender() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    MedialAxisUtility<MPTraits> m_medialAxisUtility;
                                ///< Tool for pushing Cfgs to the medial axis.

    double m_extendDist;        ///< The step size to use for medial-axis push.
    size_t m_maxIntermediates;  ///< The maximum number of steps to make.
    string m_lpLabel;           ///< The local planner for connecting steps.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
MedialAxisExtender<MPTraits>::
MedialAxisExtender() {
  this->SetName("MedialAxisExtender");
}


template <typename MPTraits>
MedialAxisExtender<MPTraits>::
MedialAxisExtender(XMLNode& _node) : ExtenderMethod<MPTraits>(_node),
    m_medialAxisUtility(_node) {
  this->SetName("MedialAxisExtender");

  m_extendDist = _node.Read("extendDist", true, 0.5, 0.0, MAX_DBL, "Step size");
  m_maxIntermediates = _node.Read("maxIntermediates", false, 10, 1, MAX_INT,
      "Maximum number of intermediates on an edge");
  m_lpLabel = _node.Read("lpLabel", true, "",
      "Local Planner between intermediates");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
MedialAxisExtender<MPTraits>::
Print(ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  m_medialAxisUtility.Print(_os);
  _os << "\textend dist: " << m_extendDist << endl;
  _os << "\tmax intermediates: " << m_maxIntermediates << endl;
  _os << "\tlocal planner label: \"" << m_lpLabel << "\"" << endl;
}

/*-------------------------- ExtenderMethod Overrides ------------------------*/

template <typename MPTraits>
bool
MedialAxisExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp) {
  //Setup

  if(!m_medialAxisUtility.IsInitialized()) {
    m_medialAxisUtility.SetMPLibrary(this->GetMPLibrary());
    m_medialAxisUtility.Initialize();
  }

  Environment* env = this->GetEnvironment();
  auto dm = this->GetDistanceMetric(m_medialAxisUtility.GetDistanceMetricLabel());
  auto lp = this->GetLocalPlanner(m_lpLabel);

  LPOutput<MPTraits> lpOutput;

  CfgType tick = _start, curr(this->GetTask()->GetRobot());
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  double dist = 0, length = 0;

  VDClearAll();
  VDAddTempCfg(_start, false);
  VDAddTempCfg(_end, false);

  do {
    curr = tick;
    length += dist;
    _lp.m_intermediates.push_back(curr);

    if(_lp.m_intermediates.size() > m_maxIntermediates)
      break;
    //take a step at distance _extendDist
    CfgType incr = _end - curr;
    dm->ScaleCfg(m_extendDist, incr);
    tick = curr + incr;

    VDAddTempCfg(curr, true);
    VDAddTempCfg(tick, true);
    VDClearLastTemp();

    /*cout << "\nExpanding near: " << _start << " to " << _end << endl;
    cout << "curr: " << curr << endl;
    cout << "tick: " << tick << endl;
    */
    //Push tick to the MA
    if(!m_medialAxisUtility.PushToMedialAxis(tick, env->GetBoundary())) {
      if(this->m_debug)
        cout << "PushToMedialAxis failed...MARRTExpand failed" << endl;
      break;
    }

    VDAddTempCfg(tick, true);
    VDAddTempEdge(curr, tick);
    VDAddTempEdge(curr, tick);
    dist = dm->Distance(curr, tick);

    /*cout << "tick pushed: " << tick << endl;
    cout << "distance stepped: " << dist << endl;
    */

    //stop on 1/3 conditions
    //1) not enough progress is made along extension
    //2) no simple path exists between adjacent configurations
    //3) traveled too far
  } while(
      dist > this->m_minDist
      && lp->IsConnected(curr, tick, &lpOutput, positionRes, orientationRes)
      && length + dist <= this->m_maxDist
      );

  _lp.m_intermediates.erase(_lp.m_intermediates.begin());
  if(_lp.m_intermediates.empty())
    return false;
  else {
    _new = _lp.m_intermediates.back();
    _lp.m_intermediates.pop_back();
    return true;
  }
}

/*----------------------------------------------------------------------------*/

#endif

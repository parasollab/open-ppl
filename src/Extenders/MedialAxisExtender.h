#ifndef MEDIAL_AXIS_EXTENDER_H_
#define MEDIAL_AXIS_EXTENDER_H_

#include "ExtenderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Extends along medial axis of @cfree.
/// @tparam MPTraits Motion planning universe
///
/// Extend along the medial axis of @cfree from \f$q_{near}\f$ towards
/// \f$q_{dir}\f$ until either \f$q_{dir}\f$ is reached, a distance of
/// \f$\Delta q\f$ is extended, or no progress is made.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MedialAxisExtender : public ExtenderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;

    MedialAxisExtender(const MedialAxisUtility<MPTraits>& _medialAxisUtility =
        MedialAxisUtility<MPTraits>(),
        double _delta = 1.0, double _minDist = 0.001, double _extendDist = 0.5,
        size_t _maxIntermediates = 10, const string& _lpLabel = "");
    MedialAxisExtender(MPProblemType* _problem, XMLNode& _node);

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, LPOutput<MPTraits>& _lpOutput);

  private:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;
    double m_delta, m_minDist, m_extendDist;
    size_t m_maxIntermediates;
    string m_lpLabel;
};

template<class MPTraits>
MedialAxisExtender<MPTraits>::
MedialAxisExtender(const MedialAxisUtility<MPTraits>& _medialAxisUtility,
    double _delta, double _minDist, double _extendDist,
    size_t _maxIntermediates, const string& _lpLabel) :
  ExtenderMethod<MPTraits>(), m_medialAxisUtility(_medialAxisUtility),
  m_delta(_delta), m_minDist(_minDist), m_extendDist(_extendDist),
  m_maxIntermediates(_maxIntermediates), m_lpLabel(_lpLabel) {
    this->SetName("MedialAxisExtender");
  }

template<class MPTraits>
MedialAxisExtender<MPTraits>::
MedialAxisExtender(MPProblemType* _problem, XMLNode& _node) :
  ExtenderMethod<MPTraits>(_problem, _node),
  m_medialAxisUtility(_problem, _node) {
    this->SetName("MedialAxisExtender");
    ParseXML(_node);
  }

template<class MPTraits>
void
MedialAxisExtender<MPTraits>::
ParseXML(XMLNode& _node) {
  m_delta = _node.Read("delta", false, 1.0, 0.0, MAX_DBL, "Delta distance");
  m_minDist = _node.Read("minDist", true, 0.001, 0.0, MAX_DBL,
      "Minimum expansion between each step");
  m_extendDist = _node.Read("extendDist", true, 0.5, 0.0, MAX_DBL, "Step size");
  m_maxIntermediates = _node.Read("maxIntermediates", false, 10, 1, MAX_INT,
      "Maximum number of intermediates on an edge");
  m_lpLabel = _node.Read("lpLabel", true, "",
      "Local Planner between intermediates");
}

template<class MPTraits>
void
MedialAxisExtender<MPTraits>::
Print(ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  m_medialAxisUtility.Print(_os);
  _os << "\tdelta: " << m_delta << endl;
  _os << "\tmin dist: " << m_minDist << endl;
  _os << "\textend dist: " << m_extendDist << endl;
  _os << "\tmax intermediates: " << m_maxIntermediates << endl;
  _os << "\tlocal planner label: \"" << m_lpLabel << "\"" << endl;
}

template<class MPTraits>
bool
MedialAxisExtender<MPTraits>::
Extend(const CfgType& _near, const CfgType& _dir,
    CfgType& _new, LPOutput<MPTraits>& _lpOutput) {
  //Setup
  Environment* env = this->GetEnvironment();
  DistanceMetricPointer dm =
    this->GetDistanceMetric(m_medialAxisUtility.GetDistanceMetricLabel());
  LocalPlannerPointer lp = this->GetLocalPlanner(m_lpLabel);

  LPOutput<MPTraits> lpOutput;

  CfgType tick = _near, curr;
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  double dist = 0, length = 0;

  VDClearAll();
  VDAddTempCfg(_near, false);
  VDAddTempCfg(_dir, false);

  do {
    curr = tick;
    length += dist;
    _lpOutput.m_intermediates.push_back(curr);

    if(_lpOutput.m_intermediates.size() > m_maxIntermediates)
      break;
    //take a step at distance _extendDist
    CfgType incr = _dir - curr;
    dm->ScaleCfg(m_extendDist, incr);
    tick = curr + incr;

    VDAddTempCfg(curr, true);
    VDAddTempCfg(tick, true);
    VDClearLastTemp();

    /*cout << "\nExpanding near: " << _near << " to " << _dir << endl;
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
      dist > m_minDist
      && lp->IsConnected(curr, tick, &lpOutput, positionRes, orientationRes)
      && length + dist <= m_delta
      );

  _lpOutput.m_intermediates.erase(_lpOutput.m_intermediates.begin());
  if(_lpOutput.m_intermediates.empty())
    return false;
  else {
    _new = _lpOutput.m_intermediates.back();
    _lpOutput.m_intermediates.pop_back();
    return true;
  }
}

#endif

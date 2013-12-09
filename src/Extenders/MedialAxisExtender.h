/*
 * =============================================================================
 *
 *       Filename:  MedialAxisExtender.h
 *
 *    Description:  Restricts expansion of a tree on/near the medial axis of the
 *    free space.
 *
 * =============================================================================
 */
#ifndef MEDIALAXISEXTENDER_H_
#define MEDIALAXISEXTENDER_H_

#include "ExtenderMethod.h"

template<class MPTraits>
class MedialAxisExtender : public ExtenderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;

    MedialAxisExtender(const MedialAxisUtility<MPTraits>& _medialAxisUtility = MedialAxisUtility<MPTraits>(),
        double _delta = 1.0, double _minDist = 0.001, double _extendDist = 0.5,
        size_t _maxIntermediates = 10, const string& _lpLabel = "");
    MedialAxisExtender(MPProblemType* _problem, XMLNodeReader& _node);

    void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os) const;

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, vector<CfgType>& _innerNodes);

  private:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;
    double m_delta, m_minDist, m_extendDist;
    size_t m_maxIntermediates;
    string m_lpLabel;
};

template<class MPTraits>
MedialAxisExtender<MPTraits>::MedialAxisExtender(const MedialAxisUtility<MPTraits>& _medialAxisUtility,
    double _delta, double _minDist, double _extendDist,
    size_t _maxIntermediates, const string& _lpLabel) :
  ExtenderMethod<MPTraits>(), m_medialAxisUtility(_medialAxisUtility),
  m_delta(_delta), m_minDist(_minDist), m_extendDist(_extendDist),
  m_maxIntermediates(_maxIntermediates), m_lpLabel(_lpLabel) {
    this->SetName("MedialAxisExtender");
  }

template<class MPTraits>
MedialAxisExtender<MPTraits>::MedialAxisExtender(MPProblemType* _problem, XMLNodeReader& _node) :
  ExtenderMethod<MPTraits>(_problem, _node), m_medialAxisUtility(_problem, _node) {
    this->SetName("MedialAxisExtender");
    ParseXML(_node);
  }

template<class MPTraits>
void
MedialAxisExtender<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_delta = _node.numberXMLParameter("delta", false, 1.0, 0.0, MAX_DBL, "Delta distance");
  m_minDist = _node.numberXMLParameter("minDist", true, 0.001, 0.0, MAX_DBL, "Minimum expansion between each step");
  m_extendDist = _node.numberXMLParameter("extendDist", true, 0.5, 0.0, MAX_DBL, "Step size");
  m_maxIntermediates = _node.numberXMLParameter("maxIntermediates", false, 10, 1, MAX_INT, "Maximum number of intermediates on an edge");
  m_lpLabel = _node.stringXMLParameter("lpLabel", true, "", "Local Planner between intermediates");
}

template<class MPTraits>
void
MedialAxisExtender<MPTraits>::PrintOptions(ostream& _os) const {
  ExtenderMethod<MPTraits>::PrintOptions(_os);
  m_medialAxisUtility.PrintOptions(_os);
  _os << "\tdelta: " << m_delta << endl;
  _os << "\tmin dist: " << m_minDist << endl;
  _os << "\textend dist: " << m_extendDist << endl;
  _os << "\tmax intermediates: " << m_maxIntermediates << endl;
  _os << "\tlocal planner label: \"" << m_lpLabel << "\"" << endl;
}

template<class MPTraits>
bool
MedialAxisExtender<MPTraits>::Extend(const CfgType& _near, const CfgType& _dir,
    CfgType& _new, vector<CfgType>& _innerNodes) {
  //Setup
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_medialAxisUtility.GetDistanceMetricLabel());
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);

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
    _innerNodes.push_back(curr);

    if(_innerNodes.size() > m_maxIntermediates)
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
      if(this->m_debug) cout << "PushToMedialAxis failed...MARRTExpand failed" << endl;
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

  _innerNodes.erase(_innerNodes.begin());
  if(_innerNodes.empty())
    return false;
  else {
    _new = _innerNodes.back();
    _innerNodes.pop_back();
    return true;
  }
}

#endif


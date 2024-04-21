#include "MedialAxisExtender.h"
#include "MPLibrary/MPLibrary.h"
/*------------------------------- Construction -------------------------------*/


MedialAxisExtender::
MedialAxisExtender() {
  this->SetName("MedialAxisExtender");
}



MedialAxisExtender::
MedialAxisExtender(XMLNode& _node) : ExtenderMethod(_node),
    m_medialAxisUtility(_node) {
  this->SetName("MedialAxisExtender");

  m_extendDist = _node.Read("extendDist", true, 0.5, 0.0, MAX_DBL, "Step size");
  m_maxIntermediates = _node.Read("maxIntermediates", false, 10, 1, MAX_INT,
      "Maximum number of intermediates on an edge");
  m_lpLabel = _node.Read("lpLabel", true, "",
      "Local Planner between intermediates");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/


void
MedialAxisExtender::
Print(ostream& _os) const {
  ExtenderMethod::Print(_os);
  m_medialAxisUtility.Print(_os);
  _os << "\textend dist: " << m_extendDist << endl;
  _os << "\tmax intermediates: " << m_maxIntermediates << endl;
  _os << "\tlocal planner label: \"" << m_lpLabel << "\"" << endl;
}

/*-------------------------- ExtenderMethod Overrides ------------------------*/


bool
MedialAxisExtender::
Extend(const Cfg& _start, const Cfg& _end, Cfg& _new, LPOutput& _lp) {
  //Setup

  if(!m_medialAxisUtility.IsInitialized()) {
    m_medialAxisUtility.SetMPLibrary(this->GetMPLibrary());
    m_medialAxisUtility.Initialize();
  }

  Environment* env = this->GetEnvironment();
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_medialAxisUtility.GetDistanceMetricLabel());
  auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);

  LPOutput lpOutput;

  Cfg tick = _start, curr(this->GetTask()->GetRobot());
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
    Cfg incr = _end - curr;
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
        cout << "PushToMedialAxis failed...MedialAxisRRTExpand failed" << endl;
      break;
    }

    VDAddTempCfg(tick, true);
    VDAddTempEdge(curr, tick);
    VDAddTempEdge(curr, tick);
    dist = dm->Distance(curr, tick);
    std::cout << "DISTANCE " << dist << std::endl;

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

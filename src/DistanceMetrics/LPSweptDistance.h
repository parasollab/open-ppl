#ifndef LPSWEPTDISTANCE_H_
#define LPSWEPTDISTANCE_H_

#include "DistanceMetricMethod.h"
#include "MPProblem/Environment.h"
#include "LocalPlanners/LPOutput.h"

class StatClass;

template<class MPTraits>
class LPSweptDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    LPSweptDistance(string _lpLabel="", double _positionRes = 0.1,
        double _orientationRes = 0.1, bool _bbox = false);
    LPSweptDistance(MPProblemType* _problem, XMLNodeReader& _node, bool _warn = true);
    virtual ~LPSweptDistance();

    virtual void PrintOptions(ostream& _os) const;

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);

  protected:
    double SweptDistance(const vector<GMSPolyhedron>& _poly1, const vector<GMSPolyhedron>& _poly2);

    string m_lpLabel;
    double m_positionRes, m_orientationRes;
    bool m_useBBox;
};

template<class MPTraits>
LPSweptDistance<MPTraits>::LPSweptDistance(string _lpLabel, double _posRes,
    double _oriRes, bool _bbox) :
  DistanceMetricMethod<MPTraits>(), m_lpLabel(_lpLabel), m_positionRes(_posRes),
  m_orientationRes(_oriRes), m_useBBox(_bbox) {
    this->SetName("LPSwept");
  }

template<class MPTraits>
LPSweptDistance<MPTraits>::LPSweptDistance(MPProblemType* _problem,
    XMLNodeReader& _node, bool _warn) :
  DistanceMetricMethod<MPTraits>(_problem, _node, false) {
    this->SetName("LPSwept");
    m_positionRes = _node.numberXMLParameter("posRes", false, _problem->GetEnvironment()->GetPositionRes(), 0.0, 1000.0, "position resolution");
    m_orientationRes = _node.numberXMLParameter("oriRes", false, _problem->GetEnvironment()->GetOrientationRes(), 0.0, 1000.0, "orientation resolution");
    m_useBBox = _node.boolXMLParameter("useBBox", false, false, "use bbox instead of robot vertices");
    m_lpLabel = _node.stringXMLParameter("lpLabel", true, "", "Local Planner");

    if(_warn)
      _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
LPSweptDistance<MPTraits>::~LPSweptDistance() {
}

template<class MPTraits>
void
LPSweptDistance<MPTraits>::PrintOptions(ostream& _os) const {
  DistanceMetricMethod<MPTraits>::PrintOptions(_os);
  _os << "\tpositionRes = " << m_positionRes << endl;
  _os << "\torientationRes = " << m_orientationRes << endl;
  _os << "\tuseBBox = " << m_useBBox << endl;
}

template<class MPTraits>
double
LPSweptDistance<MPTraits>::Distance(const CfgType& _c1, const CfgType& _c2) {
  if (_c1.GetRobotIndex() != _c2.GetRobotIndex()){
    cerr << "LPSweptDistance::Distance error - the cfgs reference different multibodies" << endl;
    exit(1);
  }
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass stats;
  LocalPlannerPointer lpMethod = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);
  LPOutput<MPTraits> lpOutput;
  CfgType dummy;

  lpMethod->IsConnected(_c1, _c2, dummy, &lpOutput, m_positionRes, m_orientationRes, false, true);
  //lpPath does not include _c1 and _c2, so adding them manually
  vector<CfgType> cfgs(1, _c1);
  cfgs.insert(cfgs.end(), lpOutput.m_path.begin(), lpOutput.m_path.end());
  cfgs.push_back(_c2);
  double d = 0;
  vector<GMSPolyhedron> poly2;
  int robot = _c1.GetRobotIndex();
  int bodyCount = env->GetMultiBody(robot)->GetFreeBodyCount();
  cfgs.begin()->ConfigEnvironment(env);
  for(int b=0; b<bodyCount; ++b)
    if(m_useBBox)
      poly2.push_back(env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldBoundingBox());
    else
      poly2.push_back(env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldPolyhedron());
  for(typename vector<CfgType>::const_iterator cit = cfgs.begin(); cit+1 != cfgs.end(); ++cit) {
    vector<GMSPolyhedron> poly1(poly2);
    poly2.clear();
    (cit+1)->ConfigEnvironment(env);
    for(int b=0; b<bodyCount; ++b)
      if(m_useBBox)
        poly2.push_back(env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldBoundingBox());
      else
        poly2.push_back(env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldPolyhedron());
    d += SweptDistance(poly1, poly2);
  }
  return d;
}

template<class MPTraits>
double
LPSweptDistance<MPTraits>::SweptDistance(const vector<GMSPolyhedron>& _poly1, const vector<GMSPolyhedron>& _poly2) {
  double d = 0;
  int count = 0;
  for(size_t b=0; b<_poly1.size(); ++b)
    for(size_t i=0; i<_poly1[b].m_vertexList.size(); ++i) {
      d += (_poly1[b].m_vertexList[i] - _poly2[b].m_vertexList[i]).norm();
      count++;
  }
  return count ? d/(double)count : 0;
}

#endif

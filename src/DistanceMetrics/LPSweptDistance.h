#ifndef LPSWEPTDISTANCE_H_
#define LPSWEPTDISTANCE_H_

#include "DistanceMetricMethod.h"
#include "MPProblem/Environment.h"
#include "LocalPlanners/LPOutput.h"

template<class MPTraits>
class LPSweptDistance : public DistanceMetricMethod<MPTraits> {
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::WeightType WeightType;
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;


  public:
    LPSweptDistance(MPProblemType* _problem, XMLNodeReader& _node, bool _warn = true);
    LPSweptDistance(string _lpLabel="", double _positionRes = 0.1, double _orientationRes = 0.1, bool _bbox = false);
    virtual ~LPSweptDistance();

    virtual void PrintOptions(ostream& _os) const;
    
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    
  protected:
    double SweptDistance(const vector<GMSPolyhedron>& _poly1, const vector<GMSPolyhedron>& _poly2);
    
    string m_lpLabel;
    double m_positionRes, m_orientationRes;
    bool m_useBbox;
};


template<class MPTraits>
LPSweptDistance<MPTraits>::
LPSweptDistance(MPProblemType* _problem, XMLNodeReader& _node, bool _warn) : DistanceMetricMethod<MPTraits>(_problem, _node, false) {
  this->m_name = "LPSwept";
  this->m_positionRes = _node.numberXMLParameter("posRes", false, _problem->GetEnvironment()->GetPositionRes(), 0.0, 1000.0, "position resolution");
  this->m_orientationRes = _node.numberXMLParameter("oriRes", false, _problem->GetEnvironment()->GetOrientationRes(), 0.0, 1000.0, "orientation resolution");
  this->m_useBbox = _node.boolXMLParameter("useBBox", false, false, "use bbox instead of robot vertices");
  this->m_lpLabel = _node.stringXMLParameter("lpLabel", true, "", "Local Planner");
  
  if(_warn)
    _node.warnUnrequestedAttributes();
}

template<class MPTraits>
LPSweptDistance<MPTraits>::
LPSweptDistance(string _lpLabel, double _posRes, double _oriRes, bool _bbox) : 
  DistanceMetricMethod<MPTraits>(), m_lpLabel(_lpLabel), m_positionRes(_posRes), m_orientationRes(_oriRes), m_useBbox(_bbox) {
    this->m_name = "LPSwept";
}

template<class MPTraits>
LPSweptDistance<MPTraits>::~LPSweptDistance() {}

template<class MPTraits>
void LPSweptDistance<MPTraits>::PrintOptions(ostream& _os) const {
  DistanceMetricMethod<MPTraits>::PrintOptions(_os); 
  _os << "\tpositionRes = " << this->m_positionRes;
  _os << "\torientationRes = " << this->m_orientationRes;
  _os << "\tuse_bbox = " << this->m_useBbox;
}

template<class MPTraits>
double LPSweptDistance<MPTraits>::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  StatClass stats;
  shared_ptr<DistanceMetricMethod<MPTraits> > dm;
  LPOutput<MPTraits> lpOutput;
  LocalPlannerPointer lpMethod = this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel);
  CfgType dummy;
  lpMethod->IsConnected(_env, stats, dm, _c1, _c2, dummy, &lpOutput, this->m_positionRes, this->m_orientationRes, false, true);
  //lpPath does not include _c1 and _c2, so adding them manually
  vector<CfgType> cfgs(1, _c1);
  cfgs.insert(cfgs.end(), lpOutput.path.begin(), lpOutput.path.end());
  cfgs.push_back(_c2);
  double d = 0;
  vector<GMSPolyhedron> poly2;
  int robot = _env->GetRobotIndex();
  int bodyCount = _env->GetMultiBody(robot)->GetFreeBodyCount();
  cfgs.begin()->ConfigEnvironment(_env);
  for(int b=0; b<bodyCount; ++b)
    if(this->m_useBbox)
      poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldBoundingBox());
    else
      poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldPolyhedron());
  for(typename vector<CfgType>::const_iterator cit = cfgs.begin(); cit+1 != cfgs.end(); ++cit) {
    vector<GMSPolyhedron> poly1(poly2);
    poly2.clear();
    (cit+1)->ConfigEnvironment(_env);
    for(int b=0; b<bodyCount; ++b)
      if(this->m_useBbox)
        poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldBoundingBox());
      else
        poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldPolyhedron());
    d += SweptDistance(poly1, poly2);
  }
  return d;
}

template<class MPTraits>
double LPSweptDistance<MPTraits>::SweptDistance(const vector<GMSPolyhedron>& _poly1, const vector<GMSPolyhedron>& _poly2) {
  double d = 0;
  int count = 0;
  for(size_t b=0; b<_poly1.size(); ++b)
    for(size_t i=0; i<_poly1[b].m_vertexList.size(); ++i) {
      d += (_poly1[b].m_vertexList[i] - _poly2[b].m_vertexList[i]).magnitude();
      count++;
    }
  return d/(double)count;
}


#endif

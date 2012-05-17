#include "LPSweptDistance.h"
#include "CollisionDetection.h"
#include "LocalPlanners.h"
#include "MPStrategy.h"


LPSweptDistance::LPSweptDistance() : DistanceMetricMethod() {
  m_name = "lp_swept";
}

LPSweptDistance::
LPSweptDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, false) {
  m_name = "lp_swept";
  m_positionRes = _node.numberXMLParameter("pos_res", false, _problem->GetEnvironment()->GetPositionRes(), 0.0, 1000.0, "position resolution");
  m_orientationRes = _node.numberXMLParameter("ori_res", false, _problem->GetEnvironment()->GetOrientationRes(), 0.0, 1000.0, "orientation resolution");
  m_useBbox = _node.boolXMLParameter("use_bbox", false, false, "use bbox instead of robot vertices");
  m_lp = _node.stringXMLParameter("lp_method", true, "", "Local Planner");
  if(_warn)
    _node.warnUnrequestedAttributes();
}


LPSweptDistance::
LPSweptDistance(string _lp, double _posRes, double _oriRes, bool _bbox) : 
  DistanceMetricMethod(), m_lp(_lp), m_positionRes(_posRes), m_orientationRes(_oriRes), m_useBbox(_bbox) {}

LPSweptDistance::~LPSweptDistance() {}

void LPSweptDistance::PrintOptions(ostream& _os) const {
  _os << "    " << this->GetName() << "::  ";
  _os << "\tlp_method = " << m_lp; 
  _os << "\tpositionRes = " << m_positionRes;
  _os << "\torientationRes = " << m_orientationRes;
  _os << "\tuse_bbox = " << m_useBbox;
  _os << endl;
}

double LPSweptDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  StatClass stats;
  shared_ptr<DistanceMetricMethod > dm;
  LPOutput<CfgType, WeightType> lpOutput;
  LocalPlannerPointer lpMethod = GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp);
  CfgType dummy;
  lpMethod->IsConnected(_env, stats, dm, _c1, _c2, dummy, &lpOutput, m_positionRes, m_orientationRes, false, true);
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
    if(m_useBbox)
      poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldBoundingBox());
    else
      poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldPolyhedron());
  for(vector<CfgType>::const_iterator cit = cfgs.begin(); cit+1 != cfgs.end(); ++cit) {
    vector<GMSPolyhedron> poly1(poly2);
    poly2.clear();
    (cit+1)->ConfigEnvironment(_env);
    for(int b=0; b<bodyCount; ++b)
      if(m_useBbox)
        poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldBoundingBox());
      else
        poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldPolyhedron());
    d += SweptDistance(poly1, poly2);
  }
  return d;
}

double LPSweptDistance::SweptDistance(const vector<GMSPolyhedron>& _poly1, const vector<GMSPolyhedron>& _poly2) {
  double d = 0;
  int count = 0;
  for(size_t b=0; b<_poly1.size(); ++b)
    for(size_t i=0; i<_poly1[b].vertexList.size(); ++i) {
      d += (_poly1[b].vertexList[i] - _poly2[b].vertexList[i]).magnitude();
      count++;
    }
  return d/(double)count;
}

#include "BinaryLPSweptDistance.h"
#include "CollisionDetection.h"
#include "LocalPlanners.h"
#include "MPStrategy.h"


BinaryLPSweptDistance::BinaryLPSweptDistance() : DistanceMetricMethod() {
  m_name = "binary_lp_swept";
}

BinaryLPSweptDistance::
BinaryLPSweptDistance(string _lp, double _posRes, double _oriRes, double _tolerance, int _maxAttempts, bool _bbox) : 
DistanceMetricMethod(), m_lp(_lp), m_positionRes(_posRes), m_orientationRes(_oriRes), m_tolerance(_tolerance), m_maxAttempts(_maxAttempts), 
m_distCallsCount(0), m_useBbox(_bbox) {
  m_name = "binary_lp_swept";
}

BinaryLPSweptDistance::
BinaryLPSweptDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, false) {
  m_name = "binary_lp_swept";
  m_positionRes = _node.numberXMLParameter("pos_res", false, _problem->GetEnvironment()->GetPositionRes() * 50, 0.0, 1000.0, "position resolution");
  m_orientationRes = _node.numberXMLParameter("ori_res", false, _problem->GetEnvironment()->GetOrientationRes() * 50, 0.0, 1000.0, "orientation resolution");
  m_tolerance = _node.numberXMLParameter("tolerance", false, 0.01, 0.0, 1000.0, "tolerance");
  m_maxAttempts = _node.numberXMLParameter("max_attempts", false, 10, 1, 100, "maximum depth of lp_swept distance search");
  m_useBbox = _node.boolXMLParameter("use_bbox", false, false, "use bbox instead of robot vertices");
  m_lp = _node.stringXMLParameter("lp_method", true, "", "Local Planner");
  if(_warn)
    _node.warnUnrequestedAttributes();
}

BinaryLPSweptDistance::~BinaryLPSweptDistance() {}

void BinaryLPSweptDistance::PrintOptions(ostream& _os) const {
  _os << "    " << this->GetName() << "::  ";
  _os << "\tlp_method = " << m_lp;
  _os << "\tpositionRes = " << m_positionRes;
  _os << "\torientationRes = " << m_orientationRes;
  _os << "\tuse_bbox = " << m_useBbox;
  _os << "\ttolerance = " << m_tolerance;
  _os << "\tmax_attempts = " << m_maxAttempts;
  _os << endl;
}

double BinaryLPSweptDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double posRes = m_positionRes;
  double oriRes = m_orientationRes;
  double oldDist = DistanceCalc(_env, _c1, _c2, posRes, oriRes);
  double newDist = 0.0;
  int matchCount = 1;
  for (int i = 1; i < m_maxAttempts; i++) {
    posRes /= 2.0;
    oriRes /= 2.0;
    if (posRes < _env->GetPositionRes())   posRes = _env->GetPositionRes();
    if (oriRes < _env->GetOrientationRes()) oriRes = _env->GetOrientationRes();
    newDist = DistanceCalc(_env, _c1, _c2, posRes, oriRes);
    if (newDist - oldDist < m_tolerance) matchCount++;
    else matchCount = 1;
    if (matchCount == 3) break;
    if (posRes == _env->GetPositionRes() && oriRes == _env->GetOrientationRes()) break;
    oldDist = newDist;
  }
  return newDist;
}

double BinaryLPSweptDistance::DistanceCalc(Environment* _env, const Cfg& _c1, const Cfg& _c2, double posRes, double oriRes) {
  StatClass stats;
  shared_ptr<DistanceMetricMethod > dm;
  LPOutput<CfgType, WeightType> lpOutput;
  LocalPlannerPointer lpMethod = GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp);
  CfgType col;
  lpMethod->IsConnected(_env, stats, dm, _c1, _c2, col, &lpOutput, posRes, oriRes, false, true);
  //lpPath does not include _c1 and _c2, so adding them manually
  vector<CfgType> cfgs(1, _c1);
  cfgs.insert(cfgs.end(), lpOutput.path.begin(), lpOutput.path.end());
  cfgs.push_back(_c2);
  double d = 0;
  vector<GMSPolyhedron> poly2;
  int robot = _env->GetRobotIndex();
  int body_count = _env->GetMultiBody(robot)->GetFreeBodyCount();
  cfgs.begin()->ConfigEnvironment(_env);
  for(int b=0; b<body_count; ++b)
    if(m_useBbox)
      poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldBoundingBox());
    else
      poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldPolyhedron());
  for(vector<CfgType>::const_iterator cit = cfgs.begin(); cit+1 != cfgs.end(); ++cit) {
    vector<GMSPolyhedron> poly1(poly2);
    poly2.clear();
    (cit+1)->ConfigEnvironment(_env);
    for(int b=0; b<body_count; ++b)
      if(m_useBbox)
        poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldBoundingBox());
      else
        poly2.push_back(_env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldPolyhedron());
    d += SweptDistance(poly1, poly2);
  }
  m_distCallsCount++;
  return d;
}

double
BinaryLPSweptDistance::
SweptDistance(const vector<GMSPolyhedron>& _poly1, const vector<GMSPolyhedron>& _poly2) {
  double d = 0;
  int count = 0;
  for(size_t b=0; b<_poly1.size(); ++b) {
    for(size_t i=0; i<_poly1[b].vertexList.size(); ++i) {
      d += (_poly1[b].vertexList[i] - _poly2[b].vertexList[i]).magnitude();
      count++;
    }
  }
  return d/(double)count;
}

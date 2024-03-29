#include "LPSweptDistance.h"

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/LocalPlanners/LocalPlannerMethod.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Environment/Environment.h"

#include <ctgmath>

/*------------------------------- Construction -------------------------------*/

LPSweptDistance::
LPSweptDistance(string _lpLabel, double _posRes, double _oriRes, bool _bbox) :
    DistanceMetricMethod(), m_lpLabel(_lpLabel), m_positionRes(_posRes),
    m_orientationRes(_oriRes), m_useBBox(_bbox) {
  this->SetName("LPSwept");
}


LPSweptDistance::
LPSweptDistance(XMLNode& _node) : DistanceMetricMethod(_node),
    m_lpLabel("sl"), m_positionRes(0.1), m_orientationRes(0.1), m_useBBox(false) {
  this->SetName("LPSwept");
  
  m_positionRes = _node.Read("posRes", false, nan(""), 0., 1000.,
      "position resolution");
  m_orientationRes = _node.Read("oriRes", false, nan(""), 0., 1000.,
      "orientation resolution");
  m_useBBox = _node.Read("useBBox", false, false, "use bbox instead of robot "
      "vertices");
  m_lpLabel = _node.Read("lpLabel", true, "sl", "Local Planner");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

void
LPSweptDistance::
Print(std::ostream& _os) const {
  DistanceMetricMethod::Print(_os);
  _os << "\tpositionRes = " << m_positionRes << endl;
  _os << "\torientationRes = " << m_orientationRes << endl;
  _os << "\tuseBBox = " << m_useBBox << endl;
}

/*----------------------------- Distance Interface ---------------------------*/

double
LPSweptDistance::
Distance(const Cfg& _c1, const Cfg& _c2) {
  if(_c1.GetRobot() != _c2.GetRobot())
    throw RunTimeException(WHERE) << "The cfgs cannot reference different robots"
                                  << "('" << _c1.GetRobot()->GetLabel()
                                  << "' and '" << _c2.GetRobot()->GetLabel()
                                  << "').";

  // Determine the resolution to use.
  auto env = this->GetEnvironment();
  const double posRes = std::isnan(m_positionRes) ? env->GetPositionRes()
                                                  : m_positionRes;
  const double oriRes = std::isnan(m_orientationRes) ? env->GetOrientationRes()
                                                     : m_orientationRes;

  // Local plan from _c1 to _c2.
  auto lpMethod = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);
  LPOutput lpOutput;
  Cfg dummy;
  lpMethod->IsConnected(_c1, _c2, dummy, &lpOutput, posRes, oriRes, false, true);

  // lpPath does not include _c1 and _c2, so adding them manually
  /// @todo Verify this, I believe m_path does include _c1 and _c2 for some lps.
  ///       We need to make sure the usage is consistent across all of them.
  std::vector<Cfg> cfgs{_c1};
  cfgs.insert(cfgs.end(), lpOutput.m_path.begin(), lpOutput.m_path.end());
  cfgs.push_back(_c2);

  // init distance to 0
  double d = 0;

  // create vector containing all geometric bodies in _c1 (start config)
  std::vector<GMSPolyhedron> poly1, poly2;
  
  
  auto robot = _c1.GetMultiBody();
  int bodyCount = robot->GetNumBodies();
  cfgs.begin()->ConfigureRobot();
  for(int b=0; b<bodyCount; ++b) {
    if(m_useBBox)
      poly2.push_back(robot->GetBody(b)->GetWorldBoundingBox());
    else
      poly2.push_back(robot->GetBody(b)->GetWorldPolyhedron());
  }
  // iterate through all cfg's in lp path
  for(auto cit = cfgs.begin(); cit + 1 != cfgs.end(); ++cit) {
    // swap vectors (by reference, O(1) operation)
    poly1.std::vector<GMSPolyhedron>::swap(poly2);
    
    // clear poly2
    poly2.clear();
    (cit+1)->ConfigureRobot();
    // populate poly2 with all geometric bodies in the next configuration
    for(int b=0; b<bodyCount; ++b)
      if(m_useBBox)
        poly2.push_back(robot->GetBody(b)->GetWorldBoundingBox());
      else
        poly2.push_back(robot->GetBody(b)->GetWorldPolyhedron());
    // add swept distance between current config and next config
    d += SweptDistance(poly1, poly2);
  }
  return d;
}

/*---------------------------------- Helpers ---------------------------------*/

double
LPSweptDistance::
SweptDistance(const std::vector<GMSPolyhedron>& _poly1,
    const std::vector<GMSPolyhedron>& _poly2) {
  double sum = 0;
  size_t count = 0;
  // tally sum of vertex displacements between two polygon lists
  for(size_t b=0; b<_poly1.size(); ++b)
    for(size_t i=0; i<_poly1[b].GetVertexList().size(); ++i, ++count)
      sum += (_poly1[b].GetVertexList()[i] - _poly2[b].GetVertexList()[i]).norm();
  // divide by total count ?
  return count ? sum / count : 0;
}

/*----------------------------------------------------------------------------*/

#ifndef LP_SWEPT_DISTANCE_H_
#define LP_SWEPT_DISTANCE_H_

#include <ctgmath>

#include "DistanceMetricMethod.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "Geometry/Bodies/FreeBody.h"
#include "PlanningLibrary/LocalPlanners/LPOutput.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LPSweptDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType       CfgType;

    ///@}
    ///@name Construction
    ///@{

    LPSweptDistance(string _lpLabel="", double _positionRes = 0.1,
        double _orientationRes = 0.1, bool _bbox = false);
    LPSweptDistance(XMLNode& _node);
    virtual ~LPSweptDistance() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    double SweptDistance(const vector<GMSPolyhedron>& _poly1,
        const vector<GMSPolyhedron>& _poly2);

    ///@}
    ///@name Internal State
    ///@{

    string m_lpLabel;
    double m_positionRes, m_orientationRes;
    bool m_useBBox;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LPSweptDistance<MPTraits>::
LPSweptDistance(string _lpLabel, double _posRes, double _oriRes, bool _bbox) :
    DistanceMetricMethod<MPTraits>(), m_lpLabel(_lpLabel), m_positionRes(_posRes),
    m_orientationRes(_oriRes), m_useBBox(_bbox) {
  this->SetName("LPSwept");
}


template <typename MPTraits>
LPSweptDistance<MPTraits>::
LPSweptDistance(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node) {
  this->SetName("LPSwept");
  m_positionRes = _node.Read("posRes", false, nan(""), 0., 1000.,
      "position resolution");
  m_orientationRes = _node.Read("oriRes", false, nan(""), 0., 1000.,
      "orientation resolution");
  m_useBBox = _node.Read("useBBox", false, false, "use bbox instead of robot "
      "vertices");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local Planner");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
LPSweptDistance<MPTraits>::
Print(ostream& _os) const {
  DistanceMetricMethod<MPTraits>::Print(_os);
  _os << "\tpositionRes = " << m_positionRes << endl;
  _os << "\torientationRes = " << m_orientationRes << endl;
  _os << "\tuseBBox = " << m_useBBox << endl;
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
LPSweptDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  if (_c1.GetRobotIndex() != _c2.GetRobotIndex()){
    cerr << "LPSweptDistance::Distance error - the cfgs reference different "
         << "multibodies" << endl;
    exit(1);
  }

  if(std::isnan(m_positionRes))
    m_positionRes = this->GetEnvironment()->GetPositionRes();
  if(std::isnan(m_orientationRes))
    m_orientationRes = this->GetEnvironment()->GetOrientationRes();

  Environment* env = this->GetEnvironment();
  StatClass stats;
  auto lpMethod = this->GetLocalPlanner(m_lpLabel);
  LPOutput<MPTraits> lpOutput;
  CfgType dummy;

  lpMethod->IsConnected(_c1, _c2, dummy, &lpOutput, m_positionRes,
      m_orientationRes, false, true);

  //lpPath does not include _c1 and _c2, so adding them manually
  vector<CfgType> cfgs(1, _c1);
  cfgs.insert(cfgs.end(), lpOutput.m_path.begin(), lpOutput.m_path.end());
  cfgs.push_back(_c2);
  double d = 0;
  vector<GMSPolyhedron> poly2;
  int robot = _c1.GetRobotIndex();
  int bodyCount = env->GetRobot(robot)->NumFreeBody();
  cfgs.begin()->ConfigEnvironment();
  for(int b=0; b<bodyCount; ++b)
    if(m_useBBox)
      poly2.push_back(env->GetRobot(robot)->GetFreeBody(b)->
          GetWorldBoundingBox());
    else
      poly2.push_back(env->GetRobot(robot)->GetFreeBody(b)->
          GetWorldPolyhedron());
  for(auto cit = cfgs.begin(); cit + 1 != cfgs.end(); ++cit) {
    vector<GMSPolyhedron> poly1(poly2);
    poly2.clear();
    (cit+1)->ConfigEnvironment();
    for(int b=0; b<bodyCount; ++b)
      if(m_useBBox)
        poly2.push_back(env->GetRobot(robot)->GetFreeBody(b)->
            GetWorldBoundingBox());
      else
        poly2.push_back(env->GetRobot(robot)->GetFreeBody(b)->
            GetWorldPolyhedron());
    d += SweptDistance(poly1, poly2);
  }
  return d;
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
double
LPSweptDistance<MPTraits>::
SweptDistance(const vector<GMSPolyhedron>& _poly1,
    const vector<GMSPolyhedron>& _poly2) {
  double d = 0;
  int count = 0;
  for(size_t b=0; b<_poly1.size(); ++b)
    for(size_t i=0; i<_poly1[b].m_vertexList.size(); ++i) {
      d += (_poly1[b].m_vertexList[i] - _poly2[b].m_vertexList[i]).norm();
      count++;
  }
  return count ? d/(double)count : 0;
}

/*----------------------------------------------------------------------------*/

#endif

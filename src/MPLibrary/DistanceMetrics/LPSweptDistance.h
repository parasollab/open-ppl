#ifndef PMPL_LP_SWEPT_DISTANCE_H_
#define PMPL_LP_SWEPT_DISTANCE_H_

#include "DistanceMetricMethod.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "MPLibrary/LocalPlanners/LocalPlannerMethod.h"

#include <ctgmath>


////////////////////////////////////////////////////////////////////////////////
/// Measures the distance swept by a robot when moving between two
/// configurations.
///
/// @todo Is this the summed vertex displacement?
///
/// @todo Add paper reference.
///
/// @todo This implementation is very poor and COPIES ENTIRE POLYHEDRONS
///       MULTIPLE TIMES (once per intermediate). It will be massively
///       inefficient until we remove the extraneous copies.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LPSweptDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

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

    virtual void Print(std::ostream& _os) const override;

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
Print(std::ostream& _os) const {
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
  auto lpMethod = this->GetLocalPlanner(m_lpLabel);
  LPOutput<MPTraits> lpOutput;
  CfgType dummy;
  lpMethod->IsConnected(_c1, _c2, dummy, &lpOutput, posRes, oriRes, false, true);

  // lpPath does not include _c1 and _c2, so adding them manually
  /// @todo Verify this, I believe m_path does include _c1 and _c2 for some lps.
  ///       We need to make sure the usage is consistent across all of them.
  std::vector<CfgType> cfgs{_c1};
  cfgs.insert(cfgs.end(), lpOutput.m_path.begin(), lpOutput.m_path.end());
  cfgs.push_back(_c2);

  double d = 0;

  std::vector<GMSPolyhedron> poly2;
  auto robot = _c1.GetMultiBody();
  int bodyCount = robot->GetNumBodies();
  cfgs.begin()->ConfigureRobot();
  for(int b=0; b<bodyCount; ++b) {
    if(m_useBBox)
      poly2.push_back(robot->GetBody(b)->GetWorldBoundingBox());
    else
      poly2.push_back(robot->GetBody(b)->GetWorldPolyhedron());
  }
  for(auto cit = cfgs.begin(); cit + 1 != cfgs.end(); ++cit) {
    vector<GMSPolyhedron> poly1(poly2);
    poly2.clear();
    (cit+1)->ConfigureRobot();
    for(int b=0; b<bodyCount; ++b)
      if(m_useBBox)
        poly2.push_back(robot->GetBody(b)->GetWorldBoundingBox());
      else
        poly2.push_back(robot->GetBody(b)->GetWorldPolyhedron());
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
  double sum = 0;
  size_t count = 0;

  for(size_t b=0; b<_poly1.size(); ++b)
    for(size_t i=0; i<_poly1[b].GetVertexList().size(); ++i, ++count)
      sum += (_poly1[b].GetVertexList()[i] - _poly2[b].GetVertexList()[i]).norm();

  return count ? sum / count : 0;
}

/*----------------------------------------------------------------------------*/

#endif

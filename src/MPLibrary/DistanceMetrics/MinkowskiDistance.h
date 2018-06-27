#ifndef MINKOWSKI_DISTANCE_H_
#define MINKOWSKI_DISTANCE_H_

#include "DistanceMetricMethod.h"

#include "MPProblem/Environment/Environment.h"
#include "MPProblem/IsClosedChain.h"

#include <boost/utility/enable_if.hpp>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MinkowskiDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType                         CfgType;
    typedef typename MPTraits::GroupCfgType                    GroupCfgType;
    typedef typename DistanceMetricMethod<MPTraits>::Formation Formation;

    ///@}
    ///@name Construction
    ///@{

    MinkowskiDistance(double _r1 = 3, double _r2 = 3, double _r3 = 1. / 3,
        bool _normalize = false);

    MinkowskiDistance(XMLNode& _node);

    virtual ~MinkowskiDistance() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    virtual void ScaleCfg(double _length, CfgType& _c,
                          const CfgType& _o) override;

    static double PositionDistance(Environment *_env, const CfgType& _c,
                                   double _r1, double _r3, bool _normalize);


    /// GroupCfg Overloads:
    virtual double Distance(const GroupCfgType& _c1,
                            const GroupCfgType& _c2) override;

    virtual void ScaleCfg(double _length, GroupCfgType& _c,
                          const GroupCfgType& _o) override;

    static double PositionDistance(Environment *_env, const GroupCfgType& _c,
                                   double _r1, double _r3, bool _normalize);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    //default implementation
    template <typename EnableCfg>
    EnableCfg DifferenceCfg(const EnableCfg& _c1, const EnableCfg& _c2,
        typename boost::disable_if<IsClosedChain<EnableCfg> >::type* _dummy = 0) {
      return _c1 - _c2;
    }

    //reachable distance implementation
    template <typename EnableCfg>
    EnableCfg DifferenceCfg(const EnableCfg& _c1, const EnableCfg& _c2,
        typename boost::enable_if<IsClosedChain<EnableCfg> >::type* _dummy = 0) {
      cerr << "Error::DistanceMetrics for ReachableDistance disabled." << endl;
      exit(1);
    }

    double PositionDistance(const CfgType& _c);
    double OrientationDistance(const CfgType& _c);

    ///@}
    ///@name Internal State
    ///@{

    double m_r1{3};          ///< For position part.
    double m_r2{3};          ///< For rotation part.
    double m_r3{1. / 3.};    ///< For calculating root.
    bool m_normalize{false}; ///< Normalize distance w.r.t. environment?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
MinkowskiDistance<MPTraits>::
MinkowskiDistance(double _r1, double _r2, double _r3, bool _normalize) :
    DistanceMetricMethod<MPTraits>(), m_r1(_r1), m_r2(_r2), m_r3(_r3),
    m_normalize(_normalize) {
  this->SetName("Minkowski");
}


template <typename MPTraits>
MinkowskiDistance<MPTraits>::
MinkowskiDistance(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
    m_r1(3), m_r2(3), m_r3(1./3.), m_normalize(false) {
  this->SetName("Minkowski");

  m_r1 = _node.Read("r1", false, 3., 0., MAX_DBL, "r1");
  m_r2 = _node.Read("r2", false, 3., 0., MAX_DBL, "r2");
  m_r3 = _node.Read("r3", false, 1. / 3., 0., MAX_DBL, "r3");
  m_normalize = _node.Read("normalize", false, false, "flag if position dof "
      "should be normalized by environment diagonal");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
MinkowskiDistance<MPTraits>::
Print(ostream& _os) const {
  DistanceMetricMethod<MPTraits>::Print(_os);
  _os << "\tr1 = " << m_r1 << endl
      << "\tr2 = " << m_r2 << endl
      << "\tr3 = " << m_r3 << endl
      << "\tnormalize = " << m_normalize << endl;
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  CfgType diff = DifferenceCfg(_c1, _c2);
  double pos = PositionDistance(diff);
  double orient = OrientationDistance(diff);
  return pow(pos + orient, m_r3);
}


template <typename MPTraits>
void
MinkowskiDistance<MPTraits>::
ScaleCfg(double _length, CfgType& _c, const CfgType& _o) {
  double originalLength = this->Distance(_o, _c);
  double diff = _length - originalLength;
  do {
    _c = (_c - _o) * (_length / originalLength) + _o;
    originalLength = this->Distance(_o, _c);
    diff = _length - originalLength;
  } while((diff > 0.1) || (diff < -0.1));
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
PositionDistance(const CfgType& _c) {
  return PositionDistance(this->GetEnvironment(), _c, m_r1, m_r3, m_normalize);
}


template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
PositionDistance(Environment *_env,
    const CfgType& _c, double _r1, double _r3, bool _normalize) {
  const vector<double> p = _c.GetPosition();
  double distance = 0;

  if(_normalize) {
    const double diagonal = _env->GetBoundary()->GetMaxDist(_r1, _r3);
    for(size_t i = 0; i < p.size(); ++i)
      distance += pow(fabs(p[i]) / diagonal, _r1);
  }
  else
    for(size_t i = 0; i < p.size(); ++i)
      distance += pow(fabs(p[i]), _r1);

  return distance;
}


template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
OrientationDistance(const CfgType& _c) {
  const vector<double> o = _c.GetOrientation();
  double distance = 0;
  for(size_t i = 0; i < o.size(); ++i)
    distance += pow(fabs(o[i]), m_r2);
  return distance;
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
Distance(const GroupCfgType& _c1, const GroupCfgType& _c2) {
  if(this->m_activeRobots.empty())
    throw RunTimeException(WHERE, "Used group version but didn't provide active"
                                  " robots!");

  const double closeToZero = std::numeric_limits<double>::epsilon();
  const bool isMultiPartAllowed = this->m_activeRobots.size() > 1;

  Formation robotsMoved;
  double compositeDistance = 0.0;

  // Loop through ALL robots, to ensure that we have a valid pair (robots can
  // only "move" if they are in the activeRobot list, or if they move and no
  // other robot moves).
  for(size_t i = 0; i < _c1.GetNumRobots(); ++i) {
    // Get the robot cfg and use the normal distance metric, but accumulated.
    const double cfgDist = Distance(_c1.GetRobotCfg(i), _c2.GetRobotCfg(i));

    // Ensure that the robots "moved" between the two cfgs are allowed:
    if (cfgDist > closeToZero) {
      compositeDistance += cfgDist; // Accumulate individual distances.
      robotsMoved.push_back(i);

      // We can quit right away if more than one part is moved and we're not
      // allowing for a multi-part subassembly.
      if(!isMultiPartAllowed && robotsMoved.size() > 1)
        return std::numeric_limits<double>::infinity(); // Invalid neighbors.
    }
  }

  if(robotsMoved.empty())
    return 0;

  // m_activeRobots is sorted ascending when it's set (see SetActiveRobots) and
  // we know that robotsMoved will be ascending due to the loop above.
  if(robotsMoved != this->m_activeRobots)
    return std::numeric_limits<double>::infinity(); // Invalid neighbors.

  return compositeDistance;
}

template <typename MPTraits>
void
MinkowskiDistance<MPTraits>::
ScaleCfg(double _length, GroupCfgType& _c, const GroupCfgType& _o) {
  throw RunTimeException(WHERE, "Not implemented!");
}

template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
PositionDistance(Environment *_env, const GroupCfgType& _c,
                 double _r1, double _r3, bool _normalize) {
  throw RunTimeException(WHERE, "Not implemented!");
}

#endif

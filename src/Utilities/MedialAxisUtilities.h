#ifndef MEDIAL_AXIS_UTILITY_H_
#define MEDIAL_AXIS_UTILITY_H_

#include "MPUtils.h"

#include <ctgmath>
#include <deque>

#include "MetricUtils.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/FixedBody.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// Return structure for clearance information.
////////////////////////////////////////////////////////////////////////////////
struct ClearanceStats {
  double m_avg{0};
  double m_min{0};
  double m_max{0};
  double m_var{0};
  double m_pathLength{0};
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// A ray in configuration space for approximate methods.
////////////////////////////////////////////////////////////////////////////////
template<class CfgType>
struct Ray {
  CfgType m_incr;
  CfgType m_tick;

  Ray(const CfgType& _i, const CfgType& _t) : m_incr(_i), m_tick(_t) {}
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// Computes clearance and penetration of configurations with respect to
/// obstacles.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ClearanceUtility : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType        CfgType;
    typedef typename MPTraits::WeightType     WeightType;
    typedef typename MPTraits::RoadmapType    RoadmapType;
    typedef typename RoadmapType::GraphType   GraphType;
    typedef typename RoadmapType::VID         VID;

    ///@}
    ///@name Construction
    ///@{

    ClearanceUtility(
        string _vcLabel = "", string _dmLabel = "",
        bool _exactClearance = false, bool _exactPenetration = false,
        size_t _clearanceRays = 10, size_t _penetrationRays = 10,
        double _approxStepSize = MAX_DBL, double _approxResolution = MAX_DBL,
        bool _useBBX = true, bool _positional = true, bool _debug = false);

    ClearanceUtility(XMLNode& _node);

    virtual ~ClearanceUtility() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;
    virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{

    const string& GetDistanceMetricLabel() const {return m_dmLabel;}
    const string& GetValidityCheckerLabel() const {return m_vcLabel;}
    void SetValidityCheckerLabel(const string& _s) {m_vcLabel = _s;}
    bool GetExactClearance() const {return m_exactClearance;}

    ///@}
    ///@name Clearance Functions
    ///@{

    /// Calculate clearance information for the medial axis computation.
    bool CollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
        const Boundary* const _b, CDInfo& _cdInfo);

    /// Calculate clearance information exactly by taking the validity
    ///        checker results against obstacles to the bounding box.
    bool ExactCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
        const Boundary* const _b, CDInfo& _cdInfo);

    /// Calculate the approximate clearance using a series of rays. The
    ///        specified number of rays are pushed outward until they change in
    ///        validity. The shortest ray is then considered the best candidate.
    bool ApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
        const Boundary* const _b, CDInfo& _cdInfo);

    /// Calculate roadmap clearance statistics including averages, mins,
    ///        and maxes for clearance across roadmaps, paths, and edges.
    ClearanceStats RoadmapClearance();

    ClearanceStats PathClearance(vector<VID>& _path);
    ClearanceStats PathClearance(vector<Cfg>& _path);

    double MinEdgeClearance(const CfgType& _c1, const CfgType& _c2,
        const WeightType& _weight);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    string m_vcLabel{"cd4"};        ///< Validity checker method label.
    string m_dmLabel{"euclidean"};  ///< Distance metric method label.

    bool m_useBBX{true};            ///< Use bounding box as obstacle?
    bool m_positional{true};        ///< Use only positional dofs?

    bool m_exactClearance{false};   ///< Use exact clearance calculations?
    bool m_exactPenetration{false}; ///< Use exact penetration calculations?

    size_t m_clearanceRays{10};  ///< Number of rays for approximate clearance.
    size_t m_penetrationRays{10};///< Number of rays for approximate penetration.

    /// Step size for approximate clearance and penetration as a multiple
    /// of env res.
    double m_approxStepSize{MAX_DBL};

    /// Resolution for approximate clearance and penetration as a
    /// multiple of env res.
    double m_approxResolution{MAX_DBL};

    ///@}
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// Tool for pushing configurations to the medial axis.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MedialAxisUtility : public ClearanceUtility<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    MedialAxisUtility(
        string _vcLabel = "", string _dmLabel = "",
        bool _exactClearance = false, bool _exactPenetration = false,
        size_t _clearanceRays = 10, size_t _penetrationRays = 10,
        bool _useBBX = true, bool _positional = true, bool _debug = false,
        double _epsilon = 0.1, size_t _historyLength = 5);

    MedialAxisUtility(XMLNode& _node);

    virtual ~MedialAxisUtility() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Property Accessors
    ///@{

    double GetEpsilon() const {return m_epsilon;}

    ///@}
    ///@name Medial Axis Functions
    ///@{

    /// Push a configuration to the medial axis.
    bool PushToMedialAxis(CfgType& _cfg, const Boundary* const _b);

    /// Push a configuration that is known to be inside an obstacle
    /// towards the free-space medial axis. It will be pushed until it is
    /// outside the obstacle.
    bool PushFromInsideObstacle(CfgType& _cfg, const Boundary* const _b);

    /// Push a configuration to the medial axis by stepping away from the
    /// nearest obstacle at the resolution until the witness
    /// configuration changes.
    bool PushCfgToMedialAxis(CfgType& _cfg, const Boundary* const _b);

    ///@}

  private:

    ///@name Helpers
    ///@{

    bool FindInitialDirection(CfgType& _cfg, const Boundary* const _b,
        double _posRes, CfgType& _transCfg, CDInfo& _prevInfo);

    bool FindMedialAxisBorderExact(
        const CfgType& _cfg, const Boundary* const _b,
        CfgType& _transCfg, CDInfo& _prevInfo,
        CfgType& _startCfg, CfgType& _endingCfg,
        double& _upperBound, double& _lowerBound, double& _stepSize);

    bool FindMedialAxisBorderApprox(
        const CfgType& _cfg, const Boundary* const _b,
        CfgType& _transCfg, CDInfo& _prevInfo,
        CfgType& _startCfg, CfgType& _endingCfg,
        double& _upperBound, double& _lowerBound, double& _stepSize);

    CfgType BinarySearchForPeaks(CfgType& _startCfg, CfgType& _midMCfg,
        CfgType& _endingCfg, double _lowerBound, double _upperBound,
        CfgType& _transCfg, CfgType& _cfg, const Boundary* const _b,
        vector<double>& _dists);

    ///@}
    ///@name Internal State
    ///@{

    double m_epsilon{.1};
    size_t m_historyLength{5};

    ///@}
};

/*---------------------------- Clearance Utility -----------------------------*/

template<class MPTraits>
ClearanceUtility<MPTraits>::
ClearanceUtility(
    string _vcLabel, string _dmLabel,
    bool _exactClearance, bool _exactPenetration,
    size_t _clearanceRays, size_t _penetrationRays,
    double _approxStepSize, double _approxResolution,
    bool _useBBX, bool _positional, bool _debug):
    m_vcLabel(_vcLabel), m_dmLabel(_dmLabel),
    m_useBBX(_useBBX), m_positional(_positional),
    m_exactClearance(_exactClearance), m_exactPenetration(_exactPenetration),
    m_clearanceRays(_clearanceRays), m_penetrationRays(_penetrationRays),
    m_approxStepSize(_approxStepSize), m_approxResolution(_approxResolution) {
  this->SetName("ClearanceUtility");
  this->m_debug = _debug;
}


template<class MPTraits>
ClearanceUtility<MPTraits>::
ClearanceUtility(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  this->SetName("ClearanceUtility");

  m_vcLabel = _node.Read("vcLabel", false, m_vcLabel, "Validity Test Method");
  m_dmLabel = _node.Read("dmLabel", false, m_dmLabel, "Distance metric");

  string clearanceType = _node.Read("clearanceType", true, "",
      "Clearance Computation (exact or approx)");
  m_exactClearance = clearanceType.compare("exact") == 0;

  string penetrationType = _node.Read("penetrationType", true, "",
      "Penetration Computation (exact or approx)");
  m_exactPenetration = penetrationType.compare("exact") == 0;

  m_approxStepSize = _node.Read("stepSize", false, MAX_DBL, 0.,
      MAX_DBL, "Step size for initial approximate computations as multiple of "
      "environment resolution");
  m_approxResolution = _node.Read("resolution", false, MAX_DBL, 0.,
      MAX_DBL, "Resolution for final approximate computations as multiple of "
      "environment resolution");

  // If using approximate calculations, require number of rays to be defined.
  m_clearanceRays = _node.Read("clearanceRays", !m_exactClearance,
      m_clearanceRays, size_t(1), size_t(1000), "Number of Clearance Rays");
  m_penetrationRays = _node.Read("penetrationRays", !m_exactPenetration,
      m_penetrationRays, size_t(1), size_t(1000), "Number of Penetration Rays");

  m_useBBX = _node.Read("useBBX", false, m_useBBX, "Use the Bounding Box as an "
      "obstacle");
  m_positional = _node.Read("positional", false, m_positional, "Use only "
      "positional DOFs");
}


template<class MPTraits>
void
ClearanceUtility<MPTraits>::
Print(ostream& _os) const {
  MPBaseObject<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl
      << "\tdmLabel = " << m_dmLabel << endl
      << "\tpositional = " << m_positional << endl
      << "\tuseBBX = " << m_useBBX << endl
      << "\tclearance = " << (m_exactClearance ? "exact" :
         "approx, " + to_string(m_clearanceRays) + " rays") << endl
      << "\tpenetration = " << (m_exactPenetration ? "exact" :
         "approx, " + to_string(m_penetrationRays) + " rays") << endl;
  if(!m_exactClearance || !m_exactPenetration)
    _os << "\tapproxStepSize = " << m_approxStepSize << endl
        << "\tapproxResolution = " << m_approxResolution << endl;
}


template <class MPTraits>
void
ClearanceUtility<MPTraits>::
Initialize() {
  double minStepSize = 1. / this->GetEnvironment()->GetBoundary()->GetMaxDist();
  if(m_approxStepSize == MAX_DBL)
    m_approxStepSize = minStepSize;
  if(m_approxResolution == MAX_DBL)
    m_approxResolution = minStepSize;
}


template<class MPTraits>
bool
ClearanceUtility<MPTraits>::
CollisionInfo(CfgType& _cfg, CfgType& _clrCfg, const Boundary* const _b,
    CDInfo& _cdInfo) {
  if(m_exactClearance)
    return ExactCollisionInfo(_cfg, _clrCfg, _b, _cdInfo);
  else
    return ApproxCollisionInfo(_cfg, _clrCfg, _b, _cdInfo);
}


template<class MPTraits>
bool
ClearanceUtility<MPTraits>::
ExactCollisionInfo(CfgType& _cfg, CfgType& _clrCfg, const Boundary* const _b,
    CDInfo& _cdInfo) {
  if(_cfg.m_witnessCfg.get() != 0) {
    _cdInfo = _cfg.m_clearanceInfo;
    _clrCfg = CfgType(*_cfg.m_witnessCfg);
    return true;
  }

  if(this->m_debug) {
    VDClearAll();
    VDAddTempCfg(_cfg, false);
  }

  // ClearanceUtility variables
  Environment* env = this->GetEnvironment();

  // Setup Validity Checker
  string callee = this->GetNameAndLabel() + "::ExactCollisionInfo";
  auto vcm = this->GetValidityChecker(m_vcLabel);
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  // If not in BBX or valid, return false (IsValid gets _cdInfo)
  bool initInside = vcm->IsInsideObstacle(_cfg);         // Initially Inside Obst
  bool initValidity = vcm->IsValid(_cfg, _cdInfo, callee); // Initial Validity
  initValidity = initValidity && !initInside;

  if(!initValidity) {
    _cdInfo.m_minDist = -_cdInfo.m_minDist;
  }

  // If not using the bbx, done
  if(m_useBBX) {
    // CfgType is now know as good, get BBX and ROBOT info
    auto multiBody = _cfg.GetMultiBody();

    // Find closest point between robot and bbx, set if less than min dist
    // from obstacles
    for(size_t m = 0; m < multiBody->NumFreeBody(); ++m) {
      const GMSPolyhedron& poly = multiBody->GetFreeBody(m)->
          GetWorldPolyhedron();
      for(size_t j = 0; j < poly.m_vertexList.size(); ++j) {
        double clr = _b->GetClearance(poly.m_vertexList[j]);
        if(clr < _cdInfo.m_minDist) {
          _cdInfo.m_robotPoint = poly.m_vertexList[j];
          _cdInfo.m_objectPoint = _b->GetClearancePoint(poly.m_vertexList[j]);
          _cdInfo.m_minDist = clr;
          _cdInfo.m_nearestObstIndex = -1;
        }
      }
    }
  }

  Vector3d clrDir = _cdInfo.m_objectPoint - _cdInfo.m_robotPoint;
  auto robot = this->GetTask()->GetRobot();
  CfgType stepDir(robot);
  double factor = 0;
  for(size_t i = 0; i < _clrCfg.DOF(); ++i) {
    if(i < _clrCfg.PosDOF()) {
      _clrCfg[i] = _cfg[i] + clrDir[i];
      stepDir[i] = clrDir[i];
      factor += clrDir[i] * clrDir[i];
    }
    else {
      _clrCfg[i] = _cfg[i];
      stepDir[i] = 0.0;
    }
    stepDir *= env->GetPositionRes() / sqrt(factor);
  }

  if(_clrCfg == _cfg)
    return false;

  if(!initValidity) {
    CfgType incr(robot), tmpCfg = _clrCfg;
    int nTicks;
    incr.FindIncrement(_cfg, _clrCfg, &nTicks,
        env->GetPositionRes(), env->GetOrientationRes());

    bool tmpValidity = vcm->IsValid(tmpCfg, callee);
    tmpValidity = tmpValidity && !vcm->IsInsideObstacle(tmpCfg);
    while(!tmpValidity) {
      tmpCfg += incr;

      tmpValidity = vcm->IsValid(tmpCfg, callee);
      tmpValidity = tmpValidity && !vcm->IsInsideObstacle(tmpCfg);
      bool inBBX = env->InBounds(tmpCfg, _b);
      if(!inBBX) {
        if(this->m_debug)
          cout << "ERROR: Fell out of BBX, error out... " << endl;
        return false;
      }
    }
    _clrCfg = tmpCfg;
  }

  if(this->m_debug)
    VDAddTempCfg(_clrCfg, true);

  _cfg.m_clearanceInfo = _cdInfo;
  _cfg.m_witnessCfg = shared_ptr<Cfg>(new CfgType(_clrCfg));

  return true;
}


template<class MPTraits>
bool
ClearanceUtility<MPTraits>::
ApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
    const Boundary* const _b, CDInfo& _cdInfo) {
  // Check computation cache
  if(_cfg.m_witnessCfg.get() != 0) {
    _cdInfo = _cfg.m_clearanceInfo;
    _clrCfg = CfgType(*_cfg.m_witnessCfg);
    return true;
  }

  // If in BBX, check validity to get _cdInfo, return false if not valid
  Environment* env = this->GetEnvironment();
  if(!env->InBounds(_cfg, _b))
    return false;

  // Initialization
  string callee = this->GetNameAndLabel() + "::ApproxCollisionInfo";
  auto dm  = this->GetDistanceMetric(m_dmLabel);
  auto vcm = this->GetValidityChecker(m_vcLabel);
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  // Compute initial validity state
  bool initInside = vcm->IsInsideObstacle(_cfg);
  bool initValidity = vcm->IsValid(_cfg, _cdInfo, callee);
  if(this->m_debug)
    cout << "\nDEBUG:: initInside = " << initInside
      << "\tinitValidity = " << initValidity;
  initValidity = initValidity && !initInside;
  if(this->m_debug)
    cout << "\tinitValidity -> " << initValidity << endl;

  // Setup random rays
  size_t numRays;
  if(initValidity)
    numRays = m_clearanceRays;
  else
    numRays = m_penetrationRays;
  if(this->m_debug)
    cout << "DEBUG:: numRays = " << numRays << endl;
  vector<Ray<CfgType>> rays;
  double posRes = env->GetPositionRes();
  for(size_t i = 0; i < numRays; ++i) {
    CfgType tmpDirection(this->GetTask()->GetRobot());
    tmpDirection.GetRandomRay(m_approxStepSize * env->GetPositionRes(),
        dm, false);
    if(this->m_debug)
      cout << "DEBUG:: tmpDirection " << i << " is " << tmpDirection << endl;
    if(m_positional) { // Use only positional dofs
      double factor = 0.0;
      for(size_t j = 0; j<tmpDirection.DOF(); j++) {
        if(j < tmpDirection.PosDOF())
          factor += tmpDirection[j] * tmpDirection[j];
        else
          tmpDirection[j] = 0.0;
      }
      tmpDirection *= posRes / sqrt(factor);
    }
    if(this->m_debug) {
      cout << "DEBUG:: tmpDirection " << i << " is " << tmpDirection << endl;
      cout << "DEBUG:: \tdistance(_cfg, _cfg+tmpDirection) = "
        << dm->Distance(_cfg, _cfg + tmpDirection) << endl;
    }
    rays.push_back(Ray<CfgType>(tmpDirection, _cfg));
  }
  if(this->m_debug) {
    cout << "DEBUG:: rays initialized\n";
    cout << "DEBUG:: \ttick are:\n\t\t";
    for(auto&  ray : rays)
      cout << ray.m_tick << "\n\t\t";
    cout << endl;
    cout << "DEBUG:: \tincr are:\n\t\t";
    for(auto&  ray : rays)
      cout << ray.m_incr << "\n\t\t";
    cout << endl;
  }

  // Step out along each direction to determine the candidates
  vector<pair<size_t, CfgType>> candidates;
  bool stateChangedFlag = false;
  size_t iterations = 0, maxIterations = 100; //TODO: Smarter maxIter number

  if(this->m_debug)
    cout << "DEBUG:: stepping out along each direction to find candidates";

  while(!stateChangedFlag && iterations++ < maxIterations) {
    if(this->m_debug)
      cout << "\n\t" << iterations;
    for(auto rit = rays.begin(); rit != rays.end(); ++rit) {
      //step out
      rit->m_tick += rit->m_incr;

      //determine new state
      CDInfo tmpInfo;
      bool currInside = vcm->IsInsideObstacle(rit->m_tick);
      bool currValidity = vcm->IsValid(rit->m_tick, tmpInfo, callee);
      currValidity = currValidity && !currInside;
      if(m_useBBX)
        currValidity = (currValidity && env->InBounds(rit->m_tick, _b));

      if(this->m_debug)
        cout << " (currValidity for direction " << distance(rays.begin(), rit)
             << " = " << currValidity << ")";

      //if state has changed, add to candidate list
      if(currValidity != initValidity) {
        stateChangedFlag = true;
        CfgType candidate = rit->m_tick - rit->m_incr;
        candidates.push_back(make_pair(distance(rays.begin(), rit), candidate));
      }
    }
  }
  if(this->m_debug) {
    cout << "\nDEBUG:: done stepping out along rays\n"
         << "   found " << candidates.size() << " candidates:\n";
    for(auto& cand : candidates) {
      cout << "\t" << cand.first << ": " << cand.second;

      CDInfo tmpInfo;
      bool currInside = vcm->IsInsideObstacle(cand.second);
      bool currValidity = vcm->IsValid(cand.second, tmpInfo, callee);
      currValidity = currValidity && !currInside;
      if(m_useBBX)
        currValidity = (currValidity && env->InBounds(cand.second, _b));
      cout << " (currValidity = " << currValidity << ")" << endl;
    }
  }

  // Remove spurious candidates
  if(this->m_debug)
    cout << "DEBUG:: checking for spurious candidates and removing them\n";

  vector<bool> remove;
  for(auto& cand : candidates) {
    if(this->m_debug)
      cout << "\t" << cand.first;
    CDInfo tmpInfo;

    CfgType lowCfg = rays[cand.first].m_incr * 0.0 + cand.second;
    bool lowInside = vcm->IsInsideObstacle(lowCfg);
    bool lowValidity = vcm->IsValid(lowCfg, tmpInfo, callee);
    lowValidity = lowValidity && !lowInside;
    if(m_useBBX)
      lowValidity = (lowValidity && env->InBounds(lowCfg, _b));
    if(this->m_debug)
      cout << " (lowValidity = " << lowValidity << ")" << endl;

    CfgType highCfg = rays[cand.first].m_incr * 1.0 + cand.second;
    bool highInside = vcm->IsInsideObstacle(highCfg);
    bool highValidity = vcm->IsValid(highCfg, tmpInfo, callee);
    highValidity = highValidity && !highInside;
    if(m_useBBX)
      highValidity = (highValidity && env->InBounds(highCfg, _b));
    if(this->m_debug)
      cout << " (highValidity = " << highValidity << ")" << endl;

    remove.push_back(lowValidity == highValidity);
  }
  if(this->m_debug) {
    cout << "   remove = ";
    copy(remove.begin(), remove.end(), ostream_iterator<bool>(cout, " "));
    cout << endl;
  }
  size_t offset = 0;
  for(size_t i = 0; i < remove.size(); ++i)
    if(remove[i]) {
      candidates.erase(candidates.begin() + (i - offset));
      offset++;
    }

  if(this->m_debug) {
    cout << "   found " << candidates.size() << " candidates:\n";
    for(auto& cand : candidates) {
      cout << "\t" << cand.first << ": " << cand.second;

      CDInfo tmpInfo;
      bool currInside = vcm->IsInsideObstacle(cand.second);
      bool currValidity = vcm->IsValid(cand.second, tmpInfo, callee);
      currValidity = currValidity && !currInside;
      if(m_useBBX)
        currValidity = (currValidity && env->InBounds(cand.second, _b));
      cout << " (currValidity = " << currValidity << ")";
      cout << endl;
    }
  }

  if(candidates.size() == 0)
    return false;

  // Binary search on candidates to find the best result at specified resolution
  if(this->m_debug)
    cout << "DEBUG:: binary searching on candidates to find best result\n";

  double low = 0.0, mid = 0.5, high = 1.0;
  while((candidates.size() > 1 && low != high) ||
        (candidates.size() == 1 &&
         dm->Distance(
           rays[candidates.front().first].m_incr * low +
           candidates.front().second,
           rays[candidates.front().first].m_incr * high +
           candidates.front().second) >
         m_approxResolution * env->GetPositionRes())) {
    mid = (low + high) / 2.;
    vector<bool> remove;
    bool foundStateChange = false;

    if(this->m_debug)
      cout << "   mid = " << mid << "\t";

    for(auto&  cand : candidates) {
      CfgType middleCfg = rays[cand.first].m_incr * mid + cand.second;
      CDInfo tmpInfo;
      bool midInside = vcm->IsInsideObstacle(middleCfg);
      bool midValidity = vcm->IsValid(middleCfg, tmpInfo, callee);
      midValidity = midValidity && !midInside;
      if(m_useBBX)
        midValidity = (midValidity && env->InBounds(middleCfg, _b));

      if(this->m_debug)
        cout << "(midValidity for direction " << cand.first
          << " = " << midValidity
          << ", distance = " << dm->Distance(
              rays[cand.first].m_incr * low + cand.second, middleCfg) << ") ";

      remove.push_back(midValidity == initValidity);
      if(midValidity != initValidity) // If Validity State has changed
        foundStateChange = true;
    }
    if(this->m_debug)
      cout << endl;

    // If at least one good candidate is found, remove all known bad ones
    if(foundStateChange) {
      if(this->m_debug) {
        cout << "\tfound state change: remove = ";
        copy(remove.begin(), remove.end(), ostream_iterator<bool>(cout, " "));
        cout << endl;
      }
      size_t offset = 0;
      for(size_t i = 0; i < remove.size(); ++i) {
        if(remove[i]) {
          candidates.erase(candidates.begin() + (i - offset));
          offset++;
        }
      }
      high = mid;
    }
    else {
      low = mid;
    }
  }
  if(this->m_debug)
    cout << "DEBUG:: done with binary search\n"
         << "   resulting candidate: " << candidates[0].first
         << ": " << candidates[0].second << endl
         << "   low = " << low << "\thigh = " << high << endl;

  // Finalizing search, keep mid as the low cfg if computing clearance and mid
  // as the high cfg if computing penetration
  // Computing clerarance, so low is needed as long as its not the initial cfg
  if(initValidity) {
    while(low == 0.0 && high != low) {
      mid = (low + high) / 2.;
      CfgType middleCfg = rays[candidates[0].first].m_incr * mid +
          candidates[0].second;

      CDInfo tmpInfo;
      bool midInside = vcm->IsInsideObstacle(middleCfg);
      bool midValidity = vcm->IsValid(middleCfg, tmpInfo, callee);
      midValidity = midValidity && !midInside;
      if(m_useBBX)
        midValidity = (midValidity && env->InBounds(middleCfg, _b));
      // If Validity State has changed
      if(midValidity != initValidity)
        high = mid;
      else
        low = mid;
    }
    mid = low;
  }
  // Computing penetration, so high is needed and will never be the initial cfg
  else
    mid = high;

  // Set return info
  if(this->m_debug)
    cout << "DEBUG:: setting info, returning\n";

  _clrCfg = rays[candidates[0].first].m_incr * mid + candidates[0].second;
  _cdInfo.m_minDist = (initValidity ? 1.0 : -1.0) * dm->Distance(_clrCfg, _cfg);
  if(_clrCfg == _cfg) //shouldn't happen, but should return an error
    return false;
  if(env->InBounds(_clrCfg, _b)) {
    _cfg.m_clearanceInfo = _cdInfo;
    _cfg.m_witnessCfg = shared_ptr<Cfg>(new CfgType(_clrCfg));
    return true;
  }
  else
    return false;
}


template<class MPTraits>
ClearanceStats
ClearanceUtility<MPTraits>::
RoadmapClearance() {
  GraphType* g = this->GetRoadmap()->GetGraph();

  //TODO handle case of singleton nodes to be part of roadmap clearance
  //computation.
  if(g->get_num_edges() == 0)
    return ClearanceStats();

  vector<double> clearanceVec;

  //loop over graph edges and calculate clearance
  for(auto it = g->edges_begin(); it != g->edges_end(); ++it) {
    double currentClearance =
      MinEdgeClearance(g->GetVertex((*it).source()),
          g->GetVertex((*it).target()), (*it).property());
    //Save this value for variance computation later
    clearanceVec.push_back(currentClearance);
  }

  //min, max, avg, variance of graph edge variances
  ClearanceStats stats;
  stats.m_min = *min_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_max = *max_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_avg = accumulate(clearanceVec.begin(), clearanceVec.end(), 0.0) /
    clearanceVec.size();
  double varSum = 0;
  for(auto&  i : clearanceVec)
    varSum += sqr(i - stats.m_avg);
  stats.m_var = varSum / clearanceVec.size();

  return stats;
}


template<class MPTraits>
ClearanceStats
ClearanceUtility<MPTraits>::
PathClearance(vector<VID>& _path) {
  if(_path.empty())
    return ClearanceStats();

  GraphType* g = this->GetRoadmap()->GetGraph();
  auto dm = this->GetDistanceMetric(m_dmLabel);

  typedef typename GraphType::EI EI;
  typedef typename GraphType::VI VI;
  typedef typename GraphType::EID EID;

  vector<double> clearanceVec;
  double pathLength = 0;

  for(auto vit = _path.begin(); (vit+1) != _path.end(); ++vit) {
    EI ei;
    VI vi;
    EID ed(*vit, *(vit + 1));
    g->find_edge(ed, vi, ei);
    CfgType& s = g->GetVertex((*ei).source()), t = g->GetVertex((*ei).target());
    pathLength += dm->Distance(s, t);
    double currentClearance = MinEdgeClearance(s, t, (*ei).property());
    clearanceVec.push_back(currentClearance);
  }

  //min, max, avg, variance of graph edge variances
  ClearanceStats stats;
  stats.m_pathLength = pathLength;
  stats.m_min = *min_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_max = *max_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_avg = accumulate(clearanceVec.begin(), clearanceVec.end(), 0.0) /
    clearanceVec.size();
  double varSum = 0;
  for(auto&  i : clearanceVec)
    varSum += sqr(i - stats.m_avg);
  stats.m_var = varSum / clearanceVec.size();

  return stats;
}

template<class MPTraits>
ClearanceStats
ClearanceUtility<MPTraits>::
PathClearance(vector<Cfg>& _path) {
  if(_path.empty())
    return ClearanceStats();

  GraphType* g = this->GetRoadmap()->GetGraph();
  auto dm = this->GetDistanceMetric(m_dmLabel);

  typedef typename GraphType::EI EI;
  typedef typename GraphType::VI VI;
  typedef typename GraphType::EID EID;
  typedef typename GraphType::const_vertex_iterator CVI;

  double pathLength = 0;
  vector<double> clearanceVec;

  for(auto cit = _path.begin(); cit + 1 != _path.end(); ++cit) {
    pathLength += dm->Distance(*cit, *(cit + 1));

    WeightType weight;
    bool sourceFound = false;
    CVI si;
    for(si = g->begin(); si != g->end(); ++si)
      if(si->property() == *cit) {
        sourceFound = true;
        break;
      }
    if(sourceFound) {
      bool targetFound = false;
      CVI ti;
      for(ti = g->begin(); ti != g->end(); ++ti)
        if(ti->property() == *(cit + 1)) {
          targetFound = true;
          break;
        }
      if(targetFound) {
        VI vi;
        EI ei;
        EID ed(VID(si->descriptor()), VID(ti->descriptor()));
        if(g->find_edge(ed, vi, ei))
          weight = (*ei).property();
      }
    }
    double currentClearance = MinEdgeClearance(*cit, *(cit + 1), weight);
    clearanceVec.push_back(currentClearance);
  }

  //min, max, avg, variance of graph edge variances
  ClearanceStats stats;
  stats.m_pathLength = pathLength;
  stats.m_min = *min_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_max = *max_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_avg = accumulate(clearanceVec.begin(), clearanceVec.end(), 0.0) /
    clearanceVec.size();
  double varSum = 0;
  for(auto&  i : clearanceVec)
    varSum += sqr(i - stats.m_avg);
  stats.m_var = varSum / clearanceVec.size();

  return stats;
}


template<class MPTraits>
double
ClearanceUtility<MPTraits>::
MinEdgeClearance(const CfgType& _c1, const CfgType& _c2,
    const WeightType& _weight) {
  if(_weight.HasClearance())
    return _weight.GetClearance();

  Environment* env = this->GetEnvironment();
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto robot = this->GetTask()->GetRobot();
  double minClearance = 1e6;

  //Reconstruct the path given the two nodes
  vector<CfgType> intermediates = _weight.GetIntermediates();
  vector<CfgType> reconEdge;

  if(_weight.GetLPLabel() != "RRTExpand") {
    reconEdge = this->GetLocalPlanner(_weight.GetLPLabel())->
      ReconstructPath(_c1, _c2, intermediates,
          env->GetPositionRes(), env->GetOrientationRes());
  }
  else {
    StraightLine<MPTraits> sl;
    sl.SetMPLibrary(this->GetMPLibrary());
    sl.Initialize();
    intermediates.insert(intermediates.begin(), _c1);
    intermediates.push_back(_c2);
    for(auto cit = intermediates.begin(); cit + 1 != intermediates.end();
        ++cit) {
      StatClass dummyStats;
      LPOutput<MPTraits> lpOutput;
      CfgType col(robot);
      vector<CfgType> edge = sl.ReconstructPath(*cit, *(cit + 1), intermediates,
          env->GetPositionRes(), env->GetOrientationRes());
      reconEdge.insert(reconEdge.end(), edge.begin(), edge.end());
    }
  }

  reconEdge.insert(reconEdge.begin(), _c1);
  reconEdge.push_back(_c2);
  for(auto it = reconEdge.begin(); it != reconEdge.end(); ++it) {
    CDInfo collInfo;
    CfgType clrCfg(robot);
    //Decide which collision info function to use
    CollisionInfo(*it, clrCfg, env->GetBoundary(), collInfo);
    double currentClearance = collInfo.m_minDist;
    //If newly computed clearance is less than the previous minimum
    //it becomes the minimum
    if(currentClearance < minClearance)
      minClearance = currentClearance;
  }
  return minClearance;
}

/*--------------------------- Medial Axis Utility ----------------------------*/

template<class MPTraits>
MedialAxisUtility<MPTraits>::
MedialAxisUtility(
    string _vcLabel, string _dmLabel,
    bool _exactClearance, bool _exactPenetration,
    size_t _clearanceRays, size_t _penetrationRays,
    bool _useBBX, bool _positional, bool _debug,
    double _epsilon, size_t _historyLength) :
    ClearanceUtility<MPTraits>(_vcLabel, _dmLabel,
      _exactClearance, _exactPenetration, _clearanceRays, _penetrationRays,
      _useBBX, _positional, _debug),
    m_epsilon(_epsilon), m_historyLength(_historyLength) {
  this->SetName("MedialAxisUtility");
}


template<class MPTraits>
MedialAxisUtility<MPTraits>::
MedialAxisUtility(XMLNode& _node) : ClearanceUtility<MPTraits>(_node) {
  this->SetName("MedialAxisUtility");

  m_epsilon = _node.Read("epsilon", false, m_epsilon, 0.0, 1.0,
      "Epsilon-Close to the MA (fraction of the resolution)");
  m_historyLength = _node.Read("historyLength", false, m_historyLength,
      size_t(3), size_t(100), "History Length");
}


template<class MPTraits>
void
MedialAxisUtility<MPTraits>::
Print(ostream& _os) const {
  ClearanceUtility<MPTraits>::Print(_os);
  _os << "\tepsilon::" << m_epsilon << endl
      << "\thistory length::" << m_historyLength << endl;
}


template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::
PushToMedialAxis(CfgType& _cfg, const Boundary* const _b) {
  // Initialization
  string callee = this->GetNameAndLabel() + "::PushToMedialAxis";
  if(this->m_debug)
    cout << endl << callee << endl << "Being Pushed: " << _cfg;

  auto vcm = this->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // If invalid, push to the outside of the obstacle
  bool inside = vcm->IsInsideObstacle(_cfg);
  bool inCollision = !(vcm->IsValid(_cfg, tmpInfo, callee));

  if(this->m_debug)
    cout << " Inside/In-Collision: " << inside << "/" << inCollision << endl;

  if(inside || inCollision) {
    if(!PushFromInsideObstacle(_cfg, _b))
      return false;
  }

  // Cfg is free, find medial axis
  if(!PushCfgToMedialAxis(_cfg, _b)) {
    if(this->m_debug)
      cout << "Not found!! ERR in pushtomedialaxis" << endl;
    return false;
  }

  if(this->m_debug)
    cout << "FINAL CfgType: " << _cfg << endl
         << callee << "::END" << endl << endl;

  return true;
}


template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::
PushFromInsideObstacle(CfgType& _cfg, const Boundary* const _b) {
  string callee = this->GetNameAndLabel() + "::PushFromInsideObstacle";
  if(this->m_debug)
    cout << callee << endl << " CfgType: " << _cfg << endl;

  CfgType transCfg = _cfg;
  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // Determine direction to move.
  if(this->CollisionInfo(_cfg, transCfg, _b, tmpInfo)) {
    if(this->m_debug)
      cout << "Clearance Cfg: " << transCfg << endl;
  }
  else
    return false;

  // Bad collision information call might return the current Cfg as the
  // clearance Cfg.
  if(transCfg == _cfg)
    return false;

  _cfg = transCfg;

  if(this->m_debug)
    cout << "FINAL CfgType: " << _cfg << endl << callee << "::END " << endl;

  return true;
}


template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::
PushCfgToMedialAxis(CfgType& _cfg, const Boundary* const _b) {
  string callee = this->GetNameAndLabel() + "::PushCfgToMedialAxis";
  if(this->m_debug)
    cout << callee << endl << "Cfg: " << _cfg << " eps: " << m_epsilon << endl;

  Environment* env = this->GetEnvironment();
  auto vcm = this->GetValidityChecker(this->m_vcLabel);
  auto robot = this->GetTask()->GetRobot();

  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // Should already be in free space
  if(vcm->IsInsideObstacle(_cfg))
    return false;

  // Determine positional direction to move
  CfgType transCfg(robot);
  CDInfo prevInfo;
  if(!FindInitialDirection(_cfg, _b,
        env->GetPositionRes(), transCfg, prevInfo))
    return false;

  CfgType startCfg(robot), endingCfg(robot);
  double upperBound = 0, lowerBound = 0, stepSize = 0;
  bool borderFound;
  if(this->m_exactClearance)
    borderFound = FindMedialAxisBorderExact(_cfg, _b, transCfg, prevInfo,
        startCfg, endingCfg, upperBound, lowerBound, stepSize);
  else
    borderFound = FindMedialAxisBorderApprox(_cfg, _b, transCfg, prevInfo,
        startCfg, endingCfg, upperBound, lowerBound, stepSize);
  if(!borderFound)
    return false;

  double middle = (lowerBound + upperBound) / 2.;
  if(this->m_debug)
    cout << "Lower and upper bounds: " << lowerBound << "/" << middle << "/"
         << upperBound << endl;
  CfgType midMCfg = transCfg * middle + _cfg;

  if(this->m_debug) {
    VDClearComments();
    cout << "start/mid/end: " << endl
         << startCfg << endl << midMCfg << endl << endingCfg << endl;
  }

  vector<double> dists(5, 0);
  CfgType tmpTransCfg(robot);
  bool passed = this->CollisionInfo(startCfg, tmpTransCfg, _b, tmpInfo);
  if(!passed)
    return false;
  dists[0] = tmpInfo.m_minDist;

  passed = this->CollisionInfo(midMCfg, tmpTransCfg, _b, tmpInfo);
  if(!passed)
    return false;
  dists[2] = tmpInfo.m_minDist;

  passed = this->CollisionInfo(endingCfg, tmpTransCfg, _b, tmpInfo);
  if(!passed)
    return false;
  dists[4] = tmpInfo.m_minDist;

  double middleS = (lowerBound + middle) / 2.;
  CfgType midSCfg = transCfg * middleS + _cfg;

  double middleE = (middle + upperBound) / 2.;
  CfgType midECfg = transCfg * middleE + _cfg;

  if(this->m_debug) {
    cout << "dists: ";
    for(size_t i = 0; i < dists.size(); ++i)
      cout << dists[i] << " ";
    cout << "\nstart/mids/end: " << endl << startCfg << endl << midSCfg
         << endl << midMCfg << endl << midECfg << endl << endingCfg << endl;
  }

  midMCfg = BinarySearchForPeaks(startCfg, midMCfg, endingCfg,
      lowerBound, upperBound, transCfg, _cfg, _b, dists);

  if(midMCfg == CfgType(robot))
    return false;

  if(this->m_debug) {
    VDComment("Final CFG");
    VDAddTempCfg(midMCfg, true);
    VDClearLastTemp();
    VDClearComments();
    VDClearLastTemp();
    VDClearLastTemp();
  }

  _cfg = midMCfg;

  if(this->m_debug)
    cout << "FINAL Cfg: " << _cfg << " steps: " << stepSize << endl;

  return true;
}


template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::
FindInitialDirection(CfgType& _cfg, const Boundary* const _b, double _posRes,
    CfgType& _transCfg, CDInfo& _prevInfo) {
  _prevInfo.ResetVars();
  _prevInfo.m_retAllInfo = true;

  if(this->CollisionInfo(_cfg, _transCfg, _b, _prevInfo)) {
    if(_prevInfo.m_minDist < numeric_limits<float>::epsilon()) {
      if(this->m_debug)
        cout << "Start Cfg adjacent to obstacle" << endl;
      return false;
    }

    _transCfg = _cfg - _transCfg;
    double magnitude = 0;
    for(size_t i = 0; i < _transCfg.DOF(); ++i)
      magnitude += _transCfg[i] * _transCfg[i];
    magnitude = sqrt(magnitude);

    for(size_t i = 0; i < _transCfg.DOF(); ++i) {
      _transCfg[i] *= _posRes / magnitude;
      if(i > _transCfg.PosDOF() && _transCfg[i] > .5)
        _transCfg[i]--;
    }
    if(this->m_debug)
      cout << "TRANS CFG: " << _transCfg << endl;

    return true;
  }
  else
    return false;
}


template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::
FindMedialAxisBorderExact(const CfgType& _cfg, const Boundary* const _b,
    CfgType& _transCfg, CDInfo& _prevInfo, CfgType& _startCfg,
    CfgType& _endingCfg, double& _upperBound, double& _lowerBound,
    double& _stepSize) {
  Environment* env = this->GetEnvironment();
  auto robot = _cfg.GetRobot();
  auto multiBody = _cfg.GetMultiBody();
  auto vcm = this->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // Determine gap for medial axis
  CfgType tmpCfg = _cfg;
  CfgType heldCfg(robot);
  _stepSize = 0.0;
  bool witnessPointMoved = false;

  while(!witnessPointMoved) {
    // Increment
    heldCfg = tmpCfg;
    tmpCfg = _transCfg * _stepSize + _cfg;

    // Test for in BBX and inside obstacle
    bool inside = false;
    bool inBBX = env->InBounds(tmpCfg, _b);
    if(this->m_debug) {
      VDAddTempCfg(tmpCfg, (inside || !inBBX));
      VDClearLastTemp();
    }

    // If inside obstacle or out of the bbx, step back
    if(inside || !inBBX) {
      if(!inBBX && !this->m_useBBX) {
        return false;
      }
      //TMP FIX::Alway return false when not inside and obstacle and inside the
      //bounding box
      return false;
    }
    else {
      tmpInfo.ResetVars();
      tmpInfo.m_retAllInfo = true;

      // If tmp is valid, move on to next step
      if(this->m_debug)
        cout << "TMP Cfg: " << tmpCfg;
      CfgType tmpTransCfg(robot);
      if(this->CollisionInfo(tmpCfg, tmpTransCfg, _b, tmpInfo)) {
        if(!tmpCfg.GetLabel("VALID"))
          return false;
        Vector3d transDir = tmpInfo.m_objectPoint - _prevInfo.m_objectPoint;
        if(this->m_debug)
          cout << " obst: " << tmpInfo.m_nearestObstIndex;
        double tmpDist = 0.0;
        for(size_t i = 0; i < _transCfg.PosDOF(); ++i)
          tmpDist += transDir[i] * transDir[i];
        tmpDist = sqrt(tmpDist);
        if(tmpDist > multiBody->GetBoundingSphereRadius() *
            (2. + numeric_limits<float>::epsilon())) { // TODO: better value
          if(this->m_debug)
            cout << "\n WP moved: " << tmpDist;
          witnessPointMoved = true;
        }
        _prevInfo = tmpInfo;
        if(this->m_debug)
          cout << " clearance: " << tmpInfo.m_minDist << endl;
      }
      else { // Couldn't calculate clearance info
        if(this->m_debug)
          cout << "BROKE!" << endl;
        return false;
      }

      // Increment step size
      if(!witnessPointMoved)
        _stepSize++;
    }
  }

  // Setup start, middle and end CfgTypes
  _startCfg = heldCfg;
  _endingCfg = tmpCfg;
  if(this->m_debug) {
    VDComment("Binary Cfgs: ");
    VDAddTempCfg(_startCfg, true);
    VDAddTempCfg(_endingCfg, true);
    cout << "\nCalculating Mid... stepSize/cbStepSize: " << _stepSize << "/0\n";
  }

  _upperBound = _stepSize;
  _lowerBound = _upperBound - 1.;

  return true;
}


template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::
FindMedialAxisBorderApprox(const CfgType& _cfg, const Boundary* const _b,
    CfgType& _transCfg, CDInfo& _prevInfo, CfgType& _startCfg,
    CfgType& _endingCfg, double& _upperBound, double& _lowerBound,
    double& _stepSize) {
  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  auto vcm = this->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // Determine gap for medial axis
  CfgType tmpCfg = _cfg;
  CfgType heldCfg(robot);
  _stepSize = 0.0;
  double cbStepSize = 0.0;
  deque<double> segDists;
  deque<CfgType> segCfgs;
  bool peakFound = false, checkBack = false, fellOut = false;
  double errEps = numeric_limits<double>::epsilon();

  while(!peakFound) {

    // Increment
    heldCfg = tmpCfg;
    tmpCfg = _transCfg * (checkBack ? cbStepSize : _stepSize) + _cfg;

    // Test for in BBX and inside obstacle
    bool inside = vcm->IsInsideObstacle(tmpCfg);
    bool inBBX = env->InBounds(tmpCfg, _b);
    if(this->m_debug) {
      VDAddTempCfg(tmpCfg, (inside || !inBBX));
      VDClearLastTemp();
    }

    // If inside obstacle or out of the bbx, step back
    if(inside || !inBBX) {
      if(!inBBX && !this->m_useBBX) {
        return false;
      }
      if(segCfgs.size() > 0) {
        fellOut = true;
        break;
      }
      else {
        return false;
      }
    }
    else {
      tmpInfo.ResetVars();
      tmpInfo.m_retAllInfo = true;

      // If tmp is valid, move on to next step
      if(this->m_debug)
        cout << "TMP Cfg: " << tmpCfg;
      CfgType tmpTransCfg(robot);
      if(this->CollisionInfo(tmpCfg, tmpTransCfg, _b, tmpInfo)) {
        _prevInfo = tmpInfo;
        if(this->m_debug)
          cout << " clearance: " << tmpInfo.m_minDist;

        // Update clearance history distances
        if(segDists.size() > m_historyLength) {
          segDists.pop_front();
          segCfgs.pop_front();
        }
        if(checkBack) {
          segDists.push_front(tmpInfo.m_minDist);
          segCfgs.push_front(tmpCfg);
        }
        else {
          segDists.push_back(tmpInfo.m_minDist);
          segCfgs.push_back(tmpCfg);
          checkBack = false;
        }
      }
      else { // Couldn't calculate clearance info
        if(this->m_debug)
          cout << "BROKE!" << endl;
        return false;
      }

      // enumerate deltas to find peak
      int posCnt = 0, negCnt = 0, zroCnt = 0;
      for(auto dit = segDists.begin(); dit + 1 != segDists.end(); ++dit) {
        double distDelta = *(dit + 1) - (*dit);
        if(distDelta > errEps)
          ++posCnt;
        else if(distDelta < -errEps)
          ++negCnt;
        else
          ++zroCnt;
      }
      if(this->m_debug)
        cout << "  Pos/Zero/Neg counts: " << posCnt << "/"
             << zroCnt << "/" << negCnt << endl;
      if((negCnt > 0 && negCnt > posCnt) || (zroCnt > 0 && zroCnt > posCnt)) {
        // Only see negatives or zeros, check back
        if(posCnt < 1 && (zroCnt < 1 || negCnt < 1)) {
          --cbStepSize;
          --_stepSize;
          checkBack = true;
        }
        else {
          peakFound = true;
          if(this->m_debug) {
            cout << "Found peak!  Dists: ";
            for(size_t i = 0; i < segDists.size(); ++i)
              cout << segDists[i] << " ";
            cout << endl;
          }
        }
      }
      // No peak found
      else
        checkBack = false;

      if(this->m_debug)
        cout << endl;

      // Increment step size
      if(!peakFound)
        ++_stepSize;
    }
  }

  // Check if there are enough cfgs
  if(segCfgs.size() < 2)
    return false;

  // Setup start, middle and end CfgTypes
  _startCfg = segCfgs[0];
  _endingCfg = segCfgs[segCfgs.size() - 1];

  if(this->m_debug) {
    VDComment("Binary Cfgs: ");
    VDAddTempCfg(_startCfg, true);
    VDAddTempCfg(_endingCfg, true);
    cout << "\nCalculating Mid... _stepSize/cbStepSize: "
         << _stepSize << "/" << cbStepSize << endl;
    if(fellOut)
      cout << "\n\nFellOut\n\n";
  }

  _upperBound = fellOut ? _stepSize - 1. : _stepSize;
  if(fellOut) {
    _lowerBound = _upperBound - 1.;
    return false;
  }
  else {
    if(segCfgs.size() < (_stepSize - cbStepSize))
      _lowerBound = _stepSize - segCfgs.size();
    else
      _lowerBound = cbStepSize;
  }

  return true;
}


template<class MPTraits>
typename MPTraits::CfgType
MedialAxisUtility<MPTraits>::
BinarySearchForPeaks(CfgType& _startCfg, CfgType& _midMCfg, CfgType& _endingCfg,
    double _lowerBound, double _upperBound, CfgType& _transCfg, CfgType& _cfg,
    const Boundary* const _b, vector<double>& _dists) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);
  auto robot = this->GetTask()->GetRobot();

  // Variables for modified binary search
  size_t attempts = 0, maxAttempts = 20, badPeaks = 0, maxBadPeaks = 10;
  vector<double> deltas(4, 0), distsFromMax(5, 0);
  bool peaked = false;
  double errEps = numeric_limits<double>::epsilon();

  do { // Modified Binary search to find peaks
    if(this->m_debug) {
      VDAddTempCfg(_midMCfg, true);
      VDClearLastTemp();
    }

    // Update Cfgs
    attempts++;
    double middle = (_lowerBound + _upperBound)/ 2.;
    double middleS = (_lowerBound + middle) / 2.;
    double middleE = (middle + _upperBound) / 2.;
    if(this->m_debug)
      cout << "Bounds: " << _lowerBound << "/"
           << middleS << "/" << middle << "/" << middleE << "/"
           << _upperBound << endl;
    CfgType midSCfg = _transCfg*middleS + _cfg;
    CfgType midECfg = _transCfg*middleE + _cfg;

    CfgType tmpTransCfg(robot);
    CDInfo tmpInfo;
    bool passed = this->CollisionInfo(midSCfg, tmpTransCfg, _b, tmpInfo);
    if(!passed)
      return CfgType(robot);
    _dists[1] = tmpInfo.m_minDist;

    passed = this->CollisionInfo(midECfg, tmpTransCfg, _b, tmpInfo);
    if(!passed)
      return CfgType(robot);
    _dists[3] = tmpInfo.m_minDist;

    // Compute Deltas and Max Distance
    deltas.clear();
    distsFromMax.clear();
    double maxDist = _dists[0];
    if(this->m_debug)
      cout << "Deltas: ";

    for(auto dit = _dists.begin(); dit + 1 != _dists.end(); ++dit) {
      maxDist = max(maxDist, *(dit + 1));
      deltas.push_back(*(dit + 1) - *dit);
      if(this->m_debug)
        cout << " " << deltas.back();
    }
    for(auto dit = _dists.begin(); dit != _dists.end(); ++dit)
      distsFromMax.push_back(maxDist - *dit);

    if(this->m_debug)
      cout << endl;

    // Determine Peak
    size_t peak;
    if(deltas[0] > errEps && deltas[1] <= errEps && distsFromMax[1] < errEps) {
      peak = 1;
      _endingCfg = _midMCfg;
      _midMCfg = midSCfg;
      _dists[4] = _dists[2];
      _dists[2] = _dists[1];
      _upperBound = middle;
    }
    else if(deltas[1] > errEps &&
        deltas[2] <= errEps && distsFromMax[2] < errEps) {
      peak = 2;
      _startCfg = midSCfg;
      _endingCfg = midECfg;
      _dists[0] = _dists[1];
      _dists[4] = _dists[3];
      _lowerBound = middleS;
      _upperBound = middleE;
    }
    else if(deltas[2] > errEps &&
        deltas[3] <= errEps && distsFromMax[3] < errEps) {
      peak = 3;
      _startCfg = _midMCfg;
      _midMCfg = midECfg;
      _dists[0] = _dists[2];
      _dists[2] = _dists[3];
      _lowerBound = middle;
    }
    else {
      if(deltas[0] > errEps &&
          deltas[1] > errEps && deltas[2] > errEps && deltas[3] > errEps) {
        peak = 3;
        _startCfg = _midMCfg;
        _midMCfg = midECfg;
        _dists[0] = _dists[2];
        _dists[2] = _dists[3];
        _lowerBound = middle;
      }
      else if(deltas[0] < -errEps &&
          deltas[1] < -errEps && deltas[2] < -errEps && deltas[3] < -errEps) {
        peak = 1;
        _endingCfg = _midMCfg;
        _midMCfg = midSCfg;
        _dists[4] = _dists[2];
        _dists[2] = _dists[1];
        _upperBound = middle;
      }
      // No peak found, recalculate old, mid and new clearance
      else {
        badPeaks++;
        peak = -1;

        if(this->CollisionInfo(_startCfg, tmpTransCfg, _b, tmpInfo)
            && _dists[0] >= tmpInfo.m_minDist)
          _dists[0] = tmpInfo.m_minDist;

        if(this->CollisionInfo(_midMCfg, tmpTransCfg, _b, tmpInfo)
            && _dists[2] >= tmpInfo.m_minDist)
          _dists[2] = tmpInfo.m_minDist;

        if(this->CollisionInfo(_endingCfg, tmpTransCfg, _b, tmpInfo)
            && _dists[4] >= tmpInfo.m_minDist)
          _dists[4] = tmpInfo.m_minDist;
      }
    }
    if(this->m_debug)
      cout << " peak: " << peak << "  cfg: " << _midMCfg;
    double gapDist = dm->Distance(_startCfg, _endingCfg);
    if(this->m_debug)
      cout << " gap: " << gapDist << endl;
    if(m_epsilon >= gapDist)
      peaked = true;

  } while(!peaked && attempts < maxAttempts && badPeaks < maxBadPeaks);

  return _midMCfg;
}

#endif

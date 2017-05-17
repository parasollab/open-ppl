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

    double GetPositionResolution() const {return m_rayTickResolution;}
    double GetOrientationResolution() const {return m_orientationResolution;}

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

    // These replaced two similar members from the ClearanceUtility
    // class (m_approxStepSize and m_approxResolution):
    double m_rayTickResolution;//no default values as these are required in node
    double m_orientationResolution;

    //These two are linearly related based on the resolution for ray ticking.
    // The max ray magnitude is just how far out a ray will be attempted.
    double m_maxRayMagnitude;
    size_t m_maxRayIterations;

    //These are just for getting rayTickResolution and orientationResolution.
    // They get multiplied into their respective values from the environment,
    // and should only really be used there. See Initialize() for more info.
    double m_orientationResFactor{0.};
    double m_positionalResFactor{0.};

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
    /// nearest obstacle at the resolution until a second witness (invalid cfg)
    /// is found and then the midpoint of those two cfgs will be the MA cfg.
    bool PushCfgToMedialAxisMidpointRule(CfgType& _cfg, const Boundary* const _b);
    ///@}

  private:

    ///@name Internal State
    ///@{

    // m_epsilon is used only externally right now (not used in pushing to MA)
    double m_epsilon{.1};
    size_t m_historyLength{5};

    ///@}
};

/*---------------------------- Clearance Utility -----------------------------*/

template<class MPTraits>
ClearanceUtility<MPTraits>::
ClearanceUtility(
    string _vcLabel, string _dmLabel, bool _exactClearance,
    bool _exactPenetration, size_t _clearanceRays, size_t _penetrationRays,
    double _approxStepSize, double _approxResolution, bool _useBBX,
    bool _positional, bool _debug):
    m_vcLabel(_vcLabel), m_dmLabel(_dmLabel),
    m_useBBX(_useBBX), m_positional(_positional),
    m_exactClearance(_exactClearance), m_exactPenetration(_exactPenetration),
    m_clearanceRays(_clearanceRays), m_penetrationRays(_penetrationRays)
    {
  this->SetName("ClearanceUtility");
  this->m_debug = _debug;
  this->m_rayTickResolution = 0.;
  this->m_orientationResolution = 0.;
  this->m_maxRayMagnitude = DBL_MAX;//So that Initialize() will update this.
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

  //Update the orientation and positional resolutions based on input factor.
  //if one is approx, ensure we initialize correctly:
  bool useApprox = !m_exactClearance || !m_exactPenetration;
  m_positionalResFactor = _node.Read("rayTickPosResolutionFactor", useApprox,
          1., 0., MAX_DBL, "Resolution factor for ray ticking, gets multiplied "
              "into the environment/CD resolution");

  m_orientationResFactor = _node.Read("orientationResolutionFactor", useApprox,
      1., 0., MAX_DBL, "Resolution factor for orientation, gets multiplied "
          "into the environment/CD resolution");


  //I cannot access env vars like orientation/position res and, so we must do
  // that in Initialize(). If we are doing approx but don't Initialize(),
  // then we SHOULD have problems, which is why I'm setting these to 0.
  m_rayTickResolution = 0.;
  m_orientationResolution = 0.;

  //get the max distance rays can go to find a witness:
  m_maxRayMagnitude = _node.Read(
        "maxRayTickMagnitude", useApprox, 100., 0., DBL_MAX,
        "Total magnitude that rays will search, determines iteration limits");


  // If using approximate calculations, require number of rays to be defined.
  this->m_clearanceRays =
      _node.Read("clearanceRays", !m_exactClearance,
      m_clearanceRays, size_t(1), size_t(1000), "Number of clearance Rays");

  // simply have clearance rays defined as a multiple of penetration:
  double penetrationRayFactor =
      _node.Read("penetrationRayFactor", !m_exactPenetration,
      2., 0., 1000., "Number of penetration Rays as multiple of clearance rays");
  this->m_penetrationRays = (size_t)(m_clearanceRays * penetrationRayFactor);

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
    _os << "\trayTickResolution = " << m_rayTickResolution << endl
        << "\torientationResolution = " << m_orientationResolution << endl;
}


//NOTE: Calling initialize is absolutely necessary if doing approximate
// clearance or approximate penetrations
template <class MPTraits>
void
ClearanceUtility<MPTraits>::
Initialize() {
  if(m_maxRayMagnitude == DBL_MAX) {
    m_maxRayMagnitude = this->GetEnvironment()->GetBoundary()->GetMaxDist();
  }

  //Note that calling initialize on a MedialAxisUtility object will call this.
  m_rayTickResolution =
        m_positionalResFactor*this->GetEnvironment()->GetPositionRes();

  m_orientationResolution =
        m_orientationResFactor*this->GetEnvironment()->GetOrientationRes();

  m_maxRayIterations = (size_t)(m_maxRayMagnitude/m_rayTickResolution);

  if(this->m_debug) {
    if(m_exactClearance)
      std::cout << "Using exact clearance" << std::endl;
    else
      std::cout << "Using approximate clearance" << std::endl;

    if(m_exactPenetration)
        std::cout << "Using exact penetration" << std::endl;
      else
        std::cout << "Using approximate penetration" << std::endl;
  }
}


template<class MPTraits>
bool
ClearanceUtility<MPTraits>::
CollisionInfo(CfgType& _cfg, CfgType& _clrCfg, const Boundary* const _b,
    CDInfo& _cdInfo) {
  if(m_exactClearance)
    return ExactCollisionInfo(_cfg, _clrCfg, _b, _cdInfo);
  else
  {
    if(this->m_debug)
      std::cout << "entering ApproxCollisionInfo" << std::endl;
    auto a = ApproxCollisionInfo(_cfg, _clrCfg, _b, _cdInfo);
    if(this->m_debug)
      std::cout << "exited ApproxCollisionInfo" << std::endl;
    return a;
  }
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

  //if initially invalid, it appears this returns a valid witness cfg anyways
  if(!initValidity) {
    CfgType incr(robot), tmpCfg = _clrCfg;
    int nTicks;
    incr.FindIncrement(_cfg, _clrCfg, &nTicks,
        env->GetPositionRes(), env->GetOrientationRes());

    bool tmpValidity = vcm->IsValid(tmpCfg, callee);
    tmpValidity = tmpValidity && !vcm->IsInsideObstacle(tmpCfg);
    while(!tmpValidity) {//increment until validity acheived
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
  const string fName = "ApproxCollisionInfo: ";
  StatClass* stats = this->GetStatClass();

  // Check computation cache
  if(_cfg.m_witnessCfg.get() != 0) {
    _cdInfo = _cfg.m_clearanceInfo;
    _clrCfg = CfgType(*_cfg.m_witnessCfg);
    return true;
  }

  // If in BBX, check validity to get _cdInfo, return false if not valid
  Environment* env = this->GetEnvironment();
  if(!env->InBounds(_cfg, _b)) {
    std::cout << fName + "returning false from not being in bounds initially" << std::endl;
    return false;
  }

  // Initialization
  string callee = this->GetNameAndLabel() + "::ApproxCollisionInfo";
  auto dm  = this->GetDistanceMetric(m_dmLabel);
  auto vcm = this->GetValidityChecker(m_vcLabel);
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  // Compute initial validity state
  const bool initValidity = vcm->IsValid(_cfg, _cdInfo, callee);
  if(this->m_debug)
    std::cout << "\tinitValidity = " << initValidity;

  // Setup random rays
  size_t numRays;
  if(initValidity)
    numRays = m_clearanceRays;
  else
    numRays = m_penetrationRays;
  if(this->m_debug)
    cout << "DEBUG:: numRays = " << numRays << endl;
  vector<Ray<CfgType>> rays;

  //initial ray starts at rand angle, then all others are uniformly distributed:
  double angleRad = 2.*PI*rand()/(double)RAND_MAX;

  for(size_t i = 0; i < numRays; ++i) {
    CfgType tmpDirection(this->GetTask()->GetRobot());

    //TODO expand to 3D, and then to any Dimensions for uniform distribution:
    if(_cfg.PosDOF() == 2)
    {
      // This evenly divides up the rest of the angles all the way around,
      // starting from the random initial angle from above.
      angleRad += 2. * PI * (1. / numRays);//note this happens numRays times
      std::vector<double> dofData = {cos(angleRad), sin(angleRad)};
      tmpDirection.SetData(dofData); // There's likely a better way to do this
    }
    else
    {
      //The old, non-uniform way to get each ray:
      tmpDirection.GetRandomRay(m_rayTickResolution, dm, false);
      //This was m_approxStepSize*m_rayTickResolution, but
      // I'm pretty sure the intention was to use a certain multiple of the
      // CD/environment resolution, but that's now built into rayTickRes.
    }

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
      tmpDirection *= m_rayTickResolution / sqrt(factor);
    }
    if(this->m_debug)
    {
      cout << "DEBUG:: tmpDirection " << i << " is " << tmpDirection << endl;
      cout << "DEBUG:: \tdistance(_cfg, _cfg+tmpDirection) = "
        << dm->Distance(_cfg, _cfg + tmpDirection) << endl;
    }

    rays.push_back(Ray<CfgType>(tmpDirection, _cfg));
  }// end for (numRays)

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
  vector<pair<size_t, CfgType>> candidates;  stats->StartClock(fName + "Everything after ray ticking timer");

  bool stateChangedFlag = false;
  size_t iterations = 0;

  if(this->m_debug)
    cout << "DEBUG:: stepping out along each direction to find candidates";

  //This loop goes until it has just one of its rays go from free->not free
  //or not free->free or iterates n times. Within it, it loops through all
  //rays each time, moving outward along these and checking that condition.
  //m_maxRayIterations is calculated by maxRayMagnitude/rayTickResolution
  stats->StartClock(fName+"Ray iteration clock (finding candidates)");
  while(!stateChangedFlag && iterations++ < m_maxRayIterations) {
    if(this->m_debug)
      cout << "\n\t" << iterations;

    for(auto rit = rays.begin(); rit != rays.end(); ++rit) {
      //step out
      rit->m_tick += rit->m_incr;

      //determine new state
      CDInfo tmpInfo;

      bool currValidity;//will get overwritten

      //Block the expensive validity check by first doing faster boundary check:
      if(m_useBBX && !env->InBounds(rit->m_tick, _b)) {
        //OOB, so we ALWAYS want to trigger a candidate, explicity force it:
        currValidity = !initValidity;
      }
      else {
        //Validity check, but only if we were in bounds for this tick:
        currValidity = vcm->IsValid(rit->m_tick, tmpInfo, callee);
      }

      if(this->m_debug)
        cout << " (currValidity for direction " << distance(rays.begin(), rit)
             << " = " << currValidity << ")";

      //if state has changed, add to candidate list
      if(currValidity != initValidity) {
        stateChangedFlag = true;
        CfgType candidate = rit->m_tick - rit->m_incr;

        //Note: this is pushing back the index of the candidate ray (what
        // distance() returns) and the actual location that we have found
        // right BEFORE the collision configuration was found.
        candidates.push_back(make_pair(distance(rays.begin(), rit), candidate));

        // quit after 1 since we just take the first witness found
        break;
      }
    }
  }//end while (!stateChangedFlag && iterations < max)

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


  // Remove spurious candidate(s):
  // This code is checking whether there are any "incorrect" candidates, meaning
  // that if a witness was found, then the candidate should have an initial cfg
  // position that is valid, and then ticking forward once  should be invalid.
  //I think it's pointless, but it's not a terrible thing to verify, and it's a
  //pretty simple/fast check in the end. It looks a little more complicated than
  //it is, since it is made for there being more than one candidate (now just 1)
  if(this->m_debug)
    cout << "DEBUG:: Verifying candidate is valid\n";

  // This could justifiably be removed, but it's also not a bad verification
  for(auto& cand : candidates) {
    if(this->m_debug)
      cout << "\t" << cand.first;
    CDInfo tmpInfo;

    CfgType lowCfg = rays[cand.first].m_incr * 0.0 + cand.second;
    bool lowValidity = vcm->IsValid(lowCfg, tmpInfo, callee);
    if(m_useBBX)
      lowValidity = (lowValidity && env->InBounds(lowCfg, _b));

    if(this->m_debug)
      cout << " (lowValidity = " << lowValidity << ")" << endl;

    CfgType highCfg = rays[cand.first].m_incr * 1.0 + cand.second;
    bool highValidity = vcm->IsValid(highCfg, tmpInfo, callee);
    if(m_useBBX)
      highValidity = (highValidity && env->InBounds(highCfg, _b));

    if(this->m_debug)
      cout << " (highValidity = " << highValidity << ")" << endl;

    if(lowValidity == highValidity) {
      //there was a problem, but still give the best data while returning false
      _clrCfg = highCfg;
      _cdInfo.m_minDist =
          (initValidity ? 1.0 : -1.0) * dm->Distance(_clrCfg, _cfg);
      if(this->m_debug)
        std::cout << "Returning false from invalid witness candidate"
                  << std::endl;
      return false;
    }
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

  //This is needed, in case a candidate was never found.
  if(candidates.size() == 0) {
    if(this->m_debug)
      std::cout << fName + "returning false from not having any ray candidates"
                << std::endl;
    return false;
  }

  // Set return info
  if(this->m_debug)
    cout << "DEBUG:: setting info, returning\n";

  //give the invalid (one more tick ahead) cfg back as the witness:
  _clrCfg = candidates[0].second + rays[candidates[0].first].m_incr;
  // Note cadidates[0].second is the point before collision, and we know that
  // adding one more m_incr will put us at opposite validity (or OOB)

  _cdInfo.m_minDist = (initValidity ? 1.0 : -1.0) * dm->Distance(_clrCfg, _cfg);


  //This check used to fail a lot of samples after turning off the binary witness
  //refining. This makes sense since the old version relied on the binary search
  //to guarantee a change to _clrCfg, but now we require witnesses of opposite
  //validity, so this is again impossible to trigger (still a good check though)
  if(_clrCfg == _cfg) {//shouldn't happen, but should return an error
    if(this->m_debug)
      std::cout << fName + "returning false from _clrCfg == _cfg" << std::endl;
    return false;
  }

  //Since we want witnesses to now be the opposite validity, if it's
  // a clearance calculation, it should either be out of bounds or invalid,
  //If penetration, then either out of bounds or valid.
  //Note that being out of bounds is ALWAYS a witness trigger.
  CDInfo tmpInfo;
  //check that it's NOT in bounds or that the witness validity is different
  // than the validity initially:
  if(!env->InBounds(_clrCfg, _b) ||
      (initValidity != vcm->IsValid(_clrCfg, tmpInfo, callee))) {
    //do all of the stuff for a successful push:
    _cfg.m_clearanceInfo = _cdInfo;
    _cfg.m_witnessCfg = shared_ptr<Cfg>(new CfgType(_clrCfg));
    return true;
  }
  else {
    //Then the cfg was either in bounds and of the same validity as _cfg:
    if(this->m_debug)
      std::cout << fName + "returning false from _clrCfg being out of bounds or"
            " of opposite validity" << std::endl;
    return false;
  }
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
          m_rayTickResolution, m_orientationResolution);
          //env->GetPositionRes(), env->GetOrientationRes());
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
          m_rayTickResolution, m_orientationResolution);
          //env->GetPositionRes(), env->GetOrientationRes());
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

  // Note that all necessary initialization is done in ClearanceUtility
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
  bool inCollision = !(vcm->IsValid(_cfg, tmpInfo, callee));

  if(this->m_debug)
    cout << "In-Collision: " << inCollision << endl;

  if(inCollision) {
    if(!PushFromInsideObstacle(_cfg, _b)) {
      if(this->m_debug)
        std::cout << callee << "Returning false from not being able to push "
            "from obstacle" << std::endl;

      return false;
    }
  }

  bool pushed = false;
  pushed = PushCfgToMedialAxisMidpointRule(_cfg, _b);


  if(!pushed) {
    if(this->m_debug)
      cout << "Not found!! ERR in pushtomedialaxis" << endl;
    std::cout << callee << "Returning false from not "
                           "being able to push to MA" << std::endl;
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

  // _cfg cannot equal transCfg here, as CollisionInfo would have returned false
  _cfg = transCfg;

  if(this->m_debug)
    cout << "FINAL CfgType: " << _cfg << endl << callee << "::END " << endl;

  return true;
}


// Tim's note:
//This version of PushCfg...() is using the premise of taking the midpoint of
// lines in the C-Space and calling it the MA. Basically the witness point and
// the normal to push the cfg along are found as normal, but this normal is
// simply pushed along until another collision is found, then the midpoint
// between the two is called the MA cfg.
//This method means that the witness only needs to be found at the beginning,
// then the cfg is pushed (checking collisions still) along that line. Only
// doing the witness/ray shooting a single time is a HUGE benefit.
template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::
PushCfgToMedialAxisMidpointRule(CfgType& _cfg, const Boundary* const _b) {
  string callee = this->GetNameAndLabel() + "::PushCfgToMedialAxisMidpointRule";
  if(this->m_debug)
    cout << callee << endl << "Cfg: " << _cfg << " eps: " << m_epsilon << endl;

  Environment* env = this->GetEnvironment();
  auto vcm = this->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // Should already be in free space
  if(vcm->IsInsideObstacle(_cfg)) {
    std::cout << callee <<" Returning false due to already in obstacle" << std::endl;
    return false;
  }

  //first, get the witness that I need
  CfgType firstWitnessCfg = _cfg;//assignment necessary for robot pointer
  if(!this->CollisionInfo(_cfg, firstWitnessCfg, _b, tmpInfo)) {
    std::cout << callee << "Returning false from not being able to"
                           " find first witness" << std::endl;
  }

  //second, find the unit normal
  CfgType unitDirectionToTickCfgAlong = _cfg - firstWitnessCfg;
//  mathtool::Vector<double> normalData(unitDirectionToTickCfgAlong.GetData());
  unitDirectionToTickCfgAlong /= (unitDirectionToTickCfgAlong.Magnitude());


  //third, move along normal until anther "witness" is found
  //we don't need to go any farther than the max boundary, and the number of
  // iterations to go that far is computed here:

  //It's important to note there that, unlike ApproxCollisionInfo(), we don't
  // care about arbitrary state change, we are requiring that _cfg is valid, and
  // as such, both of the witness cfgs SHOULD BE invalid or out of bounds.
  //A better maxIterations might be in order, but this should be fine
  const size_t maxIterations = _b->GetMaxDist()/this->m_rayTickResolution;
  CfgType magnitudeAndDirectionToTick = unitDirectionToTickCfgAlong *
                                                this->m_rayTickResolution;

  //declare these outside so I can reuse them after breaking/finishing the loop:
  bool inBounds = true;
  bool valid = true;
  size_t i;

  for(i = 0; i < maxIterations; i++) {
    //tick
    CfgType tickedCfg = _cfg + magnitudeAndDirectionToTick*(double)i;
    //check validity and if in bounds:
    CDInfo tmpInfo;
    valid = vcm->IsValid(tickedCfg, tmpInfo, callee);
    inBounds = env->InBounds(tickedCfg, _b);
    if(!inBounds || !valid) {
      break;// we have found the second and final witness
    }
  }

  //check that it's truly what we wanted:
  CfgType finalWitnessCfg = _cfg;//assignment necessary for robot pointer
  if(!inBounds || !valid) {
    //Note that i, inBounds, and valid are all preserved from the last
    // run of the loop.
    finalWitnessCfg = _cfg + magnitudeAndDirectionToTick*(double)i;
  }
  else {
    std::cout << callee << " Returning false as no second witness could "
                            "be found" << std::endl;
    return false;
  }

  //fourth, get the midpoint of the two witness cfgs and return it as the MA cfg:
  CfgType cfgMA = (firstWitnessCfg + finalWitnessCfg)/2.0;

  if(this->m_debug) {
    VDComment("Final CFG");
    VDAddTempCfg(cfgMA, true);
    VDClearLastTemp();
    VDClearComments();
    VDClearLastTemp();
    VDClearLastTemp();
  }

  _cfg = cfgMA;

  if(this->m_debug)
    cout << "FINAL Cfg: " << _cfg << endl;

  return true;
}

#endif

#ifndef MEDIALAXISUTILITY_H_
#define MEDIALAXISUTILITY_H_

#include "MPUtils.h"
#include "MetricUtils.h"
#include "LocalPlanners/StraightLine.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"

struct ClearanceStats{
  double m_avg;
  double m_min;
  double m_max;
  double m_var;
  double m_pathLength;

  ClearanceStats() : m_avg(0), m_min(0), m_max(0), m_var(0), m_pathLength(0) {}
};

//used to encapsulate all the fields and functions necessary for clearance and penetration calculations
template<class MPTraits>
class ClearanceUtility : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    ClearanceUtility(MPProblemType* _problem = NULL,
        string _vcLabel = "", string _dmLabel = "",
        bool _exactClearance = false, bool _exactPenetration = false,
        size_t _clearanceRays = 10, size_t _penetrationRays = 10,
        double _approxStepSize = MAX_DBL, double _approxResolution = MAX_DBL,
        bool _useBBX = true, bool _positional = true, bool _debug = false);

    ClearanceUtility(MPProblemType* _problem, XMLNodeReader& _node);

    void ParseXML(XMLNodeReader& _node);

    virtual void Print(ostream& _os) const;

    string GetDistanceMetricLabel() const {return m_dmLabel;}
    string GetValidityCheckerLabel() const {return m_vcLabel;}
    void SetValidityCheckerLabel(const string& _s) {m_vcLabel = _s;}
    bool GetExactClearance() const {return m_exactClearance;}

    //*********************************************************************//
    // Calculate Collision Information                                     //
    //   This is a wrapper function for getting the collision information  //
    // for the medial axis computation, calls either approx or exact       //
    //*********************************************************************//
    bool CollisionInfo(CfgType& _cfg, CfgType& _clrCfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo);

    //*********************************************************************//
    // Get Exact Collision Information Function                            //
    //   Determines the exact collision information by taking the validity //
    // checker results against obstacles to the bounding box to get a      //
    // complete solution                                                   //
    //*********************************************************************//
    bool ExactCollisionInfo(CfgType& _cfg, CfgType& _clrCfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo);

    //*********************************************************************//
    // Get Approximate Collision Information Function                      //
    //   Calculate the approximate clearance using a series of rays. The   //
    // specified number of rays are sent out till they change in validity. //
    // The shortest ray is then considered the best candidate.             //
    //*********************************************************************//
    bool ApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo);

    //*********************************************************************//
    // Get Roadmap Clearance Statistics                                    //
    //   Calculates averages mins and maxes for clearance across roadmaps  //
    // paths and edges                                                     //
    //*********************************************************************//
    ClearanceStats RoadmapClearance();

    ClearanceStats PathClearance(vector<VID>& _path);
    ClearanceStats PathClearance(vector<Cfg>& _path);

    double MinEdgeClearance(const CfgType& _c1, const CfgType& _c2, const WeightType& _weight);
  protected:
    string m_vcLabel;                //validity checker method label
    string m_dmLabel;                //distance metric method label
    bool m_exactClearance;           //use exact clearance calculations
    bool m_exactPenetration;         //use exact penetration calculations
    size_t m_clearanceRays;          //number of rays used to approximate clearance
    size_t m_penetrationRays;        //number of rays used to approximate penetration
    double m_approxStepSize, m_approxResolution;         //initial stepsize and final resolution used for approximate clearance/penetration, multiples of env res
    bool m_useBBX;                   //use bounding box as obstacle
    bool m_positional;               //use only positional dofs
};

template<class MPTraits>
class MedialAxisUtility : public ClearanceUtility<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    MedialAxisUtility(MPProblemType* _problem = NULL,
        string _vcLabel = "", string _dmLabel = "",
        bool _exactClearance = false, bool _exactPenetration = false,
        size_t _clearanceRays = 10, size_t _penetrationRays = 10,
        bool _useBBX = true, bool _positional = true, bool _debug = false,
        double _epsilon = 0.1, size_t _historyLength = 5);

    MedialAxisUtility(MPProblemType* _problem, XMLNodeReader& _node);

    void ParseXML(XMLNodeReader& _node);

    double GetEpsilon() const {return m_epsilon;}

    virtual void Print(ostream& _os) const;

    //***********************************//
    // Main Push To Medial Axis Function //
    //***********************************//
    bool PushToMedialAxis(CfgType& _cfg, shared_ptr<Boundary> _bb);

    //***************************************************************//
    // Push From Inside Obstacle                                     //
    //   In this function, a cfg is known to be inside an obstacle.  //
    // A direction is determined to move the cfg outside of the      //
    // obstacle and is pushed till outside.                          //
    //***************************************************************//
    bool PushFromInsideObstacle(CfgType& _cfg, shared_ptr<Boundary> _bb);

    //***************************************************************//
    // Push Cfg To Medial Axis                                       //
    //   This function is to perform a regular push to medial axis   //
    // algorithm stepping out at the resolution till the medial axis //
    // is found, determined by the clearance.                        //
    //***************************************************************//
    bool PushCfgToMedialAxis(CfgType& _cfg, shared_ptr<Boundary> _bb);

  private:
    bool FindInitialDirection(CfgType& _cfg, shared_ptr<Boundary> _bb, double _posRes, CfgType& _transCfg, CDInfo& _prevInfo);
    bool FindMedialAxisBorderExact(
        const CfgType& _cfg, shared_ptr<Boundary> _bb,
        CfgType& _transCfg, CDInfo& _prevInfo,
        CfgType& _startCfg, CfgType& _endingCfg,
        double& _upperBound, double& _lowerBound, double& _stepSize);
    bool FindMedialAxisBorderApprox(
        const CfgType& _cfg, shared_ptr<Boundary> _bb,
        CfgType& _transCfg, CDInfo& _prevInfo,
        CfgType& _startCfg, CfgType& _endingCfg,
        double& _upperBound, double& _lowerBound, double& _stepSize);
    CfgType BinarySearchForPeaks(CfgType& _startCfg, CfgType& _midMCfg, CfgType& _endingCfg, double _lowerBound, double _upperBound, CfgType& _transCfg, CfgType& _cfg, shared_ptr<Boundary> _bb, vector<double>& _dists);

    double m_epsilon;
    size_t m_historyLength;
};

//#ifdef PMPCfgSurface
template<class MPTraits>
class SurfaceMedialAxisUtility : public MedialAxisUtility<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    SurfaceMedialAxisUtility(MPProblemType* _problem = NULL,
        string _vcLabel = "", string _dmLabel = "");

    //***************************************************************//
    //get clearance functions for surface configurations             //
    //***************************************************************//
    double GetClearance2DSurf(Environment* _env, const Point2d& _pos, Point2d& _cdPt);
    double GetClearance2DSurf(shared_ptr<MultiBody> _mb, const Point2d& _pos, Point2d& _cdPt, double _clear);
    //***************************************************************//
    //2D-Surface version of pushing to MA                            //
    //  Takes a free cfg (surfacecfg) and pushes to medial axis of   //
    // environment defined by the boundary lines of the obstacles    //
    // (multibodies)                                                 //
    //***************************************************************//
    double PushCfgToMedialAxis2DSurf(CfgType& _cfg, shared_ptr<Boundary> _bb, bool& _valid);
};
//#endif

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Clearance Utility
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template<class MPTraits>
ClearanceUtility<MPTraits>::ClearanceUtility(MPProblemType* _problem,
    string _vcLabel, string _dmLabel,
    bool _exactClearance, bool _exactPenetration,
    size_t _clearanceRays, size_t _penetrationRays,
    double _approxStepSize, double _approxResolution,
    bool _useBBX, bool _positional, bool _debug):
  m_vcLabel(_vcLabel), m_dmLabel(_dmLabel),
  m_exactClearance(_exactClearance), m_exactPenetration(_exactPenetration),
  m_clearanceRays(_clearanceRays), m_penetrationRays(_penetrationRays),
  m_approxStepSize(_approxStepSize), m_approxResolution(_approxResolution),
  m_useBBX(_useBBX), m_positional(_positional){
    this->m_name = "ClearanceUtility";
    this->SetMPProblem(_problem);
    this->m_debug = _debug;

    if(m_approxStepSize == MAX_DBL && this->GetMPProblem() != NULL && this->GetMPProblem()->GetEnvironment() != NULL)
      m_approxStepSize = 1 / this->GetMPProblem()->GetEnvironment()->GetBoundary()->GetMaxDist();
    if(m_approxResolution == MAX_DBL && this->GetMPProblem() != NULL && this->GetMPProblem()->GetEnvironment() != NULL)
      m_approxResolution = 1 / this->GetMPProblem()->GetEnvironment()->GetBoundary()->GetMaxDist();
  }

template<class MPTraits>
ClearanceUtility<MPTraits>::ClearanceUtility(MPProblemType* _problem, XMLNodeReader& _node):
  MPBaseObject<MPTraits>(_problem, _node){
    this->m_name = "ClearanceUtility";
    ParseXML(_node);
  }

template<class MPTraits>
void
ClearanceUtility<MPTraits>::ParseXML(XMLNodeReader& _node){
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
  m_dmLabel = _node.stringXMLParameter("dmLabel", true, "", "Distance metric");

  //clearance and penetration types
  string clearanceType = _node.stringXMLParameter("clearanceType", true, "", "Clearance Computation (exact or approx)");
  m_exactClearance = clearanceType.compare("exact")==0;
  string penetrationType = _node.stringXMLParameter("penetrationType",true, "", "Penetration Computation (exact or approx)");
  m_exactPenetration = penetrationType.compare("exact")==0;

  //if approximate calculations require number of rays to be defined
  double minStepSize = 1 / this->GetMPProblem()->GetEnvironment()->GetBoundary()->GetMaxDist();
  m_approxStepSize = _node.numberXMLParameter("stepSize", false, minStepSize, minStepSize, MAX_DBL, "Step size for initial approximate computations as multiple of environment resolution");
  m_approxResolution = _node.numberXMLParameter("resolution", false, minStepSize, minStepSize, MAX_DBL, "Resolution for final approximate computations as multiple of environment resolution");
  m_clearanceRays = _node.numberXMLParameter("clearanceRays", !m_exactClearance, 10, 1, 1000, "Number of Clearance Rays");
  m_penetrationRays = _node.numberXMLParameter("penetrationRays", !m_exactPenetration, 10, 1, 1000, "Number of Penetration Rays");

  m_useBBX = _node.boolXMLParameter("useBBX", false, true, "Use the Bounding Box as an Obstacle");
  m_positional = _node.boolXMLParameter("positional", false, true, "Use only positional DOFs");
}

template<class MPTraits>
void
ClearanceUtility<MPTraits>::Print(ostream& _os) const {
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
  _os << "\tpositional = " << m_positional << endl;
  _os << "\tuseBBX = " << m_useBBX << endl;
  _os << "\tclearance = ";
  _os << ((m_exactClearance) ? "exact" : "approx");
  if(!m_exactClearance) _os << ", " << m_clearanceRays << " rays";
  _os << endl;
  _os << "\tpenetration = ";
  _os << ((m_exactPenetration) ? "exact" : "approx");
  if(!m_exactPenetration) _os << ", " << m_penetrationRays << " rays";
  _os << endl;
  if(!m_exactClearance || !m_exactPenetration) {
    _os << "\tapproxStepSize = " << m_approxStepSize << endl;
    _os << "\tapproxResolution = " << m_approxResolution << endl;
  }
}

//*********************************************************************//
// Calculate Collision Information                                     //
//   This is a wrapper function for getting the collision information  //
// for the medial axis computation, calls either approx or exact       //
//*********************************************************************//
template<class MPTraits>
bool
ClearanceUtility<MPTraits>::CollisionInfo(CfgType& _cfg, CfgType& _clrCfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo){
  if(m_exactClearance)
    return ExactCollisionInfo(_cfg, _clrCfg, _bb, _cdInfo);
  else
    return ApproxCollisionInfo(_cfg, _clrCfg, _bb, _cdInfo);
}

//*********************************************************************//
// Get Exact Collision Information Function                            //
//   Determines the exact collision information by taking the validity //
// checker results against obstacles to the bounding box to get a      //
// complete solution                                                   //
//*********************************************************************//
template<class MPTraits>
bool
ClearanceUtility<MPTraits>::ExactCollisionInfo(CfgType& _cfg, CfgType& _clrCfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo){
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
  Environment* env = this->GetMPProblem()->GetEnvironment();

  // Setup Validity Checker
  string callee = this->GetNameAndLabel() + "::ExactCollisionInfo";
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  // If not in BBX or valid, return false (IsValid gets _cdInfo)
  bool initInside = vcm->IsInsideObstacle(_cfg);             // Initially Inside Obst
  bool initValidity = vcm->IsValid(_cfg, _cdInfo, callee); // Initial Validity
  initValidity = initValidity && !initInside;

  if(!initValidity){
    _cdInfo.m_minDist = -_cdInfo.m_minDist;
  }

  // If not using the bbx, done
  if(m_useBBX){
    // CfgType is now know as good, get BBX and ROBOT info
    boost::shared_ptr<MultiBody> robot = env->GetMultiBody(_cfg.GetRobotIndex());

    // Find closest point between robot and bbx, set if less than min dist from obstacles
    for(int m=0; m < robot->GetFreeBodyCount(); ++m) {
      GMSPolyhedron& poly = robot->GetFreeBody(m)->GetWorldPolyhedron();
      for(size_t j = 0; j < poly.m_vertexList.size(); ++j){
        double clr = _bb->GetClearance(poly.m_vertexList[j]);
        if(clr < _cdInfo.m_minDist){
          _cdInfo.m_robotPoint = poly.m_vertexList[j];
          _cdInfo.m_objectPoint = _bb->GetClearancePoint(poly.m_vertexList[j]);
          _cdInfo.m_minDist = clr;
          _cdInfo.m_nearestObstIndex = -1;
        }
      }
    }
  }

  Vector3d clrDir = _cdInfo.m_objectPoint - _cdInfo.m_robotPoint;
  CfgType stepDir;
  double factor = 0;
  for(size_t i=0; i<_clrCfg.DOF(); i++) {
    if(i < _clrCfg.PosDOF()) {
      _clrCfg[i] = _cfg[i] + clrDir[i];
      stepDir[i] = clrDir[i];
      factor += clrDir[i]*clrDir[i];
    }
    else{
      _clrCfg[i] = _cfg[i];
      stepDir[i] = 0.0;
    }
    stepDir *= env->GetPositionRes()/sqrt(factor);
  }

  if(_clrCfg == _cfg)
    return false;

  if(!initValidity){
    CfgType incr, tmpCfg = _clrCfg;
    int nTicks;
    incr.FindIncrement(_cfg, _clrCfg, &nTicks, env->GetPositionRes(), env->GetOrientationRes());

    bool tmpValidity = vcm->IsValid(tmpCfg, callee);
    tmpValidity = tmpValidity && !vcm->IsInsideObstacle(tmpCfg);
    while(!tmpValidity) {
      tmpCfg += incr;

      tmpValidity = vcm->IsValid(tmpCfg, callee);
      tmpValidity = tmpValidity && !vcm->IsInsideObstacle(tmpCfg);
      bool inBBX = env->InBounds(tmpCfg, _bb);
      if(!inBBX) {
        if(this->m_debug) cout << "ERROR: Fell out of BBX, error out... " << endl;
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

//*********************************************************************//
// Get Approximate Collision Information Function                      //
//   Calculate the approximate clearance using a series of rays. The   //
// specified number of rays are sent out till they change in validity. //
// The shortest ray is then considered the best candidate.             //
//*********************************************************************//

template<class CfgType>
struct Ray {
  CfgType m_incr;
  CfgType m_tick;

  Ray(const CfgType& _i, const CfgType& _t) : m_incr(_i), m_tick(_t) {}
  ~Ray() {}
};

template<class MPTraits>
bool
ClearanceUtility<MPTraits>::ApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo){
  // Check computation cache
  if(_cfg.m_witnessCfg.get() != 0) {
    _cdInfo = _cfg.m_clearanceInfo;
    _clrCfg = CfgType(*_cfg.m_witnessCfg);
    return true;
  }

  // If in BBX, check validity to get _cdInfo, return false if not valid
  Environment* env = this->GetMPProblem()->GetEnvironment();
  if(!env->InBounds(_cfg, _bb))
    return false;

  // Initialization
  string callee = this->GetNameAndLabel() + "::ApproxCollisionInfo";
  DistanceMetricPointer dm  = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  // Compute initial validity state
  bool initInside = vcm->IsInsideObstacle(_cfg);
  bool initValidity = vcm->IsValid(_cfg, _cdInfo, callee);
  if(this->m_debug) cout << "\nDEBUG:: initInside = " << initInside << "\tinitValidity = " << initValidity;
  initValidity = initValidity && !initInside;
  if(this->m_debug) cout << "\tinitValidity -> " << initValidity << endl;

  // Setup random rays
  size_t numRays;
  if(initValidity)
    numRays = m_clearanceRays;
  else
    numRays = m_penetrationRays;
  if(this->m_debug) cout << "DEBUG:: numRays = " << numRays << endl;
  vector<Ray<CfgType> > rays;
  double posRes = env->GetPositionRes();
  for(size_t i=0; i<numRays; ++i) {
    CfgType tmpDirection;
    tmpDirection.GetRandomRay(m_approxStepSize * env->GetPositionRes(), env, dm, false);
    if(this->m_debug) cout << "DEBUG:: tmpDirection " << i << " is " << tmpDirection << endl;
    if(m_positional) { // Use only positional dofs
      double factor=0.0;
      for(size_t j=0; j<tmpDirection.DOF(); j++) {
        if(j < tmpDirection.PosDOF())
          factor += tmpDirection[j]*tmpDirection[j];
        else
          tmpDirection[j] = 0.0;
      }
      tmpDirection *= posRes/sqrt(factor);
    }
    if(this->m_debug) {
      cout << "DEBUG:: tmpDirection " << i << " is " << tmpDirection << endl;
      cout << "DEBUG:: \tdistance(_cfg, _cfg+tmpDirection) = " << dm->Distance(_cfg, _cfg + tmpDirection) << endl;
    }
    rays.push_back(Ray<CfgType>(tmpDirection, _cfg));
  }
  if(this->m_debug) {
    cout << "DEBUG:: rays initialized\n";
    cout << "DEBUG:: \ttick are:\n\t\t";
    for(typename vector<Ray<CfgType> >::const_iterator I = rays.begin(); I != rays.end(); ++I)
      cout << I->m_tick << "\n\t\t";
    cout << endl;
    cout << "DEBUG:: \tincr are:\n\t\t";
    for(typename vector<Ray<CfgType> >::const_iterator I = rays.begin(); I != rays.end(); ++I)
      cout << I->m_incr << "\n\t\t";
    cout << endl;
  }

  // Step out along each direction to determine the candidates
  vector<pair<size_t, CfgType> > candidates;
  bool stateChangedFlag = false;
  size_t iterations = 0, maxIterations=10000; // Arbitrary TODO: Smarter maxIter number
  if(this->m_debug) cout << "DEBUG:: stepping out along each direction to find candidates";
  while(!stateChangedFlag && iterations++ < maxIterations) {
    if(this->m_debug) cout << "\n\t" << iterations;
    for(typename vector<Ray<CfgType> >::iterator I = rays.begin(); I != rays.end(); ++I) {
      //step out
      I->m_tick += I->m_incr;

      //determine new state
      CDInfo tmpInfo;
      bool currInside = vcm->IsInsideObstacle(I->m_tick);
      bool currValidity = vcm->IsValid(I->m_tick, tmpInfo, callee);
      currValidity = currValidity && !currInside;
      if(m_useBBX)
        currValidity = (currValidity && env->InBounds(I->m_tick, _bb));
      if(this->m_debug) cout << " (currValidity for direction " << distance(rays.begin(), I) << " = " << currValidity << ")";

      //if state has changed, add to candidate list
      if(currValidity != initValidity) {
        stateChangedFlag = true;
        CfgType candidate = I->m_tick - I->m_incr;
        candidates.push_back(make_pair(distance(rays.begin(), I), candidate));
      }
    }
  }
  if(this->m_debug) {
    cout << "\nDEBUG:: done stepping out along rays\n";
    cout << "   found " << candidates.size() << " candidates:\n";
    for(typename vector<pair<size_t, CfgType> >::iterator I = candidates.begin(); I != candidates.end(); ++I) {
      cout << "\t" << I->first << ": " << I->second;

      CDInfo tmpInfo;
      bool currInside = vcm->IsInsideObstacle(I->second);
      bool currValidity = vcm->IsValid(I->second, tmpInfo, callee);
      currValidity = currValidity && !currInside;
      if(m_useBBX)
        currValidity = (currValidity && env->InBounds(I->second, _bb));
      cout << " (currValidity = " << currValidity << ")";
      cout << endl;
    }
  }

  // Remove spurious candidates
  // (not sure why but validity states not consistent sometimes and causes infinite loop later in binary search)
  if(this->m_debug) cout << "DEBUG:: checking for spurious candidates and removing them\n";
  vector<bool> remove;
  for(typename vector<pair<size_t, CfgType> >::iterator I = candidates.begin(); I != candidates.end(); ++I) {
    if(this->m_debug) cout << "\t" << I->first;
    CDInfo tmpInfo;

    CfgType lowCfg = rays[I->first].m_incr * 0.0 + I->second;
    bool lowInside = vcm->IsInsideObstacle(lowCfg);
    bool lowValidity = vcm->IsValid(lowCfg, tmpInfo, callee);
    lowValidity = lowValidity && !lowInside;
    if(m_useBBX)
      lowValidity = (lowValidity && env->InBounds(lowCfg, _bb));
    if(this->m_debug) cout << " (lowValidity = " << lowValidity << ")";

    CfgType highCfg = rays[I->first].m_incr * 1.0 + I->second;
    bool highInside = vcm->IsInsideObstacle(highCfg);
    bool highValidity = vcm->IsValid(highCfg, tmpInfo, callee);
    highValidity = highValidity && !highInside;
    if(m_useBBX)
      highValidity = (highValidity && env->InBounds(highCfg, _bb));
    if(this->m_debug) {
      cout << " (highValidity = " << highValidity << ")";
      cout << endl;
    }

    remove.push_back(lowValidity == highValidity);
  }
  if(this->m_debug) {
    cout << "   remove = ";
    copy(remove.begin(), remove.end(), ostream_iterator<bool>(cout, " "));
    cout << endl;
  }
  size_t offset=0;
  for(size_t i=0; i<remove.size(); ++i)
    if(remove[i]) {
      candidates.erase(candidates.begin() + (i-offset));
      offset++;
    }

  if(this->m_debug) {
    cout << "   found " << candidates.size() << " candidates:\n";
    for(typename vector<pair<size_t, CfgType> >::iterator I = candidates.begin(); I != candidates.end(); ++I) {
      cout << "\t" << I->first << ": " << I->second;

      CDInfo tmpInfo;
      bool currInside = vcm->IsInsideObstacle(I->second);
      bool currValidity = vcm->IsValid(I->second, tmpInfo, callee);
      currValidity = currValidity && !currInside;
      if(m_useBBX)
        currValidity = (currValidity && env->InBounds(I->second, _bb));
      cout << " (currValidity = " << currValidity << ")";
      cout << endl;
    }
  }

  if(candidates.size() == 0)
    return false;

  // Binary search on candidates to find the best result at specified resolution
  if(this->m_debug) cout << "DEBUG:: binary searching on candidates to find best result\n";
  double low=0.0, mid=0.5, high=1.0;
  while(candidates.size() > 1 ||
        (candidates.size() == 1 &&
         dm->Distance((rays[candidates.front().first].m_incr * low + candidates.front().second), (rays[candidates.front().first].m_incr * high + candidates.front().second)) > m_approxResolution * env->GetPositionRes())) {
    mid = (low+high)/2.0;
    vector<bool> remove;
    bool foundStateChange = false;
    if(this->m_debug) cout << "   mid = " << mid << "\t";
    for(typename vector<pair<size_t, CfgType> >::iterator I = candidates.begin(); I != candidates.end(); ++I) {
      CfgType middleCfg = rays[I->first].m_incr * mid + I->second;
      CDInfo tmpInfo;
      bool midInside = vcm->IsInsideObstacle(middleCfg);
      bool midValidity = vcm->IsValid(middleCfg, tmpInfo, callee);
      midValidity = midValidity && !midInside;
      if(m_useBBX)
        midValidity = (midValidity && env->InBounds(middleCfg, _bb));
      if(this->m_debug) cout << "(midValidity for direction " << I->first << " = " << midValidity << ", distance = " << dm->Distance((rays[I->first].m_incr * low + I->second), middleCfg) << ") ";

      remove.push_back(midValidity == initValidity);
      if(midValidity != initValidity) // If Validity State has changed
        foundStateChange = true;
    }
    if(this->m_debug) cout << endl;

    // If at least one good candidate is found, remove all known bad ones
    if(foundStateChange) {
      if(this->m_debug) {
        cout << "\tfound state change: remove = ";
        copy(remove.begin(), remove.end(), ostream_iterator<bool>(cout, " "));
        cout << endl;
      }
     size_t offset=0;
      for(size_t i=0; i<remove.size(); ++i) {
        if(remove[i]) {
          candidates.erase(candidates.begin() + (i-offset));
          offset++;
        }
      }
      high=mid;
    }
    else {
      low=mid;
    }
  }
  if(this->m_debug) {
    cout << "DEBUG:: done with binary search\n";
    cout << "   resulting candidate: " << candidates[0].first << ": " << candidates[0].second << endl;
    cout << "   low = " << low << "\thigh = " << high << endl;
  }

  // Finalizing search, keep mid as the low cfg if computing clearance and mid as the high cfg if computing penetration
  if(initValidity) { // Computing clerarance, so low is needed as long as its not the initial cfg
    while(low == 0.0 && high != low) {
      mid=(low+high)/2.0;
      CfgType middleCfg = rays[candidates[0].first].m_incr * mid + candidates[0].second;

      CDInfo tmpInfo;
      bool midInside = vcm->IsInsideObstacle(middleCfg);
      bool midValidity = vcm->IsValid(middleCfg, tmpInfo, callee);
      midValidity = midValidity && !midInside;
      if(m_useBBX)
        midValidity = (midValidity && env->InBounds(middleCfg, _bb));
      if(midValidity != initValidity) // If Validity State has changed
        high=mid;
      else
        low=mid;
    }
    mid=low;
  } else { // Computing penetration, so high is needed and will never be the initial cfg
    mid=high;
  }

  // Set return info
  if(this->m_debug) cout << "DEBUG:: setting info, returning\n";
  _clrCfg = rays[candidates[0].first].m_incr * mid + candidates[0].second;
  _cdInfo.m_minDist = (initValidity ? 1.0 : -1.0) * dm->Distance(_clrCfg, _cfg);
  if(_clrCfg == _cfg) //shouldn't happen, but should return an error
    return false;
  if(env->InBounds(_clrCfg, _bb)) {
    _cfg.m_clearanceInfo = _cdInfo;
    _cfg.m_witnessCfg = shared_ptr<Cfg>(new CfgType(_clrCfg));
    return true;
  } else
    return false;
}

//*********************************************************************//
// Get Roadmap Clearance Statistics                                    //
//   Calculates averages mins and maxes for clearance across roadmaps  //
// paths and edges                                                     //
//*********************************************************************//
template<class MPTraits>
ClearanceStats
ClearanceUtility<MPTraits>::RoadmapClearance(){
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  vector<double> clearanceVec;

  //loop over graph edges and calculate clearance
  for(typename GraphType::edge_iterator it = g->edges_begin(); it != g->edges_end(); it++){
    double currentClearance = MinEdgeClearance(g->GetVertex((*it).source()), g->GetVertex((*it).target()), (*it).property());
    clearanceVec.push_back(currentClearance);//Save this value for variance computation later
  }

  //min, max, avg, variance of graph edge variances
  ClearanceStats stats;
  stats.m_min = *min_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_max = *max_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_avg = accumulate(clearanceVec.begin(), clearanceVec.end(), 0.0) / (double)clearanceVec.size();
  double varSum = 0;
  for(vector<double>::iterator it = clearanceVec.begin(); it != clearanceVec.end(); it++){
    varSum += pow(((*it) - stats.m_avg), 2);
  }
  stats.m_var = varSum / (double)clearanceVec.size();

  return stats;
}

template<class MPTraits>
ClearanceStats
ClearanceUtility<MPTraits>::PathClearance(vector<VID>& _path){
  if(_path.empty())
    return ClearanceStats();

  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);

  typedef typename GraphType::EI EI;
  typedef typename GraphType::VI VI;
  typedef typename GraphType::EID EID;

  vector<double> clearanceVec;
  double pathLength = 0;

  typedef typename vector<VID>::iterator VIT;
  for(VIT vit = _path.begin(); (vit+1)!=_path.end(); ++vit){
    EI ei;
    VI vi;
    EID ed(*vit, *(vit+1));
    g->find_edge(ed, vi, ei);
    CfgRef s = g->GetVertex((*ei).source()), t = g->GetVertex((*ei).target());
    pathLength += dm->Distance(s, t);
    double currentClearance = MinEdgeClearance(s, t, (*ei).property());
    clearanceVec.push_back(currentClearance);
  }

  //min, max, avg, variance of graph edge variances
  ClearanceStats stats;
  stats.m_pathLength = pathLength;
  stats.m_min = *min_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_max = *max_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_avg = accumulate(clearanceVec.begin(), clearanceVec.end(), 0.0) / (double)clearanceVec.size();
  double varSum = 0;
  for(vector<double>::iterator it = clearanceVec.begin(); it != clearanceVec.end(); it++){
    varSum += sqr((*it) - stats.m_avg);
  }
  stats.m_var = varSum / (double)clearanceVec.size();

  return stats;
}

template<class MPTraits>
ClearanceStats
ClearanceUtility<MPTraits>::PathClearance(vector<Cfg>& _path){
  if(_path.empty())
    return ClearanceStats();

  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);

  typedef typename GraphType::EI EI;
  typedef typename GraphType::VI VI;
  typedef typename GraphType::EID EID;
  typedef typename GraphType::const_vertex_iterator CVI;

  double pathLength = 0;
  vector<double> clearanceVec;

  typedef typename vector<Cfg>::iterator CIT;
  for(CIT cit = _path.begin(); (cit+1)!=_path.end(); ++cit){
    pathLength += dm->Distance(*cit, *(cit+1));

    WeightType weight;
    //weight.SetLPLabel();
    bool source_found = false;
    CVI si;
    for(si = g->begin(); si != g->end(); ++si)
      if(si->property() == *cit) {
        source_found = true;
        break;
      }
    if(source_found) {
      bool target_found = false;
      CVI ti;
      for(ti = g->begin(); ti != g->end(); ++ti)
        if(ti->property() == *(cit+1)) {
          target_found = true;
          break;
        }
      if(target_found) {
        VI vi;
        EI ei;
        EID ed(VID(si->descriptor()), VID(ti->descriptor()));
        if(g->find_edge(ed, vi, ei))
          weight = (*ei).property();
      }
    }
    double currentClearance = MinEdgeClearance(*cit, *(cit+1), weight);
    clearanceVec.push_back(currentClearance);
  }

  //min, max, avg, variance of graph edge variances
  ClearanceStats stats;
  stats.m_pathLength = pathLength;
  stats.m_min = *min_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_max = *max_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_avg = accumulate(clearanceVec.begin(), clearanceVec.end(), 0.0) / (double)clearanceVec.size();
  double varSum = 0;
  for(vector<double>::iterator it = clearanceVec.begin(); it != clearanceVec.end(); it++){
    varSum += sqr((*it) - stats.m_avg);
  }
  stats.m_var = varSum / (double)clearanceVec.size();

  return stats;
}


template<class MPTraits>
double
ClearanceUtility<MPTraits>::MinEdgeClearance(const CfgType& _c1, const CfgType& _c2, const WeightType& _weight){
  if(_weight.HasClearance()){
    return _weight.GetClearance();
  }
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  double minClearance = 1e6;
  //Reconstruct the path given the two nodes
  vector<CfgType> intermediates = _weight.GetIntermediates();
  vector<CfgType> reconEdge;
  typedef typename vector<CfgType>::iterator CIT;
  if(_weight.GetLPLabel() != "RRTExpand"){
    reconEdge = this->GetMPProblem()->GetLocalPlanner(_weight.GetLPLabel())->
      ReconstructPath(_c1, _c2, intermediates, env->GetPositionRes(), env->GetOrientationRes());
  }
  else{
    StraightLine<MPTraits> sl;
    sl.SetMPProblem(this->GetMPProblem());
    intermediates.insert(intermediates.begin(), _c1);
    intermediates.push_back(_c2);
    for(CIT cit = intermediates.begin(); (cit+1)!=intermediates.end(); ++cit){
      StatClass dummyStats;
      LPOutput<MPTraits> lpOutput;
      CfgType col;
      vector<CfgType> edge = sl.ReconstructPath(*cit, *(cit+1), intermediates, env->GetPositionRes(), env->GetOrientationRes());
      reconEdge.insert(reconEdge.end(), edge.begin(), edge.end());
    }
  }
  reconEdge.insert(reconEdge.begin(), _c1);
  reconEdge.push_back(_c2);
  for(CIT it = reconEdge.begin(); it != reconEdge.end(); ++it){
    CDInfo collInfo;
    CfgType clrCfg;
    //Decide which collision info function to use
    CollisionInfo(*it, clrCfg, env->GetBoundary(), collInfo);
    double currentClearance = collInfo.m_minDist;
    //If newly computed clearance is less than the previous minimum, it becomes the minimum
    if(currentClearance < minClearance){
      minClearance = currentClearance;
    }
  }
  return minClearance;
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Medial Axis Utility
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template<class MPTraits>
MedialAxisUtility<MPTraits>::MedialAxisUtility(MPProblemType* _problem,
    string _vcLabel, string _dmLabel,
    bool _exactClearance, bool _exactPenetration,
    size_t _clearanceRays, size_t _penetrationRays,
    bool _useBBX, bool _positional, bool _debug,
    double _epsilon, size_t _historyLength) :
  ClearanceUtility<MPTraits>(_problem, _vcLabel, _dmLabel,
      _exactClearance, _exactPenetration, _clearanceRays, _penetrationRays,
      _useBBX, _positional, _debug),
  m_epsilon(_epsilon), m_historyLength(_historyLength){
    this->m_name = "MedialAxisUtility";
  }

template<class MPTraits>
MedialAxisUtility<MPTraits>::MedialAxisUtility(MPProblemType* _problem, XMLNodeReader& _node):
  ClearanceUtility<MPTraits>(_problem, _node){
    this->m_name = "MedialAxisUtility";
    ParseXML(_node);
  }

template<class MPTraits>
void
MedialAxisUtility<MPTraits>::ParseXML(XMLNodeReader& _node){
  m_epsilon = _node.numberXMLParameter("epsilon", false, 0.1, 0.0, 1.0, "Epsilon-Close to the MA (fraction of the resolution)");
  m_historyLength = _node.numberXMLParameter("historyLength", false, 5, 3, 100,"History Length");
}

template<class MPTraits>
void
MedialAxisUtility<MPTraits>::Print(ostream& _os) const {
  ClearanceUtility<MPTraits>::Print(_os);
  _os << "\tepsilon::" << m_epsilon << endl;
  _os << "\thistory length::" << m_historyLength << endl;
}

//***********************************//
// Main Push To Medial Axis Function //
//***********************************//
template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::PushToMedialAxis(CfgType& _cfg, shared_ptr<Boundary> _bb){
  // Initialization
  string callee = this->GetNameAndLabel() + "::PushToMedialAxis";
  if(this->m_debug)
    cout << endl << callee << endl << "Being Pushed: " << _cfg;

  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // If invalid, push to the outside of the obstacle
  bool inside = vcm->IsInsideObstacle(_cfg);
  bool inCollision = !(vcm->IsValid(_cfg, tmpInfo, callee));

  if(this->m_debug) cout << " Inside/In-Collision: " << inside << "/" << inCollision << endl;

  if(inside || inCollision){
    if(!PushFromInsideObstacle(_cfg, _bb))
      return false;
  }

  // Cfg is free, find medial axis
  if(!PushCfgToMedialAxis(_cfg, _bb)) {
    if(this->m_debug) cout << "Not found!! ERR in pushtomedialaxis" << endl;
    return false;
  }

  if(this->m_debug) cout << "FINAL CfgType: " << _cfg << endl << callee << "::END" << endl << endl;

  return true;
}

//***************************************************************//
// Push From Inside Obstacle                                     //
//   In this function, a cfg is known to be inside an obstacle.  //
// A direction is determined to move the cfg outside of the      //
// obstacle and is pushed till outside.                          //
//***************************************************************//
template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::PushFromInsideObstacle(CfgType& _cfg, shared_ptr<Boundary> _bb){

  // Initialization
  string callee = this->GetNameAndLabel() + "::PushFromInsideObstacle";
  if(this->m_debug) cout << callee << endl << " CfgType: " << _cfg << endl;

  // Variables
  CfgType transCfg = _cfg;

  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  //Determine direction to move
  if(this->CollisionInfo(_cfg, transCfg, _bb, tmpInfo)) {
    if(this->m_debug) cout << "Clearance Cfg: " << transCfg << endl;
  }
  else
    return false;

  //bad collision information call might return the current Cfg
  //as the clearance Cfg.
  if(transCfg == _cfg)
    return false;

  _cfg = transCfg;

  if(this->m_debug) cout << "FINAL CfgType: " << _cfg << endl << callee << "::END " << endl;

  return true;
}

//***************************************************************//
// Push Cfg To Medial Axis                                       //
//   This function is to perform a regular push to medial axis   //
// algorithm stepping out at the resolution till the medial axis //
// is found, determined by the clearance.                        //
//***************************************************************//
template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::PushCfgToMedialAxis(CfgType& _cfg, shared_ptr<Boundary> _bb){
  // Initialization
  string callee = this->GetNameAndLabel() + "::PushCfgToMedialAxis";
  if(this->m_debug) cout << callee << endl << "Cfg: " << _cfg << " eps: " << m_epsilon << endl;

  Environment* env = this->GetMPProblem()->GetEnvironment();
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(this->m_vcLabel);
  shared_ptr<MultiBody> robot = env->GetMultiBody(_cfg.GetRobotIndex());

  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // Should already be in free space
  if(vcm->IsInsideObstacle(_cfg))
    return false;

  // Determine positional direction to move
  CfgType transCfg;
  CDInfo prevInfo;
  if(!FindInitialDirection(_cfg, _bb, env->GetPositionRes(), transCfg, prevInfo))
    return false;

  CfgType startCfg, endingCfg;
  double upperBound = 0, lowerBound = 0, stepSize = 0;
  bool borderFound;
  if(this->m_exactClearance)
    borderFound = FindMedialAxisBorderExact(_cfg, _bb, transCfg, prevInfo, startCfg, endingCfg, upperBound, lowerBound, stepSize);
  else
    borderFound = FindMedialAxisBorderApprox(_cfg, _bb, transCfg, prevInfo, startCfg, endingCfg, upperBound, lowerBound, stepSize);
  if(!borderFound)
    return false;

  double middle = (lowerBound+upperBound)/2.0;
  if(this->m_debug) cout << "Lower and upper bounds: " << lowerBound << "/" << middle << "/" << upperBound  << endl;
  CfgType midMCfg = transCfg*middle + _cfg;

  if(this->m_debug) {
    VDClearComments();
    cout << "start/mid/end: " << endl << startCfg << endl << midMCfg << endl << endingCfg << endl;
  }

  vector<double> dists(5, 0);
  CfgType tmpTransCfg;
  bool passed = this->CollisionInfo(startCfg, tmpTransCfg, _bb, tmpInfo);
  if(!passed) return false;
  dists[0] = tmpInfo.m_minDist;

  passed = this->CollisionInfo(midMCfg, tmpTransCfg, _bb, tmpInfo);
  if(!passed) return false;
  dists[2] = tmpInfo.m_minDist;

  passed = this->CollisionInfo(endingCfg, tmpTransCfg, _bb, tmpInfo);
  if(!passed) return false;
  dists[4] = tmpInfo.m_minDist;

  double middleS = (lowerBound+middle)/2.0;
  CfgType midSCfg = transCfg*middleS + _cfg;

  double middleE = (middle+upperBound)/2.0;
  CfgType midECfg = transCfg*middleE + _cfg;

  if(this->m_debug) {
    cout << "dists: ";
    for(size_t i=0; i<size_t(dists.size()); i++)
      cout << dists[i] << " ";
    cout << endl;
  }

  if(this->m_debug) cout << "start/mids/end: " << endl << startCfg << endl << midSCfg
    << endl << midMCfg << endl << midECfg << endl << endingCfg << endl;

  midMCfg = BinarySearchForPeaks(startCfg, midMCfg, endingCfg, lowerBound, upperBound, transCfg, _cfg, _bb, dists);

  if(midMCfg == CfgType())
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

  if(this->m_debug) cout << "FINAL Cfg: " << _cfg << " steps: " << stepSize << endl;

  return true;
}

template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::FindInitialDirection(
    CfgType& _cfg, shared_ptr<Boundary> _bb,
    double _posRes, CfgType& _transCfg, CDInfo& _prevInfo) {
  _prevInfo.ResetVars();
  _prevInfo.m_retAllInfo = true;

  if(this->CollisionInfo(_cfg, _transCfg, _bb, _prevInfo)) {
    //if(_prevInfo.m_minDist < this->GetMPProblem()->GetEnvironment()->GetPositionRes()) {
    if(_prevInfo.m_minDist < numeric_limits<float>::epsilon()) {
      if(this->m_debug) cout << "Start Cfg adjacent to obstacle" << endl;
      return false;
    }

    _transCfg = _cfg - _transCfg;
    double magnitude = 0;
    for(size_t i = 0; i < _transCfg.DOF(); ++i){
      magnitude += _transCfg[i] * _transCfg[i];
    }
    magnitude = sqrt(magnitude);

    for(size_t i = 0; i<_transCfg.DOF(); ++i){
      _transCfg[i] *= _posRes/magnitude;
      if(i > _transCfg.PosDOF() && _transCfg[i] > 0.5){
        _transCfg[i]--;
      }
    }
    if(this->m_debug) cout << "TRANS CFG: " << _transCfg << endl;

    return true;
  }
  else{
    return false;
  }
}


template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::FindMedialAxisBorderExact(
    const CfgType& _cfg, shared_ptr<Boundary> _bb,
    CfgType& _transCfg, CDInfo& _prevInfo,
    CfgType& _startCfg, CfgType& _endingCfg,
    double& _upperBound, double& _lowerBound, double& _stepSize) {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  shared_ptr<MultiBody> robot = env->GetMultiBody(_cfg.GetRobotIndex());
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // Determine gap for medial axis
  CfgType tmpCfg = _cfg;
  CfgType heldCfg;
  _stepSize = 0.0;
  bool witnessPointMoved = false;

  while(!witnessPointMoved) {
    // Increment
    heldCfg = tmpCfg;
    tmpCfg = _transCfg*_stepSize + _cfg;

    // Test for in BBX and inside obstacle
    bool inside = false;//vcm->IsInsideObstacle(tmpCfg);
    bool inBBX = env->InBounds(tmpCfg, _bb);
    if(this->m_debug) {
      VDAddTempCfg(tmpCfg, (inside || !inBBX));
      VDClearLastTemp();
    }

    // If inside obstacle or out of the bbx, step back
    if(inside || !inBBX) {
      if(!inBBX && !this->m_useBBX){
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
      if(this->m_debug) cout << "TMP Cfg: " << tmpCfg;
      CfgType tmpTransCfg;
      if(this->CollisionInfo(tmpCfg, tmpTransCfg, _bb, tmpInfo)) {
        if(!tmpCfg.GetLabel("VALID")) return false;
        Vector3d transDir = tmpInfo.m_objectPoint - _prevInfo.m_objectPoint;
        if(this->m_debug) cout << " obst: " << tmpInfo.m_nearestObstIndex;
        double tmpDist = 0.0;
        for(size_t i=0; i<_transCfg.PosDOF(); i++)
          tmpDist += transDir[i]*transDir[i];
        tmpDist = sqrt(tmpDist);
        if(tmpDist > robot->GetBoundingSphereRadius() * (2.0 + numeric_limits<float>::epsilon()) ) { // TODO: might be better value
          if(this->m_debug) cout << "\n WP moved: " << tmpDist;
          witnessPointMoved=true;
        }
        _prevInfo = tmpInfo;
        if(this->m_debug) cout << " clearance: " << tmpInfo.m_minDist << endl;
      }
      else { // Couldn't calculate clearance info
        if(this->m_debug) cout << "BROKE!" << endl;
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
  }

  if(this->m_debug) cout << "\nCalculating Mid... stepSize/cbStepSize: " << _stepSize << "/" << "0" << endl;
  _upperBound = _stepSize;
  _lowerBound = _upperBound-1.0;

  return true;
}

template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::FindMedialAxisBorderApprox(
    const CfgType& _cfg, shared_ptr<Boundary> _bb,
    CfgType& _transCfg, CDInfo& _prevInfo,
    CfgType& _startCfg, CfgType& _endingCfg,
    double& _upperBound, double& _lowerBound, double& _stepSize) {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  //shared_ptr<MultiBody> robot = env->GetMultiBody(_cfg->GetRobotIndex());
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // Determine gap for medial axis
  CfgType tmpCfg = _cfg;
  CfgType heldCfg;
  _stepSize = 0.0;
  double cbStepSize = 0.0;
  deque<double> segDists;
  deque<CfgType> segCfgs;
  bool peakFound = false, checkBack = false, fellOut = false;
  double errEps = numeric_limits<double>::epsilon(); // Arbitrary Value for errEps

  while(!peakFound) {

    // Increment
    heldCfg = tmpCfg;
    tmpCfg = _transCfg*(checkBack ? cbStepSize : _stepSize) + _cfg;

    // Test for in BBX and inside obstacle
    bool inside = vcm->IsInsideObstacle(tmpCfg);
    bool inBBX = env->InBounds(tmpCfg, _bb);
    if(this->m_debug) {
      VDAddTempCfg(tmpCfg, (inside || !inBBX));
      VDClearLastTemp();
    }

    // If inside obstacle or out of the bbx, step back
    if(inside || !inBBX) {
      if(!inBBX && !this->m_useBBX){
        return false;
      }
      if(segCfgs.size() > 0) {
        fellOut = true;
        break;
      }
      else{
        return false;
      }
    }
    else {
      tmpInfo.ResetVars();
      tmpInfo.m_retAllInfo = true;

      // If tmp is valid, move on to next step
      if(this->m_debug) cout << "TMP Cfg: " << tmpCfg;
      CfgType tmpTransCfg;
      if(this->CollisionInfo(tmpCfg, tmpTransCfg, _bb, tmpInfo)) {
        _prevInfo = tmpInfo;
        if(this->m_debug) cout << " clearance: " << tmpInfo.m_minDist;

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
          checkBack=false;
        }
      }
      else { // Couldn't calculate clearance info
        if(this->m_debug) cout << "BROKE!" << endl;
        return false;
      }

      // enumerate deltas to find peak
      int posCnt=0, negCnt=0, zroCnt=0;
      for(deque<double>::iterator dit = segDists.begin(); dit+1 != segDists.end(); ++dit) {
        double distDelta = *(dit+1) - (*dit);
        if(distDelta > errEps)
          ++posCnt;
        else if(distDelta < -errEps)
          ++negCnt;
        else
          ++zroCnt;
      }
      if(this->m_debug) cout << "  Pos/Zero/Neg counts: " << posCnt << "/" << zroCnt << "/" << negCnt << endl;
      if((negCnt > 0 && negCnt > posCnt) || (zroCnt > 0 && zroCnt > posCnt)) {
        if(posCnt < 1 && (zroCnt < 1 || negCnt < 1)){ // Only see negatives or zeros, check back
          --cbStepSize;
          --_stepSize;
          checkBack = true;
        }
        else {
          peakFound = true;
          if(this->m_debug) {
            cout << "Found peak!  Dists: ";
            for(size_t i=0; i<size_t(segDists.size()); i++)
              cout << segDists[i] << " ";
            cout << endl;
          }
        }
      }
      else { // No peak found
        checkBack = false;
      }
      if(this->m_debug) cout << endl;

      // Increment step size
      if(!peakFound)
        _stepSize++;
    }
  }

  // Check if there are enough cfgs
  if(segCfgs.size() < 2)
    return false;

  // Setup start, middle and end CfgTypes
  _startCfg = segCfgs[0];
  _endingCfg = segCfgs[segCfgs.size()-1];
  if(this->m_debug) {
    VDComment("Binary Cfgs: ");
    VDAddTempCfg(_startCfg, true);
    VDAddTempCfg(_endingCfg, true);
  }

  if(this->m_debug) {
    cout << "\nCalculating Mid... _stepSize/cbStepSize: " << _stepSize << "/" << cbStepSize << endl;
    if(fellOut) cout << "\n\nFellOut\n\n";
  }
  _upperBound = fellOut ? _stepSize-1.0 : _stepSize;
  if(fellOut) {
    _lowerBound = _upperBound-1.0;
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
MedialAxisUtility<MPTraits>::BinarySearchForPeaks(
    CfgType& _startCfg,
    CfgType& _midMCfg,
    CfgType& _endingCfg,
    double _lowerBound, double _upperBound,
    CfgType& _transCfg, CfgType& _cfg,
    shared_ptr<Boundary> _bb, vector<double>& _dists) {
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);

  // Variables for modified binary search
  size_t attempts = 0, maxAttempts = 20, badPeaks = 0, maxBadPeaks = 10;
  vector<double> deltas(4, 0), distsFromMax(5, 0);
  bool peaked = false;
  double errEps = numeric_limits<double>::epsilon(); // Arbitrary Value for errEps

  do{ // Modified Binary search to find peaks
    if(this->m_debug) {
      VDAddTempCfg(_midMCfg, true);
      VDClearLastTemp();
    }

    // Update Cfgs
    attempts++;
    double middle = (_lowerBound + _upperBound)/2.0;
    double middleS = (_lowerBound + middle)/2.0;
    double middleE = (middle + _upperBound)/2.0;
    if(this->m_debug) cout << "Bounds: " << _lowerBound << "/" << middleS << "/" << middle << "/" << middleE << "/" << _upperBound << endl;
    CfgType midSCfg = _transCfg*middleS + _cfg;
    CfgType midECfg = _transCfg*middleE + _cfg;

    CfgType tmpTransCfg;
    CDInfo tmpInfo;
    bool passed = this->CollisionInfo(midSCfg, tmpTransCfg, _bb, tmpInfo);
    if(!passed) return CfgType();
    _dists[1] = tmpInfo.m_minDist;

    passed = this->CollisionInfo(midECfg, tmpTransCfg, _bb, tmpInfo);
    if(!passed) return CfgType();
    _dists[3] = tmpInfo.m_minDist;

    // Compute Deltas and Max Distance
    deltas.clear();
    distsFromMax.clear();
    double maxDist = _dists[0];
    if(this->m_debug) cout << "Deltas: ";
    typedef vector<double>::iterator DIT;
    for(DIT dit = _dists.begin(); dit+1 != _dists.end(); ++dit) {
      deltas.push_back(*(dit+1) - *dit);
      if(this->m_debug) cout << " " << deltas.back();
      maxDist = max(maxDist, *(dit+1));
    }
    for(DIT dit = _dists.begin(); dit!=_dists.end(); ++dit)
      distsFromMax.push_back(maxDist - *dit);

    if(this->m_debug) cout << endl;

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
    else if(deltas[1] > errEps && deltas[2] <= errEps && distsFromMax[2] < errEps) {
      peak = 2;
      _startCfg = midSCfg;
      _endingCfg = midECfg;
      _dists[0] = _dists[1];
      _dists[4] = _dists[3];
      _lowerBound = middleS;
      _upperBound = middleE;
    }
    else if(deltas[2] > errEps && deltas[3] <= errEps && distsFromMax[3] < errEps) {
      peak = 3;
      _startCfg = _midMCfg;
      _midMCfg = midECfg;
      _dists[0] = _dists[2];
      _dists[2] = _dists[3];
      _lowerBound = middle;
    }
    else {
      if(deltas[0] > errEps && deltas[1] > errEps && deltas[2] > errEps && deltas[3] > errEps) {
        peak = 3;
        _startCfg = _midMCfg;
        _midMCfg = midECfg;
        _dists[0] = _dists[2];
        _dists[2] = _dists[3];
        _lowerBound = middle;
      }
      else if(deltas[0] < -errEps && deltas[1] < -errEps && deltas[2] < -errEps && deltas[3] < -errEps) {
        peak = 1;
        _endingCfg = _midMCfg;
        _midMCfg = midSCfg;
        _dists[4] = _dists[2];
        _dists[2] = _dists[1];
        _upperBound = middle;
      }
      else {
        // No peak found, recalculate old, mid and new clearance
        badPeaks++;
        peak = -1;

        if(this->CollisionInfo(_startCfg, tmpTransCfg, _bb, tmpInfo)
            && _dists[0] >= tmpInfo.m_minDist)
          _dists[0] = tmpInfo.m_minDist;

        if(this->CollisionInfo(_midMCfg, tmpTransCfg, _bb, tmpInfo)
            && _dists[2] >= tmpInfo.m_minDist)
          _dists[2] = tmpInfo.m_minDist;

        if(this->CollisionInfo(_endingCfg, tmpTransCfg, _bb, tmpInfo)
            && _dists[4] >= tmpInfo.m_minDist)
          _dists[4] = tmpInfo.m_minDist;
      }
    }
    if(this->m_debug) cout << " peak: " << peak << "  cfg: " << _midMCfg;
    double gapDist = dm->Distance(_startCfg, _endingCfg);
    if(this->m_debug) cout << " gap: " << gapDist << endl;
    if(m_epsilon >= gapDist)
      peaked = true;

  } while(!peaked && attempts < maxAttempts && badPeaks < maxBadPeaks);

  return _midMCfg;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Surface Medial Axis Utility
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifdef PMPCfgSurface

template<class MPTraits>
SurfaceMedialAxisUtility<MPTraits>::SurfaceMedialAxisUtility(MPProblemType* _problem,
    string _vcLabel, string _dmLabel) :
  MedialAxisUtility<MPTraits>(_problem, _vcLabel, _dmLabel){
    this->m_name = "SurfaceMedialAxisUtility";
  }

//the square of the distance from pos to p1p2
inline double distsqr
(const Point2d& _pos, const Point2d& _p1, const Point2d& _p2, Point2d& _cdPt) {
  Vector2d n=_p1-_p2;
  double t=(n*(_pos-_p1))/(n*(_p2-_p1));
  if( t>=0 && t<=1 ){
    for(int i=0;i<2;i++) _cdPt[i]=(1-t)*_p1[i]+t*_p2[i];
    return (_pos-_cdPt).normsqr();
  }
  else{ //closest point is end pt
    double d1=(_p1-_pos).normsqr();
    double d2=(_p2-_pos).normsqr();
    if( d1<d2 ){ _cdPt=_p1; return d1; }
    else { _cdPt=_p2; return d2; }
  }
}

//get clearance of the point
template<class MPTraits>
double
SurfaceMedialAxisUtility<MPTraits>::GetClearance2DSurf
(shared_ptr<MultiBody> _mb, const Point2d& _pos, Point2d& _cdPt, double _clear)
{
  double minDis=1e10;
  if(this->m_debug) cout << " GetClearance2DSurf (start call) (mb,pos,cdPt, clear)" << endl;
  if(this->m_debug) cout << " getting world transformation. num fixed bodies: " << _mb->GetFixedBodyCount() << endl;
  Transformation& trans = _mb->GetFixedBody(0)->WorldTransformation();
  //check roughly <- this optimization should be added!!!
  Vector3d mbCenter = _mb->GetFixedBody(0)->GetCenterOfMass();
  Point2d tPt(mbCenter[0],mbCenter[2]);
  double* bbx = _mb->GetFixedBody(0)->GetBoundingBox();
  double rad2d = sqrt(pow(bbx[1]-bbx[0],2.0) + pow(bbx[5]-bbx[4],2.0));
  double diffWRadius = (_pos-tPt).norm()-rad2d;
  if( diffWRadius>_clear) return minDis;

  GMSPolyhedron& gmsPoly = _mb->GetFixedBody(0)->GetWorldPolyhedron();
  vector<Vector3d>& Geo=gmsPoly.GetVertexList();
  vector< pair<int,int> >& boundaryLines=gmsPoly.GetBoundaryLines();
  if(this->m_debug) cout << " boundary lines size: " << boundaryLines.size() << " transformation: " << trans << endl;
  for(int i=0; i<(int)boundaryLines.size(); i++) {
    int id1 = boundaryLines[i].first;
    int id2 = boundaryLines[i].second;
    Vector3d v1 = Geo[id1];
    Vector3d v2 = Geo[id2];
    Point2d p1(v1[0],v1[2]);
    Point2d p2(v2[0],v2[2]);
    Point2d c;
    double dist=distsqr(_pos,p1,p2,c);
    if( dist<minDis) { minDis=dist; _cdPt=c;}
  }//endfor i

  double clearDist = sqrt(minDis);
  if(this->m_debug) cout << " computed clearance: " << clearDist << endl;
  return clearDist;

}

//get clearance of the point
template<class MPTraits>
double
SurfaceMedialAxisUtility<MPTraits>::GetClearance2DSurf
(Environment* _env, const Point2d& _pos, Point2d& _cdPt) {
  if(this->m_debug) cout << "MedialAxisUtility::GetClearance2DSurf" <<endl;
  if(this->m_debug) cout << "num multibodies: " << _env->GetUsableMultiBodyCount() << endl;

  double minDist=_env->GetBoundary()->GetClearance2DSurf(_pos,_cdPt);

  for(int i=1; i<(int)_env->GetUsableMultiBodyCount(); i++) { //skip 0 (assume it's robot)
    shared_ptr<MultiBody> mb = _env->GetMultiBody(i);
    //find clearance
    Point2d c;
    double dist = GetClearance2DSurf(mb,_pos,c,minDist);
    if( dist<minDist ){ minDist=dist; _cdPt=c; }
    if(this->m_debug) cout << " getting mb: " << i;
    if(this->m_debug) cout << " GetClearance2DSurf (mb,_pos,c,minDist) minDist: " << minDist << endl;
  }//end for i
  return minDist;
}

template<class MPTraits>
double
SurfaceMedialAxisUtility<MPTraits>::PushCfgToMedialAxis2DSurf
(CfgType& _cfg, shared_ptr<Boundary> _bb, bool& _valid) {

  string callee = this->GetNameAndLabel() + "::PushCfgToMedialAxis2DSurf";

  if(this->m_debug) cout << callee << endl << "Cfg: " << _cfg  << endl;

  Environment* env = this->GetMPProblem()->GetEnvironment();
  ////////////////////////////////////////////
  Point2d closest;
  Point2d pos=_cfg.GetPos();
  if(this->m_debug) cout << " Calling GetClearance2DSurf(env,pos,closest)" << endl;
  double clear=this->GetClearance2DSurf(env,pos,closest);
  Vector2d dir=(pos-closest).normalize()*0.5;
  Point2d newClosest=closest;
  int iter=0;
  int maxIter=1000;
  do{
    closest=newClosest;
    pos=pos+dir;
    if(this->m_debug) cout << " Calling GetClearance2DSurf(env,pos,closest) new pos: " << pos << endl;
    clear=this->GetClearance2DSurf(env,pos,newClosest);
    if(this->m_debug) cout << " computed clearance: " << clear << " iteration: " << iter << endl;
    iter++;
    if(iter >= maxIter ) break;
  }while( (newClosest-closest).normsqr()<0.01 );
  _cfg.SetPos(pos);

  return clear;
}

#endif


#endif

#ifndef PMPL_SAFE_INTERVAL_TOOL_H_
#define PMPL_SAFE_INTERVAL_TOOL_H_

#include <vector>

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/XMLNode.h"
#include "MPProblem/DynamicObstacle.h"
#include "MPLibrary/Conflict.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"


////////////////////////////////////////////////////////////////////////////////
/// Computes safe intervals for Cfgs and Edges. A 'safe interval' is an interval
/// of time where collision with a known dynamic obstacle (with known
/// trajectory) will not occur.
///
/// This tool implements the concept of a 'Safe Intervals' from the paper:
///
/// Phillips, Mike, and Maxim Likhachev. "Sipp: Safe interval path planning for
/// dynamic environments." Robotics and Automation (ICRA), 2011 IEEE International
/// Conference on. IEEE, 2011.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class SafeIntervalTool final : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{
  	typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::CfgType    CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::Path Path; 

    ///@}
    ///@name Local Types
    ///@{

    typedef std::vector<Range<double>> Intervals; ///< A set of time intervals.

    ///@}
    ///@name Construction
    ///@{

    SafeIntervalTool();

    SafeIntervalTool(XMLNode& _node);

    virtual ~SafeIntervalTool();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Interval Computation
    ///@{

    /// Compute the safe intervals for a given Cfg.
    /// @param _cfg The configuration to compute safeIntervals's for.
    /// @return The set of safe intervals for _cfg.
    Intervals ComputeIntervals(const CfgType& _cfg);

    /// Compute the safe intervals for a given Edge.
    /// @param _weight The edge to compute safeIntervals's for.
    /// @return The set of safe intervals for _weight.
    Intervals ComputeIntervals(const WeightType& _weight);

    ///@}
    ///@name Interval Checking
    ///@{

    /// Determine if a timestep is contained within a SafeInterval
    /// @param _intervals The safe intervals to check in.
    /// @param _timestep The timestep to check.
    bool ContainsTimestep(const Intervals& _intervals, const double _timestep);

    //Conflict<MPTraits> FindConflictType(Path* _path, Robot*& _conflictRobot1,
    // Robot*& _conflictRobot2, double& _conflictTimestep, 
    // vector<CfgType> _obstacle);    

 
    //Conflict<MPTraits> FindConflict(Path* _path, Path* _obstacle);

    /// Checking if an edge is collision-free with an external cfg
    /// @param _source The cfg source of the edge.
    /// @param _target The cfg target of the edge.
    /// @param _timestep The external cfg to check.
    bool IsEdgeSafe(const CfgType _source, const CfgType _target, 
    	const CfgType _conflictCfg);

    /// Checking if a set of paths has inter-robot collisions.
    /// @param _paths The set f paths to check.
    pair<pair<size_t,pair<CfgType,double>>,pair<size_t,pair<CfgType,
    	double>>> FindConflict(vector<Path*> _paths) ;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Determine if a configuration is safe at the given time step.
    /// @param _cfg The configuration to check.
    /// @param _timestep The timestep for the dynamic obstacles.
    bool IsSafe(const CfgType& _cfg, const double _timestep);

    /// Computes the safe interval(s) for a set of configurations.
    /// @param _cfgs The cfgs to compute the safeIntervals for. These are
    ///              assumed to be a sequence which will be followed by the robot
    ///              at a rate of one cfg per time resolution.
    /// @return The set of time intervals for which it is safe to start
    ///         following the configuration sequence.
    Intervals ComputeSafeIntervals(const std::vector<Cfg>& _cfgs);

    //bool IsSafe(const CfgType& _cfg, const double _timestep, 
    //	Robot*& _conflictRobot2, vector<CfgType> _obstacle, 
    //	CfgType& _conflictCfg);

    //bool FindConflictCfg(const std::vector<Cfg>& _cfgs, 
    // CfgType& _conflictCfg,
    // Robot*& _conflictRobot1, Robot*& _conflictRobot2, 
    // double& _conflictTimestep, vector<CfgType> _obstacle);

 
    ///@}
    ///@name Internal State
    ///@{

    std::string m_vcLabel; ///< The validity checker to use.

    /// A cache of computed safe intervals for roadmap configurations.
    std::unordered_map<const CfgType*, Intervals> m_cfgIntervals;

    /// A cache of computed safe intervals for roadmap edges.
    std::unordered_map<const WeightType*, Intervals> m_edgeIntervals;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
SafeIntervalTool<MPTraits>::
SafeIntervalTool() : MPBaseObject<MPTraits>() {
  this->SetName("SafeIntervalTool");
}


template <typename MPTraits>
SafeIntervalTool<MPTraits>::
SafeIntervalTool(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  this->SetName("SafeIntervalTool");

  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
}


template <typename MPTraits>
SafeIntervalTool<MPTraits>::
~SafeIntervalTool() = default;

/*-------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
SafeIntervalTool<MPTraits>::
Initialize() {
  // Clear out the interval caches.
  m_cfgIntervals.clear();
  m_edgeIntervals.clear();
}

/*-------------------------- Intervals Computations ---------------------------*/

template <typename MPTraits>
typename SafeIntervalTool<MPTraits>::Intervals
SafeIntervalTool<MPTraits>::
ComputeIntervals(const CfgType& _cfg) {
  if(m_cfgIntervals[&_cfg].empty()) {
    const std::vector<CfgType> cfg{_cfg};
    m_cfgIntervals[&_cfg] = ComputeSafeIntervals(cfg);
  }

  return m_cfgIntervals[&_cfg];
}


template <typename MPTraits>
typename SafeIntervalTool<MPTraits>::Intervals
SafeIntervalTool<MPTraits>::
ComputeIntervals(const WeightType& _weight) {
  if(m_edgeIntervals[&_weight].empty()) {
    const vector<CfgType>& cfgs = _weight.GetIntermediates();
    m_edgeIntervals[&_weight] = ComputeSafeIntervals(cfgs);
  }

  return m_edgeIntervals[&_weight];
}


template <typename MPTraits>
bool
SafeIntervalTool<MPTraits>::
ContainsTimestep(const Intervals& _intervals, const double _timestep) {
  for(auto& range : _intervals) {
    if(range.Contains(_timestep))
      return true;
  }
  return false;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
SafeIntervalTool<MPTraits>::
IsSafe(const CfgType& _cfg, const double _timestep) {
  // Configure _cfg's robot at _cfg.
  auto robotMultiBody = _cfg.GetRobot()->GetMultiBody();
  robotMultiBody->Configure(_cfg);

  // Get the valididty checker and make sure it has type
  // CollisionDetectionValidity.
  /// @TODO Figure out how to avoid needing this downcast so that we can
  ///       leverage more efficient compose checks (like checking the bounding
  ///       spheres first).
  auto basevc = this->GetValidityChecker(m_vcLabel).get();
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc);

  // Compute the step number associated with _timestep.
  const double timeRes = this->GetEnvironment()->GetTimeRes();
  const size_t currentStep = std::lround(_timestep / timeRes);

  // Check this configuration against each dynamic obstacle.
  const auto& obstacles = this->GetMPProblem()->GetDynamicObstacles();
  for(const auto& obstacle : obstacles) {
    // Determine the obstacle's position at the current timestep. If it is
    // already done moving, use its last position.
    const auto& path = obstacle.GetPath();
    const size_t lastStep = path.size(),
                 useStep  = std::min(currentStep, lastStep -1);
    const CfgType& position = path[useStep];

    // Configure the obstacle at the current timestep.
    auto obstacleMultiBody = obstacle.GetRobot()->GetMultiBody();
    obstacleMultiBody->Configure(position);

    // If the obstacle is in collision with _cfg at _timestep, return false
    CDInfo cdInfo;
    if(vc->IsMultiBodyCollision(cdInfo, obstacleMultiBody, robotMultiBody,
        this->GetNameAndLabel())) {
      return false;
    }
  }

  // If we haven't detected a collision, the configuration is safe.
  return true;
}


template <typename MPTraits>
typename SafeIntervalTool<MPTraits>::Intervals
SafeIntervalTool<MPTraits>::
ComputeSafeIntervals(const std::vector<Cfg>& _cfgs) {
  MethodTimer mt(this->GetStatClass(), "SafeIntervalTool::ComputeSafeIntervals");

  // If there are no dynamic obstacles, the safe interval is infinite.
  const auto& obstacles = this->GetMPProblem()->GetDynamicObstacles();
  if(obstacles.empty())
    return Intervals{Range<double>(0, std::numeric_limits<double>::max())};

  // Find the latest timestep in which a dynamic obstacle is still moving.
  size_t timeFinal = 0;
  for(auto& obstacle : obstacles)
    if(obstacle.GetPath().size() > timeFinal)
      timeFinal = obstacle.GetPath().size();

  // Determine all of the intervals for which it is safe to start following this
  // set of configurations.
  const double timeRes = this->GetEnvironment()->GetTimeRes();

  Intervals safeIntervals;                  // All intervals for this sequence.
  Range<double>* currentInterval = nullptr; // The current interval under construction.

  // Try starting the sequence at each time step where dynamic obstacles are
  // moving.
  for(size_t tstep = 0; tstep < timeFinal; ++tstep) {
    const double startTime = tstep * timeRes;

    // Check if it is safe to start the sequence at startTime.
    bool safe = true;
    size_t i = 0;
    for(const CfgType& cfg : _cfgs) {
      safe &= IsSafe(cfg, startTime + i * timeRes);
      if(!safe)
        break;
      ++i;
    }

    // If it is not safe to start the sequence from startTime, end the current
    // safe interval, if any.
    if(!safe) {
      currentInterval = nullptr;
    }
    // Otherwise, we can include this startTime in the safe intervals.
    else {
      // Start a new interval if we don't have one already.
      if(!currentInterval) {
        safeIntervals.emplace_back(startTime, startTime);
        currentInterval = &safeIntervals.back();
      }

      // Expand the end of the current interval to include this startTime.
      currentInterval->max = startTime;
    }
  }

  // If we still have a current interval, then the last interval was safe
  // through the end of the dynamic obstacle motions. Extend it to the end of
  // time.
  if(currentInterval)
    currentInterval->max = std::numeric_limits<double>::max();

  return safeIntervals;
}

// Next commented function are not used anymore in the conflict checking 
// process of the CBS work, we may get rid of them soon
/*-------------------------------------------------------------------------
template <typename MPTraits>
bool
SafeIntervalTool<MPTraits>::
IsSafe(const CfgType& _cfg, const double _timestep, Robot*& _conflictRobot2,
 vector<CfgType> _obstacle, CfgType& _conflictCfg) {

  auto robotMultiBody = _cfg.GetRobot()->GetMultiBody();
  robotMultiBody->Configure(_cfg);
  // Get the valididty checker and make sure it has type
  // CollisionDetectionValidity.
  /// @TODO Figure out how to avoid needing this downcast so that we can
  ///       leverage more efficient compose checks (like checking the bounding
  ///       spheres first).
  auto basevc = this->GetValidityChecker(m_vcLabel);
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc.get());
  
  // Compute the step number associated with _timestep.
  const double timeRes = this->GetEnvironment()->GetTimeRes();
  //double timeRes = 1;
  const size_t currentStep = std::lround(_timestep / timeRes);

  // Check this configuration against each dynamic obstacle.
  // Determine the obstacle's position at the current timestep. If it is
  // already done moving, use its last position.
  const auto& path = _obstacle;
  _conflictRobot2 = _obstacle[0].GetRobot();
  const size_t lastStep = path.size(),
               useStep  = std::min(currentStep, lastStep);
  if (useStep > _obstacle.size()-1)
    return true; 
  const CfgType& position = path[useStep];

  // Configure the obstacle at the current timestep.
  auto obstacleMultiBody = _obstacle[0].GetRobot()->GetMultiBody();
  obstacleMultiBody->Configure(position);

  // If the obstacle is in collision with _cfg at _timestep, return false
  CDInfo cdInfo;
  if(vc->IsMultiBodyCollision(cdInfo, obstacleMultiBody, robotMultiBody,
      this->GetNameAndLabel())) {
    if(this->m_debug) {
      std::cout<< "robot cfg:" << _cfg.PrettyPrint() 
      << "\nobstacle cfg:" << position.PrettyPrint() 
      << "\nConflict at timestep " << _timestep << std::endl;
    }
    _conflictCfg = position;
    return false;
  }

  // If we haven't detected a collision, the configuration is safe.
  return true;
}

template <typename MPTraits>
bool
SafeIntervalTool<MPTraits>::
FindConflictCfg(const std::vector<Cfg>& _cfgs, CfgType& _conflictCfg, 
	Robot*& _conflictRobot1, Robot*& _conflictRobot2, 
	double& _conflictTimestep, vector<CfgType> _obstacle) {
  //If there is no cfgs to check, there is no conflict
  if(_cfgs.empty())
    return false;

  const double timeRes = this->GetEnvironment()->GetTimeRes();
  
  // We are checking both robot and obstacles at same current time
  // _conflictTimestep will have track of each cgf intermediate 
  // of the agents'path
  bool safe = true;
  for(const CfgType& cfg : _cfgs) {
    safe &= IsSafe(cfg, _conflictTimestep * timeRes, _conflictRobot2, 
    	_obstacle, _conflictCfg);
    if(!safe) {
      // If cfg is not safe, we have found a conflict
      _conflictTimestep = _conflictTimestep * timeRes;
      _conflictRobot1 = cfg.GetRobot();
      return true;
    }
    ++_conflictTimestep;
  }
  return false;
}

template <typename MPTraits>
Conflict<MPTraits>
SafeIntervalTool<MPTraits>::
FindConflictType(SafeIntervalTool<MPTraits>::Path* _path, 
	Robot*& _conflictRobot1, Robot*& _conflictRobot2, double& _conflictTimestep,
	 vector<CfgType> _obstacle) {
  Conflict<MPTraits> conflict;
  CfgType conflictCfg(nullptr);
  auto vids = _path->VIDs();
  // /We just check the first vertex
  std::vector<CfgType> cfg{_path->GetRoadmap()->GetVertex(vids[0])};
  if(FindConflictCfg(cfg, conflictCfg, _conflictRobot1, _conflictRobot2,
   _conflictTimestep, _obstacle)) {
  	if(this->m_debug)
    	std::cout << "Conflict found in Vertex: " << vids[0] <<std::endl;
    conflict.r1 = _conflictRobot1;
    conflict.r2 = _conflictRobot2;
    conflict.conflictTimestep = _conflictTimestep;
    conflict.id1 = vids[0];
    conflict.t1 = Conflict<MPTraits>::Type::Vertex;
    conflict.emptyConflict = false;
    conflict.conflictCfg = conflictCfg;
    return conflict;
  } 
  size_t i = 1;
  // We check one edge and then one vertex per iteration until we 
  // check the full path or find a conflict
  while( i < vids.size()){
    auto c1 = _path->GetRoadmap()->GetVertex(vids[i-1]);
    auto c2 = _path->GetRoadmap()->GetVertex(vids[i]);
    std::vector<CfgType> cfgs = Intermediates(c1,c2);
    ///Checking an edge
    if(FindConflictCfg(cfgs, conflictCfg, _conflictRobot1,
    	 _conflictRobot2, _conflictTimestep, _obstacle)) {
    	if(this->m_debug)
      	std::cout << "Conflict found in Edge (" << vids[i-1]  << "," 
      		<< vids[i]<< ")  "<< cfgs.size() <<std::endl;
      conflict.conflictTimestep = _conflictTimestep;
      conflict.id1 = vids[i-1];
      conflict.id2 = vids[i];
      conflict.r1 = _conflictRobot1;
      conflict.r2 = _conflictRobot2;
      conflict.t1 = Conflict<MPTraits>::Type::Edge;
      conflict.emptyConflict = false;
      conflict.conflictCfg = conflictCfg;
      return conflict;
    }     
    std::vector<CfgType> cfg{_path->GetRoadmap()->GetVertex(vids[i])};
    ///Checking a vertex 
    if(FindConflictCfg(cfg, conflictCfg, _conflictRobot1, 
    	_conflictRobot2, _conflictTimestep, _obstacle)) {
    	if(this->m_debug)
      	std::cout << "Conflict found in Vertex: " << vids[i] <<std::endl;
      conflict.conflictTimestep = _conflictTimestep;
      conflict.id1 = vids[i-1];
      conflict.id2 = vids[i];
      conflict.r1 = _conflictRobot1;
      conflict.r2 = _conflictRobot2;
      conflict.t1 = Conflict<MPTraits>::Type::Edge;
      conflict.emptyConflict = false;
      conflict.conflictCfg = conflictCfg;
      return conflict;
    }
    ++i;
  }
  return conflict;
}

template <typename MPTraits>
Conflict<MPTraits>
SafeIntervalTool<MPTraits>::
FindConflict(SafeIntervalTool<MPTraits>::Path* _path, 
	SafeIntervalTool<MPTraits>::Path* _obstacle){
  auto cfgs = _path->Cfgs();
  auto cfg = cfgs[0];
  Robot* conflictRobot1 = cfg.GetRobot(); 
  Robot* conflictRobot2 = cfg.GetRobot();
  double conflictTimestep{0};
  Conflict <MPTraits>conflict; 
  auto obstacle = FullPath(_obstacle);
  if(obstacle.empty())
    return conflict;
  conflict = FindConflictType(_path, conflictRobot1, 
  	conflictRobot2, conflictTimestep, obstacle);
  return conflict;
}
------------------------------------------------------------------*/

template <typename MPTraits>
bool
SafeIntervalTool<MPTraits>::
IsEdgeSafe(const CfgType _source, const CfgType _target, 
	const CfgType _conflictCfg) {

  std::vector<CfgType> path;
  path.push_back(_source);
  auto robot = _source.GetRobot();
  auto roadmap = this->GetRoadmap(robot);
  typename RoadmapType::EI ei;

  // Reconstructing edge 
  roadmap->GetEdge(roadmap->GetVID(_source),roadmap->GetVID(_target),ei);

  // Use the local planner from p the edge lp.
  // Fall back to straight-line if edge lp is not available (this will always
  // happen if it was grown with an extender).
  typename MPTraits::MPLibrary::LocalPlannerPointer lp;
  auto lib = this->GetMPLibrary();
  auto env = lib->GetMPProblem()->GetEnvironment();
  try {
    lp = lib->GetLocalPlanner(ei->property().GetLPLabel());
  }
  catch(...) {
    lp = lib->GetLocalPlanner("sl");
  }
  // Construct a resolution-level path along the recreated edge.
  std::vector<CfgType> recreatedEdge = ei->property().GetIntermediates();
  recreatedEdge.insert(recreatedEdge.begin(), _source);
  recreatedEdge.push_back(_target);
  for(auto cit = recreatedEdge.begin(); cit + 1 != recreatedEdge.end(); ++cit) {
    std::vector<CfgType> edge = lp->ReconstructPath(*cit, *(cit+1),
        std::vector<CfgType>(), env->GetPositionRes(), env->GetOrientationRes());
    path.insert(path.end(), edge.begin(), edge.end());
  }

  path.push_back(_target);

  auto robotMultiBody = _conflictCfg.GetRobot()->GetMultiBody();
  robotMultiBody->Configure(_conflictCfg);
  // Get the valididty checker and make sure it has type
  // CollisionDetectionValidity.
  /// @TODO Figure out how to avoid needing this downcast so that we can
  ///       leverage more efficient compose checks (like checking the bounding
  ///       spheres first).

  auto basevc = this->GetValidityChecker(m_vcLabel);

  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc.get());

  for(size_t i = 0 ; i < path.size() ; ++i ) {

    const CfgType& position = path[i];

    // Configure the obstacle at the current timestep.
    auto obstacleMultiBody = path[0].GetRobot()->GetMultiBody();
    obstacleMultiBody->Configure(position);

    // If the obstacle is in collision with _cfg at _timestep, return false
    CDInfo cdInfo;
    if(vc->IsMultiBodyCollision(cdInfo, obstacleMultiBody, robotMultiBody,
        this->GetNameAndLabel())) {
      return false;
    }
  }

  // If we haven't detected a collision, the edge is safe.
  return true;

}

template <typename MPTraits>
pair<pair<size_t,pair<typename MPTraits::CfgType,double>>,
	pair<size_t,pair<typename MPTraits::CfgType,double>>>
SafeIntervalTool<MPTraits>::
FindConflict(vector<SafeIntervalTool<MPTraits>::Path*> _paths) {

  vector<vector<CfgType>> paths;

  for(auto& path : _paths)
    //paths.push_back(FullPath(path));
  	paths.push_back(path->FullCfgs(this->GetMPLibrary()));

  // Find the latest timestep in which a robot is still moving.
  size_t timeFinal = 0;
  for(auto& path : paths)
    if(path.size() > timeFinal)
      timeFinal = path.size();

  auto basevc = this->GetValidityChecker(m_vcLabel);

  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc.get());

  for(size_t t = 0 ; t < timeFinal ; ++t) {
    const double timeRes = this->GetEnvironment()->GetTimeRes();
    for(size_t i = 0 ; i < paths.size() ; ++i ) {
      size_t t1 = t;
      if(t > paths[i].size()-1)
        t1 = paths[i].size()-1;
      auto robotMultiBody = paths[i][0].GetRobot()->GetMultiBody();
      robotMultiBody->Configure(paths[i][t1]);
      for(size_t j = 0 ; j < paths.size() ; ++j) {
        if ( i == j)
          continue;
        size_t t2 = t;
        if(t > paths[j].size()-1)
          t2 = paths[j].size()-1;
        auto obstacleMultiBody = paths[j][0].GetRobot()->GetMultiBody();
        obstacleMultiBody->Configure(paths[j][t2]);

        CDInfo cdInfo;
        if(vc->IsMultiBodyCollision(cdInfo, obstacleMultiBody, robotMultiBody,
            this->GetNameAndLabel())) {
          double ti = timeRes * static_cast<double>(t1);
          double tj = timeRes * static_cast<double>(t2);
          auto cfgConflict1 = make_pair(i,make_pair(paths[j][t2],ti));
          auto cfgConflict2 = make_pair(j,make_pair(paths[i][t1],tj));
          auto pairCfgConflicts = make_pair(cfgConflict1,cfgConflict2);
          if(this->m_debug) {
            std::cout << "Conflict on robot " << i << " at timestep " << ti << " at cfg " << paths[i][t1].PrettyPrint() 
            << "\nConflict on robot " << j << " at timestep "<< tj <<" at cfg " << paths[j][t2].PrettyPrint() 
            << std::endl;
          }
          return pairCfgConflicts;         
        }
      }
    }
  }

  return make_pair(make_pair(-1,make_pair(paths[0][0],-1)),make_pair(-1,make_pair(paths[0][0],-1))); 
}
/*----------------------------------------------------------------------------*/

#endif

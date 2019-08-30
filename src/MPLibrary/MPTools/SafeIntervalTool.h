#ifndef PMPL_SAFE_INTERVAL_TOOL_H_
#define PMPL_SAFE_INTERVAL_TOOL_H_

#include <vector>

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/XMLNode.h"
#include "MPProblem/DynamicObstacle.h"
#include "MPLibrary/Conflict.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "MPLibrary/CBSTree.h"


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

  	typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::WeightType             WeightType;
    typedef typename MPTraits::Path                   Path; 
    typedef typename RoadmapType::VID                 VID;
    typedef typename CBSNode<MPTraits>::ConflictCfg   ConflictCfg;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::vector<Range<double>> Intervals; ///< A set of time intervals.

    struct ConflictRobot {
      ConflictCfg conflictCfg;
      size_t num_robot;

    }; 

    struct PairConflict {
      ConflictRobot conflict1;
      ConflictRobot conflict2;
    };


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

    /// Compute the safe intervals for a given Edge, a source and a target.
    /// @param _weight The edge to compute safeIntervals's for.
    /// @param _weight The source VID to compute safeIntervals's for.
    /// @param _weight The target VID to compute safeIntervals's for.
    /// @return The set of safe intervals for _weight.
    Intervals ComputeIntervals(const WeightType& _weight, const VID _source, 
      const VID _target, const CfgType& _dummyCfg);

    ///@}
    ///@name Interval Checking
    ///@{

    /// Determine if a timestep is contained within a SafeInterval
    /// @param _intervals The safe intervals to check in.
    /// @param _timestep The timestep to check.
    bool ContainsTimestep(const Intervals& _intervals, const double _timestep);  

    /// Checking if an edge is collision-free with an external cfg
    /// @param _source The cfg source of the edge.
    /// @param _target The cfg target of the edge.
    /// @param _conflictCfg The external cfg to check.
    bool IsEdgeSafe(const CfgType& _source, const CfgType& _target, 
    	const CfgType& _conflictCfg);

    /// Checking if an edge is dynamically collision-free against a set of
    /// dynamic obstacles
    /// @param _source The cfg source of the edge.
    /// @param _target The cfg target of the edge.
    /// @param _timestep The external cfg to check.
    bool IsEdgeDynamicallySafe(const CfgType _source, const CfgType _target, 
    const double _sourceDistance, const double _targetDistance);

    /// Checking if a set of paths has inter-robot collisions.
    /// @param _paths The set f paths to check.
    PairConflict FindConflict(const vector<Path*>& _paths) ;

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
ComputeIntervals(const WeightType& _weight, const VID _source, 
  const VID _target, const CfgType& _dummyCfg) {
  if(m_edgeIntervals[&_weight].empty()) {

    auto robot = _dummyCfg.GetRobot();
    auto roadmap = this->GetRoadmap(robot);
    std::vector<CfgType> edge;
    edge.push_back(roadmap->GetVertex(_source));
    std::vector<CfgType> intermediates = roadmap->ReconstructEdge(this->GetMPLibrary(), 
      _source, _target);
    edge.insert(edge.end(), intermediates.begin(), intermediates.end());
    edge.push_back(roadmap->GetVertex(_target));
    std::cout << "ComputeIntervals, intermediates size: " << edge.size() << std::endl;
    m_edgeIntervals[&_weight] = ComputeSafeIntervals(edge);
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


template <typename MPTraits>
bool
SafeIntervalTool<MPTraits>::
IsEdgeSafe(const CfgType& _source, const CfgType& _target, 
	const CfgType& _conflictCfg) {
  //Reconstructing edge "path"
  std::vector<CfgType> path;
  path.push_back(_source);
  auto robot = _source.GetRobot();
  auto roadmap = this->GetRoadmap(robot);
  std::vector<CfgType> edge = this->GetMPLibrary()->ReconstructEdge(roadmap, 
    roadmap->GetVID(_source), roadmap->GetVID(_target));
  path.insert(path.end(), edge.begin(), edge.end());
  path.push_back(_target);

  // Get the valididty checker and make sure it has type
  // CollisionDetectionValidity.
  /// @TODO Figure out how to avoid needing this downcast so that we can
  ///       leverage more efficient compose checks (like checking the bounding
  ///       spheres first).
  auto basevc = this->GetValidityChecker(m_vcLabel);
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc.get());

  auto robotMultiBody = _conflictCfg.GetRobot()->GetMultiBody();
  robotMultiBody->Configure(_conflictCfg);

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
typename SafeIntervalTool<MPTraits>::PairConflict
SafeIntervalTool<MPTraits>::
FindConflict(const std::vector<Path*>& _paths) {
  vector<vector<CfgType>> paths;
  for(const auto& path : _paths)
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
          if(this->m_debug) {
            std::cout << "Conflict on robot " << i << " at timestep " << ti 
            << " at cfg " << paths[i][t1].PrettyPrint() << "\nConflict on robot " 
            << j << " at timestep "<< tj <<" at cfg " << paths[j][t2].PrettyPrint() 
            << std::endl;
          }

          PairConflict pairConflict;
          pairConflict.conflict1.conflictCfg.conflictCfg = paths[j][t2];
          pairConflict.conflict1.conflictCfg.timestep = ti;
          pairConflict.conflict1.num_robot = i;
          pairConflict.conflict2.conflictCfg.conflictCfg = paths[i][t1];
          pairConflict.conflict2.conflictCfg.timestep = tj;
          pairConflict.conflict2.num_robot = j;


          return pairConflict;         
        }
      }
    }
  }
  PairConflict nullPair;
  nullPair.conflict1.num_robot = INVALID_VID;
  return nullPair;
}


template <typename MPTraits>
bool
SafeIntervalTool<MPTraits>::
IsEdgeDynamicallySafe(const CfgType _source, const CfgType _target, 
  const double _sourceDistance, const double _targetDistance) {

  std::vector<CfgType> path;
  path.push_back(_source);
  auto robot = _source.GetRobot();
  auto roadmap = this->GetRoadmap(robot);
  std::vector<CfgType> edge = this->GetMPLibrary()->ReconstructEdge(roadmap, 
    roadmap->GetVID(_source), roadmap->GetVID(_target));
  path.insert(path.end(), edge.begin(), edge.end());
  path.push_back(_target);
  // Getting dynamic obstacles.
  const auto& obstacles = this->GetMPProblem()->GetDynamicObstacles();
  if(obstacles.empty())
    return true;
  // Find the latest timestep in which a dynamic obstacle is still moving.
  size_t timeFinal = 0;
  for(auto& obstacle : obstacles)
    if(obstacle.GetPath().size() > timeFinal)
      timeFinal = obstacle.GetPath().size();
  // Get the valididty checker and make sure it has type
  // CollisionDetectionValidity.
  /// @TODO Figure out how to avoid needing this downcast so that we can
  ///       leverage more efficient compose checks (like checking the bounding
  ///       spheres first).

  auto basevc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric("euclidean");
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc.get());
  // Checking the edge "path" against its corresponding sections of the obsctacles
  // trajetories w.r.t. the traversed time/distance 
  for(auto& obstacle : obstacles) {
    auto obstaclePath = obstacle.GetPath();
    size_t sourceCounter = 0;
    double obstacleSourceDistance = 0;
    while(obstacleSourceDistance < _sourceDistance) {
      if(sourceCounter+1 >= obstaclePath.size())
        break;
      obstacleSourceDistance += dm->Distance(obstaclePath[sourceCounter],
        obstaclePath[sourceCounter+1]);
      ++sourceCounter;
    }
    size_t targetCounter = sourceCounter;
    double obstacleTargetDistance = obstacleSourceDistance;
    while(obstacleTargetDistance < _targetDistance) {
      if(targetCounter+1 >= obstaclePath.size())
        break;
      obstacleTargetDistance += dm->Distance(obstaclePath[targetCounter],
        obstaclePath[targetCounter+1]);
      ++targetCounter;
    }

    for( size_t j = sourceCounter ; j < targetCounter ; ++j) {
      for(size_t i = 0 ; i < path.size() ; ++i ) {
        // Configure the robot at the current timestep.
        auto robotMultiBody = path[0].GetRobot()->GetMultiBody();
        robotMultiBody->Configure(path[i]);
        // Configure the obstacle at the current timestep.
        auto obstacleMultiBody = obstaclePath[0].GetRobot()->GetMultiBody();
        obstacleMultiBody->Configure(obstaclePath[j]);
        // If the obstacle is in collision with _cfg at _timestep, return false
        CDInfo cdInfo;
        if(vc->IsMultiBodyCollision(cdInfo, obstacleMultiBody, robotMultiBody,
            this->GetNameAndLabel())) {
          return false;
        }
      }    
    }
  }
  // If we haven't detected a collision, the edge is safe.
  return true;
}


/*----------------------------------------------------------------------------*/

#endif

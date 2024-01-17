#include <vector>

#include "SafeIntervalTool.h"
#include "MPProblem/DynamicObstacle.h"
#include "MPProblem/Robot/Robot.h"
#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"

/*------------------------------- Construction -------------------------------*/

SafeIntervalTool::
SafeIntervalTool() : MPBaseObject() {
  this->SetName("SafeIntervalTool");
}


SafeIntervalTool::
SafeIntervalTool(XMLNode& _node) : MPBaseObject(_node) {
  this->SetName("SafeIntervalTool");

  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
}


SafeIntervalTool::
~SafeIntervalTool() = default;

/*-------------------------- MPBaseObject Overrides ---------------------------*/

void
SafeIntervalTool::
Initialize() {
  // Clear out the interval caches.
  m_cfgIntervals.clear();
  m_edgeIntervals.clear();
}

/*-------------------------- Intervals Computations ---------------------------*/

typename SafeIntervalTool::Intervals
SafeIntervalTool::
ComputeIntervals(const Cfg& _cfg) {
  if(m_cfgIntervals[&_cfg].empty()) {
    const std::vector<Cfg> cfg{_cfg};
    m_cfgIntervals[&_cfg] = ComputeSafeIntervals(cfg);
  }

  return m_cfgIntervals[&_cfg];
}


typename SafeIntervalTool::Intervals
SafeIntervalTool::
ComputeIntervals(const WeightType& _weight, const VID _source,
  const VID _target, RoadmapType* _roadmap) {
  //if(m_edgeIntervals[&_weight].empty()) {
    std::vector<Cfg> edge;
    edge.push_back(_roadmap->GetVertex(_source));
    std::vector<Cfg> intermediates = this->GetMPLibrary()->ReconstructEdge(
        _roadmap, _source, _target);
    edge.insert(edge.end(), intermediates.begin(), intermediates.end());
    edge.push_back(_roadmap->GetVertex(_target));

    if(this->m_debug)
      std::cout << "ComputeIntervals, intermediates size: " << edge.size()
                << std::endl;
    m_edgeIntervals[&_weight] = ComputeSafeIntervals(edge);
  //}
  return m_edgeIntervals[&_weight];
}


bool
SafeIntervalTool::
ContainsTimestep(const Intervals& _intervals, const double _timestep) {
  for(auto& range : _intervals) {
    if(range.Contains(_timestep))
      return true;
  }
  return false;
}

/*--------------------------------- Helpers ----------------------------------*/

bool
SafeIntervalTool::
IsSafe(const Cfg& _cfg, const double _timestep) {
  // Configure _cfg's robot at _cfg.
  auto robotMultiBody = _cfg.GetRobot()->GetMultiBody();
  robotMultiBody->Configure(_cfg);

  // Get the valididty checker and make sure it has type
  // CollisionDetectionValidity.
  /// @TODO Figure out how to avoid needing this downcast so that we can
  ///       leverage more efficient compose checks (like checking the bounding
  ///       spheres first).
  auto basevc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto vc = dynamic_cast<CollisionDetectionValidityMethod*>(basevc);

  // Compute the step number associated with _timestep.
  const double timeRes = this->GetEnvironment()->GetTimeRes();
  const size_t currentStep = std::lround(_timestep / timeRes);

  // Check this configuration against each dynamic obstacle.
  const auto& obstacles = this->GetMPProblem()->GetDynamicObstacles();
  for(const auto& obstacle : obstacles) {
    auto obStart = obstacle.GetStartTime();

    if(currentStep < obStart)
      continue;

    auto relativeStep = currentStep - obStart;

    // Determine the obstacle's position at the current timestep. If it is
    // already done moving, use its last position.
    const auto& path = obstacle.GetPath();
    const size_t lastStep = path.size(),
                 //useStep  = std::min(currentStep, lastStep -1);
                 useStep  = std::min(relativeStep, lastStep -1);
    const Cfg& position = path[useStep];

    if(_cfg.GetRobot() == position.GetRobot()) {
      throw RunTimeException(WHERE) << "HOUSTON WE HAVE A PROBLEM" << std::endl;
    }

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


typename SafeIntervalTool::Intervals
SafeIntervalTool::
ComputeSafeIntervals(const std::vector<Cfg>& _cfgs) {
  MethodTimer mt(this->GetStatClass(), "SafeIntervalTool::ComputeSafeIntervals");

  // If there are no dynamic obstacles, the safe interval is infinite.
  const auto& obstacles = this->GetMPProblem()->GetDynamicObstacles();
  if(obstacles.empty())
    return {Range<double>(0, std::numeric_limits<double>::max())};

//  std::map<Cfg, size_t> finalResting;

  // Find the latest timestep in which a dynamic obstacle is still moving.
  size_t timeFinal = 0;
  for(auto& obstacle : obstacles) {
    auto path = obstacle.GetPath();

    auto finish = path.size() + obstacle.GetStartTime();

    //if(obstacle.GetPath().size() > timeFinal) {
    if(finish > timeFinal) {
      timeFinal = finish;//obstacle.GetPath().size();
    }
  }
  
  // Determine all of the intervals for which it is safe to start following this
  // set of configurations.
  const double timeRes = this->GetEnvironment()->GetTimeRes();

  Intervals safeIntervals;                  // All intervals for this sequence.
  Range<double>* currentInterval = nullptr; // The current interval under construction.

  // Try starting the sequence at each time step where dynamic obstacles are
  // moving.
  for(size_t tstep = 0; tstep <= timeFinal; ++tstep) {
    const double startTime = tstep * timeRes;

    // Check if it is safe to start the sequence at startTime.
    bool safe = true;
    size_t i = 0;
    for(const Cfg& cfg : _cfgs) {
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
  else {
    safeIntervals.emplace_back((timeFinal +1) * timeRes, std::numeric_limits<double>::max());
  }

  return safeIntervals;
}

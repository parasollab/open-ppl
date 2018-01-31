#ifndef SAFE_INTERVAL_TOOL_H_
#define SAFE_INTERVAL_TOOL_H_

#include <vector>

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/XMLNode.h"
#include "MPProblem/DynamicObstacle.h"
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

    typedef typename MPTraits::CfgType    CfgType;
    typedef typename MPTraits::WeightType WeightType;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::vector<Range<double>> Intervals;

    ///@}
    ///@name Construction
    ///@{

    SafeIntervalTool();

    SafeIntervalTool(XMLNode& _node);

    virtual ~SafeIntervalTool();

    ///@}
    ///@name Intervals Computation
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

    /// Determine if a timestep is contained within a SafeInterval
    /// @param _intervals The safe intervals to check in.
    /// @param _timestep The timestep to check.
    bool ContainsTimestep(const Intervals& _intervals, const double _timestep);
  private:

    ///@name Helpers
    ///@{

    /// Determine if a configuration is safe at the given time step.
    /// @param _cfg The configuration to check.
    /// @param _timestep The timestep for the dynamic obstacles.
    bool IsSafe(const CfgType& _cfg, const double _timestep);

    /// Computes the safe interval(s) for a Cfg or Edge
    /// @param _cfgs The cfg(s) to compute the safeIntervalss for.
    /// Represent a cfg as a vector of size 1 and an
    /// edge as a vector of cfgs.
    Intervals ComputeSafeIntervals(const std::vector<Cfg>& _cfgs);
    ///@}

    ///@name Internal State
    ///@{
    std::string m_vcLabel;

    std::unordered_map<const CfgType*, Intervals> m_cfgIntervals;
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

/*-------------------------- Intervals Computations ---------------------------*/

template <typename MPTraits>
typename SafeIntervalTool<MPTraits>::Intervals
SafeIntervalTool<MPTraits>::
ComputeIntervals(const CfgType& _cfg) {
  if(m_cfgIntervals[&_cfg].empty()){
    const std::vector<CfgType> cfg = {_cfg};
    m_cfgIntervals[&_cfg] = ComputeSafeIntervals(cfg);
  }

  return m_cfgIntervals[&_cfg];
}


template <typename MPTraits>
typename SafeIntervalTool<MPTraits>::Intervals
SafeIntervalTool<MPTraits>::
ComputeIntervals(const WeightType& _weight) {
  if(m_edgeIntervals[&_weight].empty()){
    const vector<CfgType>& cfgs = _weight.GetIntermediates();
    m_edgeIntervals[&_weight] = ComputeSafeIntervals(cfgs);
  }

  return m_edgeIntervals[&_weight];
}

template <typename MPTraits>
bool
SafeIntervalTool<MPTraits>::
ContainsTimestep(const Intervals& _intervals, const double _timestep){
  for(auto& range : _intervals){
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

  auto robotMultiBody = _cfg.GetRobot()->GetMultiBody();
  robotMultiBody->Configure(_cfg);

  const auto& obstacles = this->GetMPProblem()->GetDynamicObstacles();
  auto basevc = this->GetValidityChecker(m_vcLabel).get();
  auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(basevc);
  auto env = this->GetEnvironment();

  const double timeRes = env->GetTimeRes();

  const long currentStep = std::lround(_timestep / timeRes);

  Cfg* currentObstaclePosition;

  for(auto& obstacle : obstacles){
    if (_timestep > (timeRes * obstacle->m_path.size())){
      currentObstaclePosition = &obstacle->m_path.back();
    }
    else{
      currentObstaclePosition = &obstacle->m_path[currentStep];
    }
    auto obstacleMultiBody = currentObstaclePosition->GetRobot()->GetMultiBody();
    obstacleMultiBody->Configure(*currentObstaclePosition);
    // If the obstacle is in collision with _cfg at _timestep, return false
    CDInfo cdinfo;
    if(vc->IsInterRobotCollision(cdinfo, obstacleMultiBody, robotMultiBody,
          this->GetNameAndLabel())) {
      return false;
    }
  }
  return true;
}
template <typename MPTraits>
typename SafeIntervalTool<MPTraits>::Intervals
SafeIntervalTool<MPTraits>::
ComputeSafeIntervals(const std::vector<Cfg>& _cfgs){
  MethodTimer mt(this->GetStatClass(), "SafeIntervalTool::ComputeSafeIntervals");
  Intervals safeIntervals;
  Range<double>* currentInterval = nullptr;

  // Find how long there is potential for collision with dynamic obstacles
  // following known paths.
  // Need a different distinction to account for period motion of DOs.
  size_t timeFinal = 0;
  const auto& obstacles = this->GetMPProblem()->GetDynamicObstacles();

  // If there are no dynamic obstacles, the safe interval is infinite.
  if(obstacles.empty())
    return Intervals{Range<double>(0, std::numeric_limits<double>::max())};

  for(auto& obstacle : obstacles){
    if(obstacle->m_path.size() > timeFinal){
      timeFinal = obstacle->m_path.size();
    }
  }

  auto env = this->GetEnvironment();
  const double timeRes = env->GetTimeRes();

  for(size_t tstep = 0; tstep < timeFinal; tstep++){
    bool safe = true;
    int i = 0; // iterates over an edge to find safe interval
               //to start down an edge
    for(Cfg cfg : _cfgs){
      if(IsSafe(cfg,(tstep+i)*timeRes)){
        i++;
        continue;
      }
      safe = false;
      break;
    }
    // still safe and expanding current interval
    if(safe and currentInterval){
      currentInterval->max = tstep*timeRes;
    }
    // no longer safe and ending current interval
    else if(!safe and currentInterval){
      currentInterval = nullptr;
    }
    // newly safe so starting new interval
    else if(safe and !currentInterval){
      safeIntervals.emplace_back(tstep*timeRes, tstep*timeRes);
      currentInterval = &safeIntervals.back();
    }
  }
  return safeIntervals;
}
/*----------------------------------------------------------------------------*/

#endif

#ifndef SAFE_INTERVAL_TOOL_H_
#define SAFE_INTERVAL_TOOL_H_

#include <vector>

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/XMLNode.h"


////////////////////////////////////////////////////////////////////////////////
/// Computes safe intervals for Cfgs and Edges. A 'safe interval' is an interval
/// of time where collision with a known dynamic obstacle (with known
/// trajectory) will not occur.
///
/// This tool implements the concept of a 'Safe Interval' from the paper:
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

    typedef std::vector<Range<double>> Interval;

    ///@}
    ///@name Construction
    ///@{

    SafeIntervalTool();

    SafeIntervalTool(XMLNode& _node);

    virtual ~SafeIntervalTool();

    ///@}
    ///@name Interval Computation
    ///@{

    /// Compute the safe intervals for a given Cfg.
    /// @param _cfg The configuration to compute SI's for.
    /// @return The set of safe intervals for _cfg.
    Interval ComputeInterval(const CfgType& _cfg);

    /// Compute the safe intervals for a given Edge.
    /// @param _weight The edge to compute SI's for.
    /// @return The set of safe intervals for _weight.
    Interval ComputeInterval(const WeightType& _weight);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Determine if a configuration is safe at the given time step.
    /// @param _cfg The configuration to check.
    /// @param _timestep The timestep for the dynamic obstacles.
    bool IsSafe(const CfgType& _cfg, const double _timestep);

    ///@}

    ///@name Internal State
    ///@{
    std::string m_vcLabel;
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

/*-------------------------- Interval Computations ---------------------------*/

template <typename MPTraits>
typename SafeIntervalTool<MPTraits>::Interval
SafeIntervalTool<MPTraits>::
ComputeInterval(const CfgType& _cfg) {

  return Interval();
}


template <typename MPTraits>
typename SafeIntervalTool<MPTraits>::Interval
SafeIntervalTool<MPTraits>::
ComputeInterval(const WeightType& _weight) {
  //vector<Cfg>& cfgs = _weight->GetIntermediates()
  return Interval();
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
SafeIntervalTool<MPTraits>::
IsSafe(const CfgType& _cfg, const double _timestep) {

  const auto& obstacles = this->GetMPProblem()->GetDynamicObstacles();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto env = this->GetEnvironment();

  const double timeRes = env->GetTimeRes();

  const long currentStep = std::lround(_timestep / timeRes);

  for(auto& obstacle : obstacles){
    if (_timestep > (timeRes * obstacle->m_path.size()))
      continue;

    // If the obstacle is in collision with _cfg at _timestep, return false
    CDInfo cdinfo;
    if(vc->IsInCollision(cdinfo,
        obstacle->m_path[currentStep].GetRobot()->GetMultiBody(),
        _cfg.GetRobot()->GetMultiBody(), this->GetNameAndLabel())) {
      return false;
    }
  }
  return true;
}

// TODO: Implement helper function for ComputeInterval
// TODO: rename
/*template <typename MPTraits>
typename SafeIntervalTool<MPTraits>::Interval
SafeIntervalTool<MPTraits>::
ComputeInterval(std::vector<Cfg> _cfgs){

  return Interval();
}*/
/*----------------------------------------------------------------------------*/

#endif

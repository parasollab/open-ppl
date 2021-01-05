#ifndef PMPL_AO_ANALYZER_H_
#define PMPL_AO_ANALYZER_H_

#include "MPStrategyMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Run another strategy in an AO (asymptotically-optimal) setting while
/// gathering specific data into history files on each iteration.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class AOAnalyzer : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType     CfgType;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID      VID;

    ///@}
    ///@name Construction
    ///@{

    AOAnalyzer() = default;

    AOAnalyzer(XMLNode& _node);

    virtual ~AOAnalyzer() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Run() override;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_strategyLabel;   ///< Strategy to run.

    /// Include times for these NFs in the NN time history
    std::vector<std::string> m_nfClockLabels;

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
AOAnalyzer<MPTraits>::
AOAnalyzer(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  m_strategyLabel = _node.Read("querySampler", false, "", "Start/goal sampler.");

  // Parse evaluator child nodes.
  for(auto& child : _node)
    if(child.Name() == "NNClock")
      m_nfClockLabels.push_back(child.Read("label", true, "", ""));
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
AOAnalyzer<MPTraits>::
Print(std::ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  _os << "\n\tStrategy Label: " << m_strategyLabel
      << "\n\tNN Clocks:";

  for(const auto& label : m_nfClockLabels)
    _os << "\n\t\t" << label;

  _os << std::endl;
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
AOAnalyzer<MPTraits>::
Run() {
  auto stats = this->GetStatClass();
  auto s = this->GetMPStrategy(m_strategyLabel);
  const std::string clockName = s->GetNameAndLabel() + "::Run";
  // Use this timer to include map evaluation in the total run time. To exclude
  // it, comment this out and use the one below.
  MethodTimer mt(stats, clockName);

  do {
    // Add history for:
    // - roadmap size
    // - run time
    // - nearest neighbor time (need NN clock name)
    stats->AddToHistory("mapsize", this->GetRoadmap()->Size());
    stats->AddToHistory("runtime", stats->GetSeconds(clockName));

    double totalNNTime = 0;
    for(const auto& label : m_nfClockLabels) {
      auto nf = this->GetNeighborhoodFinder(label);
      const std::string clockName = nf->GetNameAndLabel() + "::FindNeighbors";
      totalNNTime += stats->GetSeconds(clockName);
    }
    stats->AddToHistory("nntime", totalNNTime);

    //MethodTimer mt(stats, clockName);
    s->Iterate();
  } while(!s->EvaluateMap() and s->IsRunning());
}

/*----------------------------------------------------------------------------*/

#endif

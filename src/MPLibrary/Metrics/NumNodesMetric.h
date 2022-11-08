#ifndef PMPL_NUM_NODES_METRIC_H
#define PMPL_NUM_NODES_METRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Evaluates the number of nodes in the current roadmap.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NumNodesMetric : virtual public MetricMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    NumNodesMetric();

    NumNodesMetric(XMLNode& _node);

    virtual ~NumNodesMetric() = default;

    ///@}
    ///@name Metric Interface
    ///@{

    virtual double operator()() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    bool m_formationSpecific{true};

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
NumNodesMetric<MPTraits>::
NumNodesMetric() {
  this->SetName("NumNodesMetric");
}


template <typename MPTraits>
NumNodesMetric<MPTraits>::
NumNodesMetric(XMLNode& _node) : MetricMethod<MPTraits>(_node){
  this->SetName("NumNodesMetric");

  m_formationSpecific = _node.Read("formationSpecific",false,m_formationSpecific,
          "Flag indiciating if metric should only count active formations in "
          "group roadmap.");
}

/*---------------------------- Metric Interface ------------------------------*/

template <typename MPTraits>
double
NumNodesMetric<MPTraits>::
operator()() {
  if(this->GetGroupRoadmap()) {
    auto rm = this->GetGroupRoadmap();
    if(!m_formationSpecific) 
      return rm->Size();

    auto activeFormations = rm->GetActiveFormations();

    size_t count = 0;
    for(auto vit = rm->begin(); vit != rm->end(); vit++) {
      const auto& cfg = vit->property();
      const auto& formations = cfg.GetFormations();
      if(formations == activeFormations) {
        count++;
        continue;
      }

      if(formations.size() != activeFormations.size())
        continue;

      std::set<Formation*> seen;
      bool match = false;
      for(auto f1 : formations) {
        match = false;
        for(auto f2 : activeFormations) {
          if(seen.count(f2))
            continue;
          if(*f1 == *f2) {
            match = true;
            seen.insert(f2);
            break;
          }
        }
        if(!match)
          break;
      }

      if(match)
        count++;
    }

    return count;
  }
  else
    return this->GetRoadmap()->Size();
}

/*----------------------------------------------------------------------------*/

#endif

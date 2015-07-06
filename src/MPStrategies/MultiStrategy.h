#ifndef MULTI_STRATEGY_H_
#define MULTI_STRATEGY_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
///
/// \todo Configure for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MultiStrategy : public MPStrategyMethod<MPTraits> {
  public:
    MultiStrategy<MPTraits>();
    MultiStrategy(typename MPTraits::MPProblemType* _problem, XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Initialize() {}
    virtual void Run();
    virtual void Finalize() {}

  private:
    vector<string> m_mpsLabels;
};


template<class MPTraits>
MultiStrategy<MPTraits>::MultiStrategy() {
  this->SetName("MultiStrategy");
}


template<class MPTraits>
MultiStrategy<MPTraits>::MultiStrategy(typename MPTraits::MPProblemType* _problem, XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
    this->SetName("MultiStrategy");
    ParseXML(_node);
  }


template<class MPTraits>
void
MultiStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node)
    if(child.Name() == "MPStrategy")
        m_mpsLabels.push_back(child.Read("method", true, "", "MPStrategy"));
}


template<class MPTraits>
void MultiStrategy<MPTraits>::Run() {
  for(auto& label : m_mpsLabels) {
    cout << "MultiStrategy: Beginning Strategy: " << label << endl;
    (*this->GetMPStrategy(label))();
  }
}

#endif

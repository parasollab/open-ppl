#ifndef MULTI_STRATEGY_H_
#define MULTI_STRATEGY_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
///
/// TODO
///
/// \todo Configure for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MultiStrategy : public MPStrategyMethod<MPTraits> {

  public:

    MultiStrategy<MPTraits>();
    MultiStrategy(XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Initialize() {}
    virtual void Run();
    virtual void Finalize() {}

  private:

    vector<string> m_mpsLabels;

};


template <typename MPTraits>
MultiStrategy<MPTraits>::MultiStrategy() {
  this->SetName("MultiStrategy");
}


template <typename MPTraits>
MultiStrategy<MPTraits>::MultiStrategy(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node) {
    this->SetName("MultiStrategy");
    ParseXML(_node);
  }


template <typename MPTraits>
void
MultiStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node)
    if(child.Name() == "MPStrategy")
        m_mpsLabels.push_back(child.Read("method", true, "", "MPStrategy"));
}


template <typename MPTraits>
void MultiStrategy<MPTraits>::Run() {
  for(auto& label : m_mpsLabels) {
    cout << "MultiStrategy: Beginning Strategy: " << label << endl;
    (*this->GetMPStrategy(label))();
  }
}

#endif

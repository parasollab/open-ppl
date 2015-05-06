#ifndef MULTI_STRATEGY_H_
#define MULTI_STRATEGY_H_

#include "MPStrategyMethod.h"

template<class MPTraits>
class MultiStrategy : public MPStrategyMethod<MPTraits> {
  public:
    MultiStrategy<MPTraits>();
    MultiStrategy(typename MPTraits::MPProblemType* _problem, XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  private:
    vector<string> m_labels;
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
        m_labels.push_back(child.Read("method", true, "", "MPStrategy"));
}

template<class MPTraits>
void MultiStrategy<MPTraits>::Initialize() {
}

template<class MPTraits>
void MultiStrategy<MPTraits>::Run() {
  typedef vector<string>::iterator SIT;
  for(SIT sit = m_labels.begin(); sit != m_labels.end(); ++sit) {
    cout << "MultiStrategy: Beginning Strategy: " << *sit << endl;
    (*this->GetMPProblem()->GetMPStrategy(*sit))();
  }
}

template<class MPTraits>
void MultiStrategy<MPTraits>::Finalize() {
}
#endif

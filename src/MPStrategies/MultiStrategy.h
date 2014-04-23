#ifndef MULTISTRATEGY_H_
#define MULTISTRATEGY_H_

#include "MPStrategyMethod.h"

template<class MPTraits>
class MultiStrategy : public MPStrategyMethod<MPTraits> {
  public:
    MultiStrategy<MPTraits>();
    MultiStrategy(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);

    virtual void ParseXML(XMLNodeReader& _node);
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
MultiStrategy<MPTraits>::MultiStrategy(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
    this->SetName("MultiStrategy");
    ParseXML(_node);
  }

template<class MPTraits>
void MultiStrategy<MPTraits>::ParseXML(XMLNodeReader& _node) {
  if(this->m_debug) cout<<"Parsing XML File"<<endl;
  for(XMLNodeReader::childiterator cIter = _node.children_begin();
      cIter != _node.children_end(); ++cIter){
    if(cIter->getName() == "MPStrategy") {
        m_labels.push_back(cIter->stringXMLParameter("method", true, "",
          "MPStrategy to be used"));
      cIter->warnUnrequestedAttributes();
    }
    else
      cIter->warnUnknownNode();
  }
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

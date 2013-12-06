#ifndef MULTISTRATEGY_H_
#define MULTISTRATEGY_H_

#include "MPStrategyMethod.h"

template<class MPTraits>
class MultiStrategy : public MPStrategyMethod<MPTraits> {
  public:
    MultiStrategy<MPTraits>();
    MultiStrategy(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~MultiStrategy();

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
MultiStrategy<MPTraits>::~MultiStrategy() {
}

template<class MPTraits>
void MultiStrategy<MPTraits>::ParseXML(XMLNodeReader& _node) {
  if(this->m_debug) cout<<"Parsing XML File"<<endl;
  for(XMLNodeReader::childiterator cIter = _node.children_begin();
      cIter != _node.children_end(); ++cIter){
    if(cIter->getName() == "MPStrategy") {
        m_labels.push_back(cIter->stringXMLParameter("Method", true, "",
          "MPStrategy to be used"));
      cIter->warnUnrequestedAttributes();
    }
    else
      cIter->warnUnknownNode();
  }
}

template<class MPTraits>
void MultiStrategy<MPTraits>::Initialize() {
  cout<<"Initializing:"<<endl;
  for(size_t i = 0; i < m_labels.size(); i++) {
    cout<<m_labels[i]<<endl;
    this->GetMPProblem()->GetMPStrategy(m_labels[i])->Initialize();
    this->GetMPProblem()->GetMPStrategy(m_labels[i])->SetBaseFilename(this->GetBaseFilename());
  }
}

template<class MPTraits>
void MultiStrategy<MPTraits>::Run() {
  cout<<"Running:"<<endl;
  for(size_t i = 0; i < m_labels.size(); i++) {
    cout<<m_labels[i]<<endl;
    this->GetMPProblem()->GetMPStrategy(m_labels[i])->Run();
  }
}

template<class MPTraits>
void MultiStrategy<MPTraits>::Finalize() {
  cout<<"Finalizing:"<<endl;
  for(size_t i = 0; i < m_labels.size(); i++) {
    cout<<"File output: "<<this->GetMPProblem()->GetMPStrategy(m_labels[i])->GetBaseFilename()<<endl;
    this->GetMPProblem()->GetMPStrategy(m_labels[i])->Finalize();
  }
}
#endif

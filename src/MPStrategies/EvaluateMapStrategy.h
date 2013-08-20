#ifndef EVALUATEMAPSTRATEGY_H_
#define EVALUATEMAPSTRATEGY_H_

#include "MPStrategies/MPStrategyMethod.h"
#include "boost/lambda/lambda.hpp"

template<class MPTraits>
class EvaluateMapStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;

    EvaluateMapStrategy(string _mapFileName = "",
        const vector<string>& _evaluatorLabels = vector<string>());
    EvaluateMapStrategy(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~EvaluateMapStrategy() {}

    virtual void PrintOptions(ostream& _os) const;

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize() {}

  protected:
    string m_mapFileName;
    vector<string> m_evaluatorLabels;
};

template<class MPTraits>
EvaluateMapStrategy<MPTraits>::EvaluateMapStrategy(
    string _mapFileName,
    const vector<string>& _evaluatorLabels) :
  m_mapFileName(_mapFileName),
  m_evaluatorLabels(_evaluatorLabels) {
    this->SetName("EvaluateMapStrategy");
  }

template<class MPTraits>
EvaluateMapStrategy<MPTraits>::EvaluateMapStrategy(MPProblemType* _problem, XMLNodeReader& _node)
  : MPStrategyMethod<MPTraits>(_problem, _node) {
    this->SetName("EvaluateMapStrategy");
    ParseXML(_node);
  }


template<class MPTraits>
void
EvaluateMapStrategy<MPTraits>::PrintOptions(ostream& _os) const {
  using boost::lambda::_1;
  _os << "EvaluateMapStrategy::";
  _os << "\n\tmap file = \"" << m_mapFileName << "\"";
  _os << "\tevaluators: ";
  for_each(m_evaluatorLabels.begin(), m_evaluatorLabels.end(), cout << _1 << " ");
}


template<class MPTraits>
void
EvaluateMapStrategy<MPTraits>::ParseXML(XMLNodeReader& _node) {

  m_mapFileName = _node.stringXMLParameter("mapFilename", true, "", "Map Filename");

  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr!= _node.children_end(); ++citr)
    if(citr->getName() == "Evaluator") {
      string method = citr->stringXMLParameter("label", true, "", "Map Evaluation Method");
      m_evaluatorLabels.push_back(method);
      citr->warnUnrequestedAttributes();
    }
    else {
      citr->warnUnknownNode();
    }

  _node.warnUnrequestedAttributes();
}

template<class MPTraits>
void
EvaluateMapStrategy<MPTraits>::Initialize() {
  this->GetMPProblem()->GetRoadmap()->Read(m_mapFileName.c_str());
}

template<class MPTraits>
void
EvaluateMapStrategy<MPTraits>::Run() {
  bool passed = this->EvaluateMap(m_evaluatorLabels);
  if(passed)
    cout << "\t  (passed)\n";
  else {
    cout << "\t  (failed)\n";
    return;
  }
}

#endif

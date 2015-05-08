#ifndef EVALUATE_MAP_STRATEGY_H_
#define EVALUATE_MAP_STRATEGY_H_

#include "MPStrategies/MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class EvaluateMapStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;

    EvaluateMapStrategy(string _mapFileName = "",
        const vector<string>& _evaluatorLabels = vector<string>());
    EvaluateMapStrategy(MPProblemType* _problem, XMLNode& _node);
    virtual ~EvaluateMapStrategy() {}

    virtual void Print(ostream& _os) const;

    virtual void ParseXML(XMLNode& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize() {}

  protected:
    string m_mapFileName;
    vector<string> m_evaluatorLabels;
};

template<class MPTraits>
EvaluateMapStrategy<MPTraits>::
EvaluateMapStrategy(string _mapFileName,
    const vector<string>& _evaluatorLabels) :
  m_mapFileName(_mapFileName),
  m_evaluatorLabels(_evaluatorLabels) {
    this->SetName("EvaluateMapStrategy");
  }

template<class MPTraits>
EvaluateMapStrategy<MPTraits>::
EvaluateMapStrategy(MPProblemType* _problem, XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
    this->SetName("EvaluateMapStrategy");
    ParseXML(_node);
  }


template<class MPTraits>
void
EvaluateMapStrategy<MPTraits>::
Print(ostream& _os) const {
  _os << "EvaluateMapStrategy::";
  _os << "\n\tmap file = \"" << m_mapFileName << "\"";
  _os << "\tevaluators: ";
  for(auto&  l : m_evaluatorLabels)
      cout << l << " ";
}


template<class MPTraits>
void
EvaluateMapStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  m_mapFileName = _node.Read("mapFilename", true, "", "Map Filename");

  for(auto& child : _node)
    if(child.Name() == "Evaluator") {
      string method = child.Read("label", true, "", "Map Evaluation Method");
      m_evaluatorLabels.push_back(method);
    }
}

template<class MPTraits>
void
EvaluateMapStrategy<MPTraits>::
Initialize() {
  this->GetRoadmap()->Read(m_mapFileName.c_str());
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

#ifndef EVALUATE_MAP_STRATEGY_H_
#define EVALUATE_MAP_STRATEGY_H_

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
///
/// TODO
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class EvaluateMapStrategy : public MPStrategyMethod<MPTraits> {

  public:

    EvaluateMapStrategy(string _mapFileName = "",
        const vector<string>& _evaluatorLabels = vector<string>());
    EvaluateMapStrategy(XMLNode& _node);
    virtual ~EvaluateMapStrategy() {}

    virtual void Print(ostream& _os) const;

    virtual void ParseXML(XMLNode& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Iterate() {}
    virtual void Finalize() {}

  protected:

    string m_mapFileName;
};


template <typename MPTraits>
EvaluateMapStrategy<MPTraits>::
EvaluateMapStrategy(string _mapFileName,
    const vector<string>& _evaluatorLabels) : m_mapFileName(_mapFileName) {
  this->m_meLabels = _evaluatorLabels;
  this->SetName("EvaluateMapStrategy");
}


template <typename MPTraits>
EvaluateMapStrategy<MPTraits>::
EvaluateMapStrategy(XMLNode& _node) :
    MPStrategyMethod<MPTraits>(_node) {
  this->SetName("EvaluateMapStrategy");
  ParseXML(_node);
}


template <typename MPTraits>
void
EvaluateMapStrategy<MPTraits>::
Print(ostream& _os) const {
  _os << "EvaluateMapStrategy::";
  _os << "\n\tmap file = \"" << m_mapFileName << "\"";
  _os << "\tevaluators: ";
  for(auto&  l : this->m_meLabels)
      cout << l << " ";
}


template <typename MPTraits>
void
EvaluateMapStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  m_mapFileName = _node.Read("mapFilename", true, "", "Map Filename");

  for(auto& child : _node)
    if(child.Name() == "Evaluator") {
      string method = child.Read("label", true, "", "Map Evaluation Method");
      this->m_meLabels.push_back(method);
    }
}


template <typename MPTraits>
void
EvaluateMapStrategy<MPTraits>::
Initialize() {
  this->GetRoadmap()->Read(m_mapFileName.c_str());
}


template <typename MPTraits>
void
EvaluateMapStrategy<MPTraits>::
Run() {
  bool passed = this->EvaluateMap();
  if(passed)
    cout << "\t  (passed)\n";
  else {
    cout << "\t  (failed)\n";
    return;
  }
}

#endif

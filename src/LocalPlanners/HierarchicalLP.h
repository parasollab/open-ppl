/**
* HierarchicalLP.h
*
* This class perfoms Hierarchical Local Planner which tries to apply Local Planners
* one by one till one of them works or all of them fail.
*/

#ifndef HIERARCHICALLP_H_
#define HIERARCHICALLP_H_

#include "LocalPlannerMethod.h"
#include "LPOutput.h"

template <class MPTraits>
class HierarchicalLP : public LocalPlannerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;

    HierarchicalLP(const vector<string>& _lpLabels = vector<string>());

    HierarchicalLP(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void PrintOptions(ostream& _os) const;

    virtual bool IsConnected(
        Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false,
        bool _saveFailedPath = false);

  private:
    vector<string> m_lpLabels;
};

template<class MPTraits>
HierarchicalLP<MPTraits>::HierarchicalLP(const vector<string>& _lpLabels) :
    m_lpLabels(_lpLabels) {
  this->SetName("HierarchicalLP");
}

template<class MPTraits>
HierarchicalLP<MPTraits>::HierarchicalLP(MPProblemType* _problem, XMLNodeReader& _node) :
    LocalPlannerMethod<MPTraits>(_problem, _node){
  this->SetName("HierarchicalLP");
  XMLNodeReader::childiterator citr;
  for(citr = _node.children_begin(); citr != _node.children_end(); ++ citr) {
    if(citr->getName() == "LocalPlanner"){
      string methodLabel = citr->stringXMLParameter("method",true, "",
          "method");
      m_lpLabels.push_back(methodLabel);
    }
    else
      citr->warnUnknownNode();
  }
}

template<class MPTraits>
void
HierarchicalLP<MPTraits>::PrintOptions(ostream& _os) const {
  LocalPlannerMethod<MPTraits>::PrintOptions(_os);
  for(vector<string>::const_iterator it = m_lpLabels.begin(); it != m_lpLabels.end(); it++)
    _os << "\t " << *it << "\n";
  _os << endl;
}

template<class MPTraits>
bool
HierarchicalLP<MPTraits>::IsConnected(
    Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  _stats.IncLPAttempts(this->GetNameAndLabel());
  for(vector<string>::iterator it = m_lpLabels.begin(); it != m_lpLabels.end(); it++) {
    LocalPlannerPointer lpMethod = this->GetMPProblem()->GetLocalPlanner(*it);
    if(lpMethod->IsConnected(_env, _stats, _dm, _c1, _c2, _col, _lpOutput,
          _posRes, _oriRes, _checkCollision, _savePath, _saveFailedPath)) {
        _stats.IncLPConnections(this->GetNameAndLabel());
        return true;
    }
  }
  return false;
}

#endif

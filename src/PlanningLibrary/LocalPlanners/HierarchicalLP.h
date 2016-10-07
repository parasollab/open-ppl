#ifndef HIERARCHICAL_LP_H_
#define HIERARCHICAL_LP_H_

#include "LocalPlannerMethod.h"
#include "LPOutput.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief Apply a sequence of local planners until one succeeds, or all fail.
/// @tparam MPTraits Motion planning universe
///
/// Hierarchical local planning applies a sequence of local planners until one
/// succeeds or they all fail. It will return the information of the successful
/// local plan.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class HierarchicalLP : public LocalPlannerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;

    HierarchicalLP(const vector<string>& _lpLabels = vector<string>(), bool _saveIntermediates = false);

    HierarchicalLP(MPProblemType* _problem, XMLNode& _node);

    virtual void Print(ostream& _os) const;

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false);

  private:
    vector<string> m_lpLabels;
};

template<class MPTraits>
HierarchicalLP<MPTraits>::
HierarchicalLP(const vector<string>& _lpLabels, bool _saveIntermediates) :
  LocalPlannerMethod<MPTraits>(_saveIntermediates), m_lpLabels(_lpLabels) {
    this->SetName("HierarchicalLP");
  }

template<class MPTraits>
HierarchicalLP<MPTraits>::
HierarchicalLP(MPProblemType* _problem, XMLNode& _node) :
  LocalPlannerMethod<MPTraits>(_problem, _node){
    this->SetName("HierarchicalLP");
    for(auto& child : _node)
      if(child.Name() == "LocalPlanner")
        m_lpLabels.push_back(
            child.Read("method",true, "", "method"));
  }

template<class MPTraits>
void
HierarchicalLP<MPTraits>::Print(ostream& _os) const {
  LocalPlannerMethod<MPTraits>::Print(_os);
  _os << "\tlocal planner labels :";
  for(vector<string>::const_iterator it = m_lpLabels.begin(); it != m_lpLabels.end(); it++)
    _os << "\n\t\t" << *it;
  _os << endl;
}

template<class MPTraits>
bool
HierarchicalLP<MPTraits>::IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath) {
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  stats->IncLPAttempts(this->GetNameAndLabel());
  for(vector<string>::iterator it = m_lpLabels.begin(); it != m_lpLabels.end(); it++) {
    LocalPlannerPointer lpMethod = this->GetMPProblem()->GetLocalPlanner(*it);
    if(lpMethod->IsConnected(_c1, _c2, _col, _lpOutput,
          _posRes, _oriRes, _checkCollision, _savePath)) {
      stats->IncLPConnections(this->GetNameAndLabel());
      return true;
    }
  }
  return false;
}
#endif

#include "HierarchicalLP.h"

#include "MPLibrary/MPLibrary.h"

/******************************************************************************/

HierarchicalLP::HierarchicalLP(const vector<string>& _lpLabels,
                               bool _saveIntermediates)
    : LocalPlannerMethod(_saveIntermediates), m_lpLabels(_lpLabels) {
  this->SetName("HierarchicalLP");
}

HierarchicalLP::HierarchicalLP(XMLNode& _node) : LocalPlannerMethod(_node) {
  this->SetName("HierarchicalLP");
  for (auto& child : _node)
    if (child.Name() == "LocalPlanner")
      m_lpLabels.push_back(child.Read("method", true, "", "method"));
}

void HierarchicalLP::Print(ostream& _os) const {
  LocalPlannerMethod::Print(_os);
  _os << "\tlocal planner labels :";
  for (auto it = m_lpLabels.begin(); it != m_lpLabels.end(); it++)
    _os << "\n\t\t" << *it;
  _os << endl;
}

bool HierarchicalLP::IsConnected(const Cfg& _c1,
                                 const Cfg& _c2,
                                 Cfg& _col,
                                 LPOutput* _lpOutput,
                                 double _posRes,
                                 double _oriRes,
                                 bool _checkCollision,
                                 bool _savePath) {
  StatClass* stats = this->GetStatClass();

  stats->IncLPAttempts(this->GetNameAndLabel());
  for (vector<string>::iterator it = m_lpLabels.begin(); it != m_lpLabels.end();
       it++) {
    auto lpMethod = this->GetMPLibrary()->GetLocalPlanner(*it);
    if (lpMethod->IsConnected(_c1, _c2, _col, _lpOutput, _posRes, _oriRes,
                              _checkCollision, _savePath)) {
      stats->IncLPConnections(this->GetNameAndLabel());
      return true;
    }
  }
  return false;
}

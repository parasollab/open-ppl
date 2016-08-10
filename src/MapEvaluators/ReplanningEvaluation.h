#ifndef REPLANNINGEVALUATION_H
#define REPLANNINGEVALUATION_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DeadCode
/// @brief This method is somewhat dated and contains various unexplained
///        assumptions, such as treating all query points but the last as roots,
///        which makes little sense for PRM. It needs proper documentation and
///        validation to be brought back into the current Traits.
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ReplanningEvaluation : public LazyQuery<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::VID VID;
    typedef typename GraphType::vertex_iterator VertexIterator;
    typedef typename GraphType::adj_edge_iterator AdjEdgeIterator;

    ReplanningEvaluation();
    ReplanningEvaluation(string _envFile, string _vcLabel, const char* _queryFileName = "") :
      m_envFile(_envFile), LazyQuery<MPTraits>(_queryFileName, _vcLabel) {
      this->SetName("ReplanningEvaluation");
    }
    ReplanningEvaluation(CfgType _start, CfgType _goal, string _vcLabel) :
      m_envFile(""), LazyQuery<MPTraits>(_start, _goal, _vcLabel) {
      this->SetName("ReplanningEvaluation");
    }
    ReplanningEvaluation(typename MPTraits::MPProblemType* _problem, XMLNode& _node);
    virtual ~ReplanningEvaluation();

    virtual void Print(ostream& _os) const;

    virtual bool operator()();

    virtual bool CanRecreatePath(RoadmapType* _rdmp,
      vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath);

  private:
    void SetEnvironment();
    void ResetValidity();
    void RemoveInvalidPortions();
    vector<VID> GetRoots();

  protected:
    string m_envFile;
};


template<class MPTraits>
ReplanningEvaluation<MPTraits>::
ReplanningEvaluation() {
  this->SetName("ReplanningEvaluation");
  m_envFile = "";
}

template<class MPTraits>
ReplanningEvaluation<MPTraits>::
ReplanningEvaluation(typename MPTraits::MPProblemType* _problem,
    XMLNode& _node) : LazyQuery<MPTraits>(_problem, _node) {
  this->SetName("ReplanningEvaluation");
  m_envFile = _node.Read("envFile", true, "", "Environment filename");
}

template<class MPTraits>
ReplanningEvaluation<MPTraits>::~ReplanningEvaluation() {
}

template<class MPTraits>
void
ReplanningEvaluation<MPTraits>::Print(ostream& _os) const {
  LazyQuery<MPTraits>::Print(_os);
  _os << "\n\tEnvironment = " << m_envFile <<endl;
}

template<class MPTraits>
bool
ReplanningEvaluation<MPTraits>::operator()() {
  StatClass* ReplanStatClass = this->GetMPProblem()->GetStatClass();
  string replanClockName = "Replan Evaluator  ";
  ReplanStatClass->StartClock(replanClockName);

  static bool flag = false;
  if(flag == false){ //called first time
    SetEnvironment();
    ResetValidity();
    flag = true;
  }

  bool ans = Query<MPTraits>::operator()();

  ReplanStatClass->StopClock(replanClockName);
  return ans;
}

template<class MPTraits>
bool
ReplanningEvaluation<MPTraits>::CanRecreatePath(RoadmapType* _rdmp,
    vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath) {
  bool ans = LazyQuery<MPTraits>::CanRecreatePath(_rdmp, _attemptedPath, _recreatedPath);
  if(!ans) {
    StatClass* removeStats = this->GetMPProblem()->GetStatClass();
    string removeClockName = "Remove Invalid  ";
    removeStats->StartClock(removeClockName);
    RemoveInvalidPortions();
    removeStats->StopClock(removeClockName);
  }
  return ans;
}

template<class MPTraits>
void
ReplanningEvaluation<MPTraits>::SetEnvironment() {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  env->Read(m_envFile.c_str() );
  this->GetMPProblem()->BuildCDStructures();
}

template<class MPTraits>
void
ReplanningEvaluation<MPTraits>::ResetValidity() {
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  for(VertexIterator vi = g->begin(); vi!=g->end(); vi++){
    ((*vi).property()).SetLabel("VALID",false);
    for(AdjEdgeIterator ei =(*vi).begin(); ei!=(*vi).end(); ei++){
      (*ei).property().SetChecked(MAX_INT);
    }
  }
}

template<class MPTraits>
void
ReplanningEvaluation<MPTraits>::RemoveInvalidPortions() {
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  vector <VID> vecRoots = GetRoots();

  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  get_cc_stats(*g, cmap, ccs);

  //Delete CCs other than the one containing roots
  for(typename vector<pair<size_t, VID> >::iterator ccIt = ccs.begin(); ccIt != ccs.end(); ccIt++) {
    bool partOfRootCC = false;
    for(typename vector<VID>::iterator vecIt = vecRoots.begin(); vecIt!=vecRoots.end(); vecIt++) {
      if(stapl::sequential::is_same_cc(*g, cmap, *vecIt, ccIt->second)){// contained in some root's cc
        partOfRootCC = true;
        break;
      }
    }
    if(!partOfRootCC) {
      cmap.reset();
      vector<VID> cciVIDs;
      get_cc(*g, cmap, ccIt->second, cciVIDs);
      for(typename vector<VID>::iterator vecit=cciVIDs.begin(); vecit!=cciVIDs.end(); vecit++) {
        g->delete_vertex(*vecit);
      }
    }
  }
}

template<class MPTraits>
vector<typename ReplanningEvaluation<MPTraits>::VID>
ReplanningEvaluation<MPTraits>::GetRoots() {
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();

  vector<VID> vecRoot;
  for(typename vector<CfgType>::iterator cit1 = this->m_query.begin(), cit2 = cit1+1; cit2!=this->m_query.end(); cit1++, cit2++) {
    if(g->IsVertex(*cit1)) {
      VID qVID = g->GetVID(*cit1);
      vecRoot.push_back(qVID);
    }
  }
  return vecRoot;
}

#endif


#ifndef REPLANNINGEVALUATION_H
#define REPLANNINGEVALUATION_H

#include "MapEvaluatorMethod.h"

template<class MPTraits>
class ReplanningEvaluation : public LazyQuery<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
     typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::VID VID;
    typedef typename GraphType::vertex_iterator vertex_iterator;
    typedef typename GraphType::adj_edge_iterator adj_edge_iterator;

    ReplanningEvaluation();
    ReplanningEvaluation(string _envFile, string _vcLabel, const char* _queryFileName = ""):
      m_envFile(_envFile), LazyQuery<MPTraits>(_queryFileName, _vcLabel){
      this->SetName("ReplanningEvaluation");
    }
    
    ReplanningEvaluation(CfgType _start, CfgType _goal, string _vcLabel) :
            LazyQuery<MPTraits>(_start, _goal, _vcLabel) { this->SetName("ReplanningEvaluation"); }

    ReplanningEvaluation(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~ReplanningEvaluation();
    virtual void PrintOptions(ostream& _os);

    vector<VID> GetRoots();
    virtual bool operator()();

    virtual bool CanRecreatePath(RoadmapType* _rdmp, 
      vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath);

    void SetEnvironment();  
    void RemoveInvalidPortions();
    void ResetValidity();
  protected:
    string m_envFile;
};

template<class MPTraits>
ReplanningEvaluation<MPTraits>::ReplanningEvaluation() {
  this->SetName("ReplanningEvaluation");
  m_envFile = "";
}

template<class MPTraits>
ReplanningEvaluation<MPTraits>::ReplanningEvaluation(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node)
     : LazyQuery<MPTraits>(_problem, _node, false){
     m_envFile = _node.stringXMLParameter("envFile", true, "", "Environment filename");
}

template<class MPTraits>
ReplanningEvaluation<MPTraits>::~ReplanningEvaluation() {
}

template<class MPTraits>
void
ReplanningEvaluation<MPTraits>::PrintOptions(ostream& _os){
  _os << "\n\tEnvironment = " << m_envFile <<endl;
}

template<class MPTraits>
vector< typename ReplanningEvaluation<MPTraits>::VID>
ReplanningEvaluation<MPTraits>::GetRoots(){
  vector<VID> vecRoot;
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph(); 
  typedef typename vector<CfgType>::iterator CIT;

  {
    for(CIT cit1 = this->m_query.begin(), cit2 = cit1+1; cit2!=this->m_query.end(); cit1++, cit2++){
      VID qVID;
      if(g->IsVertex(*cit1) ){
        qVID= g->GetVID(*cit1);
        vecRoot.push_back(qVID);
      }  
    }
  } 
  return vecRoot;
}

template<class MPTraits>
bool
ReplanningEvaluation<MPTraits>::operator()() {
  StatClass* ReplanStatClass = this->GetMPProblem()->GetStatClass();
  string replanClockName = "Replan Evaluator  ";
  ReplanStatClass->StartClock(replanClockName);
  static bool flag = false;
  static int itr=0;
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
 StatClass* RemoveStatClass = this->GetMPProblem()->GetStatClass();
 string removeClockName = "Remove Invalid  ";
 
 if(!ans){
   RemoveStatClass->StartClock(removeClockName);
   RemoveInvalidPortions();
   RemoveStatClass->StopClock(removeClockName);
 }
 return ans;
}

template<class MPTraits>
void
ReplanningEvaluation<MPTraits>::SetEnvironment() {
  Environment* env =this->GetMPProblem()->GetEnvironment() ;
  if(this->m_debug) cout<<"#oldobs:"<<env->GetUsableMultiBodyCount()<<endl;
  env->Read(m_envFile.c_str() );
  this->GetMPProblem()->BuildCDStructures();
  if(this->m_debug) cout<<"#newobs:"<<env->GetUsableMultiBodyCount()<<endl;
}

template<class MPTraits>
void
ReplanningEvaluation<MPTraits>::RemoveInvalidPortions() {
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  vector <VID> vecRoots = GetRoots();

  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  get_cc_stats(*g, cmap, ccs);
  vector<VID> cciVIDs;

  //Delete CCs other than the one containing roots 
  typename vector<pair<size_t, VID> >::iterator ccIt;
  typename vector<VID>::iterator vecIt;
  for(ccIt = ccs.begin(); ccIt != ccs.end(); ccIt++) {
    bool flag = false ;
    for(vecIt = vecRoots.begin(); vecIt!=vecRoots.end(); vecIt++){
      if(stapl::sequential::is_same_cc(*g, cmap, *vecIt, ccIt->second)){// contained in some root's cc
        flag=true;
        break;
      }
    }
    if(!flag){
      cmap.reset();
      cciVIDs.clear();
      get_cc(*g, cmap, ccIt->second, cciVIDs);
      for(typename vector<VID>::iterator vecit=cciVIDs.begin();vecit!=cciVIDs.end();vecit++){
        g->delete_vertex(*vecit);
      }  
    }
  }
}

template<class MPTraits>
void
ReplanningEvaluation<MPTraits>::ResetValidity() {
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  for(vertex_iterator vi = g->begin(); vi!=g->end(); vi++){
    ((*vi).property()).SetLabel("VALID",false);
    for(adj_edge_iterator ei =(*vi).begin(); ei!=(*vi).end(); ei++){
      (*ei).property().SetChecked(MAX_INT);
    }
  }
}

#endif


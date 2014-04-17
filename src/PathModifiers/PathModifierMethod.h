#ifndef PATHMODIFIERMETHOD_H_
#define PATHMODIFIERMETHOD_H_

#include "Utilities/MPUtils.h"
#include "LocalPlanners/LPOutput.h"

template<class MPTraits>
class PathModifierMethod : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename vector<CfgType>::iterator CfgIter;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;

    PathModifierMethod();
    PathModifierMethod(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os) const;

    virtual void Modify(vector<CfgType>& _path, vector<CfgType>& _newPath);

  protected:

    virtual bool ModifyImpl(vector<CfgType>& _path, vector<CfgType>& _newPath) = 0;

    // Helper methods
    void AddToPath(vector<CfgType>& _path, LPOutput<MPTraits>* _lpOutput, CfgType& _end);

    vector<VID> GetPathVIDs(vector<CfgType>& _path, GraphType* _graph);

    void RemoveBranches(const string& _dmLabel, vector<CfgType>& _path, vector<CfgType>& _newPath);
};

// Non-XML Constructor
template<class MPTraits>
PathModifierMethod<MPTraits>::PathModifierMethod() :
  MPBaseObject<MPTraits>() {
    this->SetName("PathModifier");
  }

// XML Constructor
template<class MPTraits>
PathModifierMethod<MPTraits>::PathModifierMethod(MPProblemType* _problem, XMLNodeReader& _node):
  MPBaseObject<MPTraits>(_problem, _node) {
    this->SetName("PathModifier");
    ParseXML(_node);
  }

template<class MPTraits>
void
PathModifierMethod<MPTraits>::ParseXML(XMLNodeReader& _node) {
  this->m_debug = _node.boolXMLParameter("debug", false, false, "Debug mode");
}

template<class MPTraits>
void
PathModifierMethod<MPTraits>::PrintOptions(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

template<class MPTraits>
void
PathModifierMethod<MPTraits>::Modify(vector<CfgType>& _path, vector<CfgType>& _newPath) {
  ModifyImpl(_path, _newPath);
}

// Auxiliary function created to avoid checking emptiness everytime
// Adds the path that is stored in lpOutput to m_path as well as the _end configuration
template<class MPTraits>
void
PathModifierMethod<MPTraits>::AddToPath(vector<CfgType>& _path, LPOutput<MPTraits>* _lpOutput, CfgType& _end) {
  if(!_lpOutput->m_path.empty())
    _path.insert(_path.end(), _lpOutput->m_path.begin(), _lpOutput->m_path.end());
  _path.push_back(_end);
}

// Get pathVIDs from path
template<class MPTraits>
vector<typename MPTraits::MPProblemType::VID>
PathModifierMethod<MPTraits>::GetPathVIDs(vector<CfgType>& _path, GraphType* _graph) {
  vector<VID> pathVIDs;
  for(CfgIter cit = _path.begin(); cit != _path.end(); ++cit) {
    VID v = _graph->GetVID(*cit);
    if(v != INVALID_VID)
      pathVIDs.push_back(v);
  }
  return pathVIDs;
}

template<class MPTraits>
void
PathModifierMethod<MPTraits>::RemoveBranches(const string& _dmLabel, vector<CfgType>& _path, vector<CfgType>& _newPath) {
  _newPath.clear();
  typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(_dmLabel);

  //RemoveBranches Algorithm
  //_path = {q_1, q_2, ..., q_m}
  //for i = 1 -> m
  //  _newPath = _newPath + {q_i}
  //  j <- m
  //  while(d(q_i, q_j) > resolution
  //    j <- j - 1
  //  i <- j
  //return _newPath

  double res = min(env->GetPositionRes(), env->GetOrientationRes());

  typedef typename vector<CfgType>::iterator CIT;
  for(CIT cit = _path.begin(); cit != _path.end(); ++cit) {
    _newPath.push_back(*cit);

    typedef typename vector<CfgType>::reverse_iterator RCIT;
    RCIT rcit = _path.rbegin();
    while(dm->Distance(*cit, *rcit) > res)
      rcit++;

    cit = rcit.base()-1;
  }
}

#endif

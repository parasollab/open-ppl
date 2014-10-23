#ifndef MEDIALAXISPATHMODIFIER_H_
#define MEDIALAXISPATHMODIFIER_H_

#include "PathModifierMethod.h"

#include "LocalPlanners/MedialAxisLP.h"
#include "Utilities/MedialAxisUtilities.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup PathModifiers
/// @brief TODO.
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MedialAxisPathModifier : public PathModifierMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    MedialAxisPathModifier(const string& _pmLabel = "",
        const string& _lpLabel = "", const string& _malpLabel = "");
    MedialAxisPathModifier(MPProblemType* _problem, XMLNodeReader& _node);

    void Print(ostream& _os) const;
    void ParseXML(XMLNodeReader& _node);

    bool ModifyImpl(vector<CfgType>& _path, vector<CfgType>& _newPath);

  private:
    string m_pmLabel;
    string m_lpLabel;
    string m_malpLabel;
};

// Non-XML Constructor
template<class MPTraits>
MedialAxisPathModifier<MPTraits>::MedialAxisPathModifier(
    const string& _pmLabel, const string& _lpLabel, const string& _malpLabel) :
  PathModifierMethod<MPTraits>(), m_pmLabel(_pmLabel), m_lpLabel(_lpLabel),
  m_malpLabel(_malpLabel) {
    this->SetName("MedialAxisPathModifier");
  }

// XML Constructor
template<class MPTraits>
MedialAxisPathModifier<MPTraits>::MedialAxisPathModifier(MPProblemType* _problem, XMLNodeReader& _node) :
  PathModifierMethod<MPTraits>(_problem, _node) {
    this->SetName("MedialAxisPathModifier");
    ParseXML(_node);
  }

template<class MPTraits>
void
MedialAxisPathModifier<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_pmLabel = _node.stringXMLParameter("pmLabel", true, "", "Path Modifier method");
  m_lpLabel = _node.stringXMLParameter("lpLabel", true, "", "Local planner method");
  m_malpLabel = _node.stringXMLParameter("malpLabel", true, "", "Medial axis local planner label needed by MedialAxisPathModifier");
}

template<class MPTraits>
void
MedialAxisPathModifier<MPTraits>::Print(ostream& _os) const {
  PathModifierMethod<MPTraits>::Print(_os);
  _os << "\tpath modifier: \"" << m_pmLabel << "\"" << endl;
  _os << "\tlocal planner: \"" << m_lpLabel << "\"" << endl;
  _os << "\tmedial axis local planner: \"" << m_malpLabel << "\"" << endl;
}


template<class MPTraits>
bool
MedialAxisPathModifier<MPTraits>::ModifyImpl(vector<CfgType>& _path, vector<CfgType>& _newPath) {
  //MAPS Algorithm
  //Input: _path
  //Ouput: _newPath
  //
  //_path' <- pm->ModifyImpl(_path)
  //for all q_i' in _path'
  //  q_i' <- PushToMedialAxis(q_i')
  //_newPath <- emptyset
  //for all (q_i', q_i+1') in _path'
  //  _newPath <- _newPath + MALP(q_i', q_i+1')
  //_newPath <- P(q_1, q_1') + _newPath + P(q_m, q_m')
  //_newPath <- RemoveBranches(_newPath)
  //return _newPath

  if(this->m_debug) cout << "\n*M* Executing MedialAxisPathModifier::Modifier" << endl;

  //Ensure malp is proper type
  MedialAxisLP<MPTraits>* malp = dynamic_cast<MedialAxisLP<MPTraits>*>(
      this->GetMPProblem()->GetLocalPlanner(m_malpLabel).get());
  if(!malp)
    throw PMPLException("Path Modification", WHERE,
        "m_malpLabel: \"" + m_malpLabel + "\" in " + this->GetNameAndLabel() +
        " needs to point to a MedialAxisLP.");

  //smooth path
  vector<CfgType> path;
  typedef typename MPProblemType::PathModifierPointer PathModifierPointer;
  PathModifierPointer pm = this->GetMPProblem()->GetPathModifier(m_pmLabel);
  pm->Modify(_path, path);

  //Ensure path comes from the roadmap
  GraphType* graph = this->GetMPProblem()->GetRoadmap()->GetGraph();
  vector<VID> pathVIDs = this->GetPathVIDs(path, graph);
  if(pathVIDs.empty())
    throw PMPLException("Path Modification", WHERE,
        "pathVIDs in " + this->GetNameAndLabel() + " is empty.");

  typedef typename vector<VID>::iterator VIT;
  typedef typename vector<CfgType>::iterator CIT;

  //Get Cfgs from pathVIDs
  vector<CfgType> pushed;
  for(VIT vit = pathVIDs.begin(); vit != pathVIDs.end(); ++vit)
    pushed.push_back(graph->GetVertex(*vit));

  //Push all the nodes of the path
  MedialAxisUtility<MPTraits>& mau = malp->GetMedialAxisUtility();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  shared_ptr<Boundary> boundary = env->GetBoundary();

  for(CIT cit = pushed.begin(); cit != pushed.end(); ++cit) {
    if(!mau.PushToMedialAxis(*cit, boundary)) {
      if(this->m_debug) cout << "Cfg: " << *cit << " failed to push." << endl;
      return false;
    }
  }

  //Create the variables used to connect the nodes
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel);
  LPOutput<MPTraits> tmpOutput;
  double posRes = env->GetPositionRes();
  double oriRes = env->GetOrientationRes();

  //Make room for the path
  _newPath.clear();

  //Connect the start configuration and its pushed version
  if(!lp->IsConnected(_path.front(), pushed.front(), &tmpOutput, posRes, oriRes, true, true, true)) {
    if(this->m_debug) cout << "Local Planner " << lp->GetNameAndLabel()
      << " could not connect the start to its pushed version" << endl;
    return false;
  }
  _newPath.push_back(_path.front());
  this->AddToPath(_newPath, &tmpOutput, pushed.front());

  //Connect the pushed configurations with MALP
  for(CIT cit1 = pushed.begin(), cit2 = cit1+1; cit2 != pushed.end(); ++cit1, ++cit2) {
    if(malp->LocalPlannerMethod<MPTraits>::IsConnected(*cit1, *cit2, &tmpOutput, posRes, oriRes, true, true, true))
      this->AddToPath(_newPath, &tmpOutput, *cit2);
    else {
      //Failure control measures
      if(this->m_debug)
        cout << malp->GetNameAndLabel() << " failed to connect the pair of nodes (" << *cit1 << ", " << *cit2 << ")" << endl;

      //First FCM: Try with the other local planner
      if(lp->IsConnected(*cit1, *cit2, &tmpOutput, posRes, oriRes, true, true, true))
        this->AddToPath(_newPath, &tmpOutput, *cit2);
      else {
        if(this->m_debug)
          cout << "FCM1: " << lp->GetNameAndLabel() << " failed" << endl;

        //Second FCM: Connect the nodes in the medial axis to the original path and back
        //This FCM should not fail, unless envLP is very badly chosen
        //Copying the old configurations
        int i = cit1 - pushed.begin();
        CfgType oldCfg1 = graph->GetVertex(pathVIDs[i]);
        CfgType oldCfg2 = graph->GetVertex(pathVIDs[i+1]);

        if(lp->IsConnected(*cit1, oldCfg1, &tmpOutput, posRes, oriRes, true, true, true)) {
          this->AddToPath(_newPath, &tmpOutput, oldCfg1);
          if(lp->IsConnected(oldCfg1, oldCfg2, &tmpOutput, posRes, oriRes, true, true, true)) {
            this->AddToPath(_newPath, &tmpOutput, oldCfg2);
            if(lp->IsConnected(oldCfg2, *cit2, &tmpOutput, posRes, oriRes, true, true, true))
              this->AddToPath(_newPath, &tmpOutput, *cit2);
            else {
              if(this->m_debug)
                cout << "*M*\t\tFCM2: Failed" << endl;
              return false;
            }
          }
          else {
            if(this->m_debug)
              cout << "*M*\t\tFCM2: Failed" << endl;
            return false;
          }
        }
        else {
          if(this->m_debug)
            cout << "*M*\t\tFCM2: Failed" << endl;
          return false;
        }
      }
    }
  }

  //Connect the goal configuration and its pushed version
  if(!lp->IsConnected(pushed.back(), _path.back(), &tmpOutput, posRes, oriRes, true, true, true)) {
    if(this->m_debug) cout << "Local Planner " << lp->GetNameAndLabel()
      << " could not connect the goal to its pushed version" << endl;
    return false;
  }
  this->AddToPath(_newPath, &tmpOutput, _path.back());

  vector<CfgType> fpath = _newPath;
  this->RemoveBranches(mau.GetDistanceMetricLabel(), fpath, _newPath);

  return true;
}

#endif

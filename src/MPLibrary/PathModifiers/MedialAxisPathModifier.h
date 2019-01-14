#ifndef MEDIAL_AXIS_PATH_MODIFIER_H_
#define MEDIAL_AXIS_PATH_MODIFIER_H_

#include "PathModifierMethod.h"

#include "MPLibrary/LocalPlanners/MedialAxisLP.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup PathModifiers
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MedialAxisPathModifier : public PathModifierMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    MedialAxisPathModifier(const string& _pmLabel = "",
        const string& _lpLabel = "", const string& _malpLabel = "");

    MedialAxisPathModifier(XMLNode& _node);

    virtual ~MedialAxisPathModifier() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Modifier Interface
    ///@{

    virtual bool ModifyImpl(RoadmapType* _graph, vector<CfgType>& _path, vector<CfgType>& _newPath)
        override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    string m_pmLabel;
    string m_lpLabel;
    string m_malpLabel;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MedialAxisPathModifier<MPTraits>::
MedialAxisPathModifier(const string& _pmLabel, const string& _lpLabel,
    const string& _malpLabel) :
    PathModifierMethod<MPTraits>(), m_pmLabel(_pmLabel), m_lpLabel(_lpLabel),
    m_malpLabel(_malpLabel) {
  this->SetName("MedialAxisPathModifier");
}


template <typename MPTraits>
MedialAxisPathModifier<MPTraits>::
MedialAxisPathModifier(XMLNode& _node) : PathModifierMethod<MPTraits>(_node) {
  this->SetName("MedialAxisPathModifier");
  m_pmLabel = _node.Read("pmLabel", false, "NULL", "Path Modifier method");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local planner method");
  m_malpLabel = _node.Read("malpLabel", true, "",
      "Medial axis local planner label needed by MedialAxisPathModifier");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
MedialAxisPathModifier<MPTraits>::
Print(ostream& _os) const {
  PathModifierMethod<MPTraits>::Print(_os);
  _os << "\tpath modifier: \"" << m_pmLabel << "\"" << endl;
  _os << "\tlocal planner: \"" << m_lpLabel << "\"" << endl;
  _os << "\tmedial axis local planner: \"" << m_malpLabel << "\"" << endl;
}

/*---------------------- PathModifierMethod Overrides ------------------------*/

template <typename MPTraits>
bool
MedialAxisPathModifier<MPTraits>::
ModifyImpl(RoadmapType* _graph, vector<CfgType>& _path, vector<CfgType>& _newPath) {
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

  if(this->m_debug)
    cout << "\n*M* Executing MedialAxisPathModifier::Modifier" << endl;

  //Ensure malp is proper type
  MedialAxisLP<MPTraits>* malp = dynamic_cast<MedialAxisLP<MPTraits>*>(
      this->GetLocalPlanner(m_malpLabel).get());
  if(!malp)
    throw RunTimeException(WHERE,
        "m_malpLabel: \"" + m_malpLabel + "\" in " + this->GetNameAndLabel() +
        " needs to point to a MedialAxisLP.");

  //smooth path
  vector<CfgType> path;
  if(m_pmLabel != "NULL") {
    auto pm = this->GetPathModifier(m_pmLabel);
    pm->Modify(_graph, _path, path);
  }
  else
    path = _path;

  //Ensure path comes from the roadmap
  vector<VID> pathVIDs = this->GetPathVIDs(path, _graph);
  if(pathVIDs.empty())
    throw PMPLException("Path Modification", WHERE,
        "pathVIDs in " + this->GetNameAndLabel() + " is empty.");

  //Get Cfgs from pathVIDs
  vector<CfgType> pushed;
  for(auto vit = pathVIDs.begin(); vit != pathVIDs.end(); ++vit)
    pushed.push_back(_graph->GetVertex(*vit));

  //Push all the nodes of the path
  MedialAxisUtility<MPTraits>& mau = malp->GetMedialAxisUtility();
  Environment* env = this->GetEnvironment();
  auto boundary = env->GetBoundary();

  for(auto cit = pushed.begin(); cit != pushed.end(); ++cit) {
    size_t tries = mau.GetExactClearance() ? 100 : 0;
    bool success = false;
    do {
      success = mau.PushToMedialAxis(*cit, boundary);
    } while(!success && tries++ < 100);
    if(!success && this->m_debug)
      cout << "Cfg: " << *cit
        << " failed to push, keeping original cfg." << endl;
  }

  //Create the variables used to connect the nodes
  auto lp = this->GetLocalPlanner(this->m_lpLabel);
  LPOutput<MPTraits> tmpOutput;
  double posRes = env->GetPositionRes();
  double oriRes = env->GetOrientationRes();

  //Make room for the path
  _newPath.clear();

  //Connect the start configuration and its pushed version
  if(!lp->IsConnected(_path.front(), pushed.front(), &tmpOutput,
        posRes, oriRes, true, true)) {
    if(this->m_debug)
      cout << "Local Planner " << lp->GetNameAndLabel()
        << " could not connect the start to its pushed version" << endl;
    return false;
  }
  _newPath.push_back(_path.front());
  this->AddToPath(_newPath, &tmpOutput, pushed.front());

  //Connect the pushed configurations with MALP
  for(auto cit1 = pushed.begin(), cit2 = cit1 + 1;
      cit2 != pushed.end(); ++cit1, ++cit2) {
    //do attempts for MALP
    size_t tries = mau.GetExactClearance() ? 10 : 0;
    bool success = false;
    do {
      success = malp->LocalPlannerMethod<MPTraits>::IsConnected(*cit1, *cit2,
          &tmpOutput, posRes, oriRes, true, true);
    } while(!success && tries++ < 10);

    //analyze MALP success
    if(success)
      this->AddToPath(_newPath, &tmpOutput, *cit2);
    else {
      //Failure control measures
      if(this->m_debug)
        cout << malp->GetNameAndLabel()
          << " failed to connect the pair of nodes (" << *cit1 << ", "
          << *cit2 << ")" << endl;

      //First FCM: Try with the other local planner
      if(lp->IsConnected(*cit1, *cit2, &tmpOutput,
            posRes, oriRes, true, true))
        this->AddToPath(_newPath, &tmpOutput, *cit2);
      else {
        if(this->m_debug)
          cout << "FCM1: " << lp->GetNameAndLabel() << " failed" << endl;

        //Second FCM: Connect the nodes in the medial axis to the original path
        //and back. This FCM should not fail, unless envLP is very badly chosen
        //Copying the old configurations
        int i = cit1 - pushed.begin();
        CfgType oldCfg1 = _graph->GetVertex(pathVIDs[i]);
        CfgType oldCfg2 = _graph->GetVertex(pathVIDs[i+1]);

        if(lp->IsConnected(*cit1, oldCfg1, &tmpOutput,
              posRes, oriRes, true, true)) {
          this->AddToPath(_newPath, &tmpOutput, oldCfg1);
          if(lp->IsConnected(oldCfg1, oldCfg2, &tmpOutput,
                posRes, oriRes, true, true)) {
            this->AddToPath(_newPath, &tmpOutput, oldCfg2);
            if(lp->IsConnected(oldCfg2, *cit2, &tmpOutput,
                  posRes, oriRes, true, true))
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
  if(!lp->IsConnected(pushed.back(), _path.back(), &tmpOutput,
        posRes, oriRes, true, true)) {
    if(this->m_debug)
      cout << "Local Planner " << lp->GetNameAndLabel()
        << " could not connect the goal to its pushed version" << endl;
    return false;
  }
  this->AddToPath(_newPath, &tmpOutput, _path.back());

  vector<CfgType> fpath = _newPath;
  this->RemoveBranches(mau.GetDistanceMetricLabel(), fpath, _newPath);

  if(this->m_debug)
    cout << "*M* Done, returing true\n";
  return true;
}

/*----------------------------------------------------------------------------*/

#endif

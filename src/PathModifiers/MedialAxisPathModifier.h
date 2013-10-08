#ifndef MEDIALAXISPATHMODIFIER_H_
#define MEDIALAXISPATHMODIFIER_H_

#include "PathModifierMethod.h"

#include "LocalPlanners/MedialAxisLP.h"
#include "Utilities/MedialAxisUtilities.h"

template<class MPTraits>
class MedialAxisPathModifier : public PathModifierMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    MedialAxisPathModifier(const string _dmLabel = "", const string  _lpLabel = "",
        const string _malpLabel = "");
    MedialAxisPathModifier(MPProblemType* _problem, XMLNodeReader& _node);

    void PrintOptions(ostream& _os) const;
    void ParseXML(XMLNodeReader& _node);

    bool ModifyImpl(vector<CfgType>& _originalPath, vector<CfgType>& _newPath);

  private:
    string m_dmLabel;           // Distance metric
    string m_lpLabel;           // Local planner
    string m_malpLabel;

};

// Non-XML Constructor
template<class MPTraits>
MedialAxisPathModifier<MPTraits>::MedialAxisPathModifier(const string _dmLabel,
    const string  _lpLabel, const string _malpLabel) :
  PathModifierMethod<MPTraits>(), m_dmLabel(_dmLabel), m_lpLabel(_lpLabel),
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
  m_dmLabel = _node.stringXMLParameter("dmLabel", false, "", "Distance metric method");
  m_lpLabel = _node.stringXMLParameter("lpLabel", true, "", "Local planner method");
  m_malpLabel = _node.stringXMLParameter("malpLabel", true, "", "Medial axis local planner label needed by MedialAxisPathModifier");
}

template<class MPTraits>
void
MedialAxisPathModifier<MPTraits>::PrintOptions(ostream& _os) const {
  PathModifierMethod<MPTraits>::PrintOptions(_os);
  _os << "\tdistance metric = \"" << m_dmLabel << "\"" << endl;
  _os << "\tlocal planner = \"" << m_lpLabel << "\"" << endl;
  _os << "\tmedial axis local planner = \"" << m_malpLabel << "\"" << endl;
}


template<class MPTraits>
bool
MedialAxisPathModifier<MPTraits>::ModifyImpl(vector<CfgType>& _originalPath, vector<CfgType>& _newPath) {
  if(this->m_debug) cout << "\n*M* Executing MedialAxisPathModifier::Modifier" << endl;

  GraphType* graph = this->GetMPProblem()->GetRoadmap()->GetGraph();

  vector<VID> pathVIDs = this->GetPathVIDs(_originalPath, graph);

  bool result = false;

  if(!pathVIDs.empty()) { //Suppose to be checked by the previous call of PathShortcuting::Modifier()
    LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel);
    Environment* env = this->GetMPProblem()->GetEnvironment();
    StatClass* stats = this->GetMPProblem()->GetStatClass();
    DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);
    LPOutput<MPTraits> tmpOutput;

    MedialAxisLP<MPTraits>* malp = dynamic_cast<MedialAxisLP<MPTraits>*>(
        this->GetMPProblem()->GetLocalPlanner(m_malpLabel).get());
    shared_ptr<Boundary> bBox = this->GetMPProblem()->GetEnvironment()->GetBoundary();

    if(malp) {
      result = true;
      MedialAxisUtility<MPTraits>& mau = malp->GetMedialAxisUtility();  

      if(this->m_recordKeep)
        stats->StartClock("Medial Axis Path Smoother");
      size_t n = pathVIDs.size();

      //Copy all the nodes from pathVIDs to avoid modifying them
      vector<CfgType> pushedNodes;
      size_t i;
      for(i = 0; i < n; ++i)
        pushedNodes.push_back(graph->GetVertex(pathVIDs[i]));

      //Copying the start and goal nodes
      CfgType& start = pushedNodes[0];
      CfgType& goal = pushedNodes[n-1];

      //Push all the nodes of the path
      i = 0;
      while(result && i < n) {
        result = mau.PushToMedialAxis(pushedNodes[i], bBox);
        //Print a debug statement if succeeded.
        if(this->m_debug && result)
          cout << "*M* Node " << i << " successfully pushed" << endl;
        ++i;
      }
      if(result) {//If all the nodes were pushed correctly
        //Create the variables used to connect the nodes
        double posRes = env->GetPositionRes();
        double oriRes = env->GetOrientationRes();

        //Make room for the path
        _newPath.clear();

        //Connect the start configuration and its pushed version
        result = lp->IsConnected(env, *stats, dm, start, pushedNodes[0], &tmpOutput, posRes, oriRes, true, true, true);
        if(result) {
          _newPath.push_back(start);
          this->AddToPath(_newPath, &(tmpOutput), pushedNodes[0]);

          //Connect the nodes that are already in the medial axis
          i = 1;
          while(result && i < n) {
            result = malp->LocalPlannerMethod<MPTraits>::IsConnected(env, *stats, dm, pushedNodes[i-1], pushedNodes[i], &tmpOutput, posRes, oriRes, true, true, true);
            if(result) {
              this->AddToPath(_newPath, &tmpOutput, pushedNodes[i]);
            }
            else{ //Failure control measures (FCM)
              if(this->m_debug)
                cout << "*M*\t" << malp->GetNameAndLabel() << " failed to connect the pair of nodes (" << (i-1) << ", " << i << ")" << endl
                << "*M*\tAttempting Failure Control Measures (FCM):" << endl;
              //First FCM: Try with the other local planner
              result = lp->IsConnected(env, *stats, dm, pushedNodes[i-1], pushedNodes[i], &tmpOutput, posRes, oriRes, true, true, true);
              if(result) {
                if(this->m_debug)
                  cout << "*M*\t\tFCM1: " << lp->GetNameAndLabel() << " succeeded" << endl;
                this->AddToPath(_newPath, &(tmpOutput), pushedNodes[i]);
              }
              else{
                if(this->m_debug)
                  cout << "*M*\t\tFCM1: " << lp->GetNameAndLabel() << " also failed" << endl;
                //Second FCM: Connect the nodes in the medial axis to the original path and back
                //This FCM should not fail, unless envLP is very badly chosen
                //Copying the old configurations
                CfgType oldCfg1 = graph->GetVertex(pathVIDs[i-1]);
                CfgType oldCfg2 = graph->GetVertex(pathVIDs[i]);
                result = lp->IsConnected(env, *stats, dm, pushedNodes[i-1], oldCfg1, &tmpOutput, posRes, oriRes, true, true, true);
                if(result) {
                  if(this->m_debug)
                    cout << "*M*\t\tFCM2: First connection established" << endl;
                  this->AddToPath(_newPath, &(tmpOutput), oldCfg1);
                  result = lp->IsConnected(env, *stats, dm, oldCfg1, oldCfg2, &tmpOutput, posRes, oriRes, true, true, true);
                }
                if(result) {
                  if(this->m_debug)
                    cout << "*M*\t\tFCM2: Second connection established" << endl;
                  this->AddToPath(_newPath, &(tmpOutput), oldCfg2);
                  result = lp->IsConnected(env, *stats, dm, oldCfg2, pushedNodes[i], &tmpOutput, posRes, oriRes, true, true, true);
                }
                if(result) {
                  if(this->m_debug)
                    cout << "*M*\t\tFCM2: Succeeded" << endl;
                  this->AddToPath(_newPath, &(tmpOutput), pushedNodes[i]);
                }
                else{
                  if(this->m_debug)
                    cout << "*M*\t\tFCM2: Failed" << endl;
                }
              }
            }
            ++i;
          }
          if(result) {//If all the intermediate connections succeeded
            //Connect the goal configuration and its pushed version
            result = lp->IsConnected(env, *stats, dm, pushedNodes[n-1], goal, &tmpOutput, posRes, oriRes, true, true, true);
            if(result) {
              //Add to path
              this->AddToPath(_newPath, &tmpOutput, goal);
              /*TODO
               * Remove Branches Algorithm (from _newPath)
               * */
            }
          }
          else{
            //If this happens, another lpLabel should be used.
            //vcLabel in malpLabel could be causing trouble too
            if(this->m_debug)
              cout << "*M* \tSecond failure control measure also failed" << endl;
          }
        }
        else{
          //If this happens, another lpLabel should be used.
          //A medial axis local planner is not a good choice when the start and goal are not on the medial axis.
          if(this->m_debug)
            cout << "*M* Local Planner " << lp->GetNameAndLabel()
            << " could not connect the start to its pushed version" << endl;
        }
      }
      else{
        if(this->m_debug)
          cout << "*M* Could not push the configuration in pathVIDs[" << (i-1) << "]" << endl;
      }
      if(this->m_recordKeep) stats->StopClock("Medial Axis Path Smoother");
    }
    else{
      cerr << "*M* m_malpLabel = \"" << m_malpLabel << "\" in " << this->GetNameAndLabel()
        << " needs to point to a MedialAxisLP. Will not execute MedialAxisSmooth()." << endl;
    }
  }
  else{
    cerr << "*M* Error. pathVIDs in " << this->GetNameAndLabel() << " is empty. Aborting smoothing operation(s)." << endl;
  }
  return !result;
}

#endif

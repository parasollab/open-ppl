#ifndef SHORTCUTTINGPATHMODIFIER_H_
#define SHORTCUTTINGPATHMODIFIER_H_

#include "PathModifierMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup PathModifiers
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ShortcuttingPathModifier : public PathModifierMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    ShortcuttingPathModifier(const string& _dmLabel = "",
        const string& _lpLabel = "");
    ShortcuttingPathModifier(MPProblemType* _problem, XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    bool ModifyImpl(vector<CfgType>& _path, vector<CfgType>& _newPath);

  private:
    string m_lpLabel; // Local planner
};

template<class MPTraits>
ShortcuttingPathModifier<MPTraits>::
ShortcuttingPathModifier(const string& _dmLabel, const string& _lpLabel) :
  PathModifierMethod<MPTraits>(), m_lpLabel(_lpLabel) {
    this->SetName("ShortcuttingPathModifier");
  }

template<class MPTraits>
ShortcuttingPathModifier<MPTraits>::
ShortcuttingPathModifier(MPProblemType* _problem, XMLNode& _node) :
  PathModifierMethod<MPTraits>(_problem, _node) {
    this->SetName("ShortcuttingPathModifier");
    ParseXML(_node);
  }

template<class MPTraits>
void
ShortcuttingPathModifier<MPTraits>::
ParseXML(XMLNode& _node) {
  m_lpLabel = _node.Read("lpLabel", true, "", "Local planner method");
}

template<class MPTraits>
void
ShortcuttingPathModifier<MPTraits>::
Print(ostream& _os) const {
  PathModifierMethod<MPTraits>::Print(_os);
  _os << "\tlocal planner = \"" << m_lpLabel << "\"" << endl;
}

// Shortens the path by skipping nodes with a greedy approach.
// This function is also supposed to be used before MedialAxisSmooth.
template<class MPTraits>
bool
ShortcuttingPathModifier<MPTraits>::
ModifyImpl(vector<CfgType>& _path, vector<CfgType>& _newPath) {

  if(this->m_debug) cout << "\n*S* Executing ShortcuttingPathModifier::Modifier" << endl;

  GraphType* graph = this->GetRoadmap()->GetGraph();

  vector<VID> originalPathVIDs = this->GetPathVIDs(_path, graph);

  bool smoothFileOutput = false;

  if(!originalPathVIDs.empty()) {
    smoothFileOutput = true;

    LocalPlannerPointer lp = this->GetLocalPlanner(m_lpLabel);
    Environment* env = this->GetEnvironment();
    StatClass* stats = this->GetStatClass();
    LPOutput<MPTraits> tmpOutput;

    bool reconstruct, skip;
    double posRes = env->GetPositionRes();
    double oriRes = env->GetOrientationRes();

    stats->StartClock("Path Modifier");
    //This variable will store how many nodes were skipped
    size_t skips = 0;

    vector<VID> newPathVIDs;
    // Make room for the path
    _newPath.clear();

    //Save the first node in the path. (i.e. it can never be skipped)
    newPathVIDs.push_back(originalPathVIDs[0]);
    _newPath.push_back(graph->GetVertex(originalPathVIDs[0]));

    size_t n = originalPathVIDs.size();
    size_t i = 0;   //i is the index of the start VID (configuration)
    size_t j = n-1; //j is the index of the goal VID (configuration)
    while(i < j) {
      if((i+1) == j) {//If the vertices are adjacent, there are no nodes to skip
        //Only reconstruct and save path if smoothing output is wanted.
        if(smoothFileOutput) {
          reconstruct = lp->IsConnected(graph->GetVertex(originalPathVIDs[i]),
              graph->GetVertex(originalPathVIDs[j]), &tmpOutput,
              posRes, oriRes, true, true);
          this->AddToPath(_newPath, &tmpOutput,
              graph->GetVertex(originalPathVIDs[j]));
          //If could not reconstruct, abort the output,
          //but continue skipping nodes
          if(!reconstruct) {
            cerr << "*S* Error. Could not reconstruct path in ShortcuttingPathModifier::Modifier()" << endl;
            smoothFileOutput = false;
            if(_newPath.empty())
              cerr << "*S*\t_newPath is empty. Cannot write anything" << endl;
            else{
              cerr << "*S*\tWill output failed path to \"error.smooth.path\"" << endl;
              WritePath("error.smooth.path", _newPath);
              _newPath.clear();
            }
            return false;
          }
        }
        newPathVIDs.push_back(originalPathVIDs[j]);
        i = j;
        j = n-1;
      }
      else {
        //Try to make a connection by skipping nodes betwen i and j
        skip = lp->IsConnected(graph->GetVertex(originalPathVIDs[i]),
            graph->GetVertex(originalPathVIDs[j]), &tmpOutput,
            posRes, oriRes, true, true);
        if(skip) {
          if(smoothFileOutput)
            this->AddToPath(_newPath, &tmpOutput,
                graph->GetVertex(originalPathVIDs[j]));
          newPathVIDs.push_back(originalPathVIDs[j]);
          skips += (j-i-1);
          i = j;
          j = n-1;
        }
        else {
          //Could not skip. Try the previous node.
          --j;
        }
      }
    }

    if(this->m_debug) {
      if(skips == 0)
        cout << "*S* Could not skip any nodes" << endl;
      else{
        cout << "*S* Smoothing operation skipped " << skips;
        if(skips == 1)
          cout << " node" << endl;
        else
          cout << " nodes" << endl;
      }
    }

    stats->StopClock("Path Modifier");

    return true;
  }
  else{
    cerr << "*S* Error. originalPathVIDs in " << this->GetNameAndLabel()
      << " is empty. Aborting smoothing operation(s)." << endl;
    return false;
  }
}

#endif

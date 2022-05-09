#ifndef SHORT_CUTTING_PATH_MODIFIER_H_
#define SHORT_CUTTING_PATH_MODIFIER_H_

#include "PathModifierMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// A path modifier that starts at the beginning of the 
/// given path, and greedly searches for the biggest possible
/// shortcut going backwards from the last vertex. When a
/// shortcut is found, the next shortcut is found in the
/// same way.
///
/// @ingroup PathModifiers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ShortcuttingPathModifier : virtual public PathModifierMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ShortcuttingPathModifier(const string& _dmLabel = "",
        const string& _lpLabel = "");

    ShortcuttingPathModifier(XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    bool ModifyImpl(RoadmapType* _graph, vector<VID>& _path) override;

  private:
    string m_lpLabel; // Local planner
};

template <typename MPTraits>
ShortcuttingPathModifier<MPTraits>::
ShortcuttingPathModifier(const string& _dmLabel, const string& _lpLabel) :
  PathModifierMethod<MPTraits>(), m_lpLabel(_lpLabel) {
    this->SetName("ShortcuttingPathModifier");
  }

template <typename MPTraits>
ShortcuttingPathModifier<MPTraits>::
ShortcuttingPathModifier(XMLNode& _node) :
  PathModifierMethod<MPTraits>(_node) {
    this->SetName("ShortcuttingPathModifier");
    ParseXML(_node);
  }

template <typename MPTraits>
void
ShortcuttingPathModifier<MPTraits>::
ParseXML(XMLNode& _node) {
  m_lpLabel = _node.Read("lpLabel", true, "", "Local planner method");
}

template <typename MPTraits>
void
ShortcuttingPathModifier<MPTraits>::
Print(ostream& _os) const {
  PathModifierMethod<MPTraits>::Print(_os);
  _os << "\tlocal planner = \"" << m_lpLabel << "\"" << endl;
}

// Shortens the path by skipping nodes with a greedy approach.
// This function is also supposed to be used before MedialAxisSmooth.
template <typename MPTraits>
bool
ShortcuttingPathModifier<MPTraits>::
ModifyImpl(RoadmapType* _graph, vector<VID>& _path) {
  if(this->m_debug) cout << "\n*S* Executing ShortcuttingPathModifier::Modifier" << endl;

  auto graph = _graph ? _graph : this->GetRoadmap();

  if(!_path.empty()) {

    auto lp = this->GetLocalPlanner(m_lpLabel);
    Environment* env = this->GetEnvironment();
    StatClass* stats = this->GetStatClass();
    LPOutput<MPTraits> tmpOutput;

    bool skip = false;
    double posRes = env->GetPositionRes();
    double oriRes = env->GetOrientationRes();

    stats->StartClock("Path Modifier");
    //This variable will store how many nodes were skipped
    size_t skips = 0;

    /// Temporary container for the new PAth
    vector<VID> newPath;

    //Save the first node in the path. (i.e. it can never be skipped)
    newPath.push_back(_path[0]);

    size_t n = _path.size();
    size_t i = 0;   //i is the index of the start VID (configuration)
    size_t j = n-1; //j is the index of the goal VID (configuration)
    while(i < j) {
      if(i < (j-1)) {// Vertices are not adjacent - attempt a skip     
        skip = lp->IsConnected(graph->GetVertex(_path[i]),
            graph->GetVertex(_path[j]), &tmpOutput,
            posRes, oriRes, true, false);
      }
      /// If we can skip or the vertices are adjacent
      ///   (and therefore connected by an edge in the path)
      if (skip || (i == (j-1))) {
        newPath.push_back(_path[j]);
          skips += ((j-1)-i);
          i = j;
          j = n-1;
      }
      else // can't skip - attempt a smaller skip
        --j;
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

    _path = newPath; // Replace the original path with the mpdified path

    return skips > 0; // true if at least one vertex in the path was skipped
  }
  else {
    cerr << "*S* Error. _path in " << this->GetNameAndLabel()
      << " is empty. Aborting smoothing operation(s)." << endl;
    return false;
  }

}

#endif

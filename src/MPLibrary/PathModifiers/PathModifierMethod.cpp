#include "PathModifierMethod.h"

#include "MPLibrary/MPLibrary.h"


/*------------------------------ Construction --------------------------------*/

PathModifierMethod::
PathModifierMethod(XMLNode& _node) : MPBaseObject(_node) {
  m_pathFile = _node.Read("pathFile", false, "", "Smoothed path filename");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

void
PathModifierMethod::
Print(ostream& _os) const {
  MPBaseObject::Print(_os);
  _os << "\tpath file: \"" << m_pathFile << "\"" << endl;
}

/*--------------------------- PathModifier Interface -------------------------*/

void
PathModifierMethod::
Modify(vector<Cfg>& _path, vector<Cfg>& _newPath) {
  ModifyImpl(this->GetRoadmap(), _path, _newPath);
}


void
PathModifierMethod::
Modify(RoadmapType* _graph, vector<Cfg>& _path, vector<Cfg>& _newPath) {
  ModifyImpl(_graph, _path, _newPath);
}


void
PathModifierMethod::
AddToPath(vector<Cfg>& _path, LPOutput* _lpOutput, Cfg& _end) {
  if(!_lpOutput->m_path.empty())
    _path.insert(_path.end(), _lpOutput->m_path.begin(),
        _lpOutput->m_path.end());
  _path.push_back(_end);
}


std::vector<size_t>
PathModifierMethod::
GetPathVIDs(vector<Cfg>& _path, RoadmapType* _graph) {
  vector<VID> pathVIDs;
  for(auto&  cfg : _path) {
    VID v = _graph->GetVID(cfg);
    if(v != INVALID_VID)
      pathVIDs.push_back(v);
  }
  return pathVIDs;
}


void
PathModifierMethod::
RemoveBranches(const string& _dmLabel, vector<Cfg>& _path,
    vector<Cfg>& _newPath) {
  _newPath.clear();

  Environment* env = this->GetEnvironment();
  auto dm = this->GetMPLibrary()->GetDistanceMetric(_dmLabel);

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

  for(auto cit = _path.begin(); cit != _path.end(); ++cit) {
    _newPath.push_back(*cit);

    auto rcit = _path.rbegin();
    while(dm->Distance(*cit, *rcit) > res)
      rcit++;

    //when q_i != q_j,
    //push q_j onto the new path to avoid skipping it in the loop
    if(cit != rcit.base()-1)
      _newPath.push_back(*rcit);

    cit = rcit.base()-1;
  }
  //the loop doesn't push the goal of the path, be sure to do it
  _newPath.push_back(_path.back());
}

/*----------------------------------------------------------------------------*/

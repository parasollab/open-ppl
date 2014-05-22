#ifndef PATHMODIFIERMETHOD_H_
#define PATHMODIFIERMETHOD_H_

#include	<string>
#include	<iostream>

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

    PathModifierMethod(const string _pathFile = "");
    PathModifierMethod(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void Print(ostream& _os) const;

    virtual void Modify(vector<CfgType>& _originalPath, vector<CfgType>& _newPath);

  protected:
    string m_pathFile;        // Where to write the smoothed path

    virtual bool ModifyImpl(vector<CfgType>& _originalPath, vector<CfgType>& _newPath) =0;

    // Helper methods
    void AddToPath(vector<CfgType>& _path, LPOutput<MPTraits>* _lpOutput, CfgType& _end);
    vector<VID> GetPathVIDs(vector<CfgType>& _path, GraphType* _graph);

  private:
    void OutputPath(vector<CfgType>& _path);
};

// Non-XML Constructor
template<class MPTraits>
PathModifierMethod<MPTraits>::PathModifierMethod(const string _pathFile) :
  MPBaseObject<MPTraits>(), m_pathFile(_pathFile) {
    this->SetName("PathModifier");
    m_pathFile = "";
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
  m_pathFile = _node.stringXMLParameter("pathFile", false, "", "Smoothed path filename");
  this->m_debug = _node.boolXMLParameter("debug", false, false, "Debug mode");
}

template<class MPTraits>
void
PathModifierMethod<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
  _os << "\tpath file = \"" << m_pathFile << "\"" << endl;
}

template<class MPTraits>
void
PathModifierMethod<MPTraits>::Modify(vector<CfgType>& _originalPath, vector<CfgType>& _newPath) {
  bool error = ModifyImpl(_originalPath, _newPath);
  if(!error)
    OutputPath(_newPath);
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
  VID v;
  for(CfgIter cit = _path.begin(); cit != _path.end(); ++cit) {
    v = _graph->GetVID(*cit);
    if(v != INVALID_VID)
      pathVIDs.push_back(_graph->GetVID(*cit));
  }
  return pathVIDs;
}

//Output the path
template<class MPTraits>
void
PathModifierMethod<MPTraits>::OutputPath(vector<CfgType>& _path) {
  if(this->m_pathFile == "") {
    cerr << "*PathModifier* Warning: no path file specified. Outputting modified path to \"modified.path\"." << endl;
    WritePath("modified.path", _path);
  }
  else{
    if(this->m_debug) cout << "*PathModifier* Writing modified path into \"" << this->m_pathFile << "\"" << endl;
    WritePath(this->m_pathFile, _path);
  }
}

#endif

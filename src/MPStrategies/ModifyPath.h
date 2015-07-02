#ifndef MODIFY_PATH_H_
#define MODIFY_PATH_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ModifyPath : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ModifyPath(const string& _pathFile = "", const string& _mapFile = "",
        const string& _pmLabel = "");
    ModifyPath(MPProblemType* _problem, XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

  protected:
    string m_pathFile;
    string m_mapFile;
    string m_pmLabel;

    vector<CfgType> m_path, m_smoothPath;
};

template<class MPTraits>
ModifyPath<MPTraits>::
ModifyPath(const string& _pathFile, const string& _mapFile,
    const string& _pmLabel) :
  m_pathFile(_pathFile), m_mapFile(_mapFile), m_pmLabel(_pmLabel) {
    this->SetName("ModifyPath");
  }

template<class MPTraits>
ModifyPath<MPTraits>::
ModifyPath(MPProblemType* _problem, XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
    this->SetName("ModifyPath");
    m_pathFile = _node.Read("pathFile", true, "", "Path Filename");
    m_mapFile = _node.Read("mapFile", false, "", "Map Filename");
    m_pmLabel = _node.Read("pmLabel", true, "", "Path modifier label");
  }

template<class MPTraits>
void
ModifyPath<MPTraits>::
Print(ostream& _os) const {
  _os << "In Query file: " << m_pathFile << endl;
  _os << "Path Modifier: " << m_pmLabel << endl;
}

template<class MPTraits>
void
ModifyPath<MPTraits>::
Initialize() {
  //read in the path at run-time
  if(!FileExists(m_pathFile))
    throw ParseException(WHERE,
        "Path file '" + m_pathFile + "' does not exist.");

  ifstream ifs(m_pathFile.c_str());
  string tmp;
  getline(ifs, tmp);
  getline(ifs, tmp);
  getline(ifs, tmp);
  CfgType c;
  while(ifs >> c)
    m_path.push_back(c);

  if(!m_mapFile.empty()) {
    if(!FileExists(m_mapFile))
      throw ParseException(WHERE,
          "Map file '" + m_mapFile + "' does not exist.");

    this->GetRoadmap()->Read(m_mapFile);
  }
}

template<class MPTraits>
void
ModifyPath<MPTraits>::
Iterate() {
  //smooth the path
  StatClass* stats = this->GetStatClass();
  stats->StartClock(this->GetNameAndLabel());
  if(m_pmLabel == "")
    m_smoothPath = m_path;
  else
    this->GetPathModifier(m_pmLabel)->Modify(m_path, m_smoothPath);
  stats->StopClock(this->GetNameAndLabel());
}

template<class MPTraits>
void
ModifyPath<MPTraits>::Finalize() {
  //output smoothed path
  string outPathFile = this->GetBaseFilename() + ".smooth.path";
  WritePath(outPathFile, m_smoothPath);
}

#endif

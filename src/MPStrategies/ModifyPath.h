#ifndef MODIFY_PATH_H_
#define MODIFY_PATH_H_

#include "MPStrategyMethod.h"

template<class MPTraits>
class ModifyPath : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ModifyPath(const string& _pathFile = "", const string& _mapFile = "", const string& _pmLabel = "");
    ModifyPath(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void PrintOptions(ostream& _os) const;

  protected:
    string m_pathFile;
    string m_mapFile;
    string m_pmLabel;
    ClearanceUtility<MPTraits> m_clearanceUtility;

    vector<CfgType> m_path, m_smoothPath;
};

template<class MPTraits>
ModifyPath<MPTraits>::ModifyPath(const string& _pathFile, const string& _mapFile, const string& _pmLabel) :
  m_pathFile(_pathFile), m_mapFile(_mapFile), m_pmLabel(_pmLabel) {
    this->SetName("ModifyPath");
  }

template<class MPTraits>
ModifyPath<MPTraits>::ModifyPath(MPProblemType* _problem, XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node), m_clearanceUtility(_problem, _node) {
    this->SetName("ModifyPath");
    m_pathFile = _node.stringXMLParameter("pathFile", true, "", "Path Filename");
    m_mapFile = _node.stringXMLParameter("mapFile", false, "", "Map Filename");
    m_pmLabel = _node.stringXMLParameter("pmLabel", true, "", "Path modifier label");
    _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
void
ModifyPath<MPTraits>::PrintOptions(ostream& _os) const {
  _os << "In Query file: " << m_pathFile << endl;
  _os << "Path Modifier: " << m_pmLabel << endl;
}

//////////////////////
//Initialization Phase
/////////////////////
template<class MPTraits>
void
ModifyPath<MPTraits>::Initialize() {
  if(this->m_debug) cout<<"\nInitializing ModifyPath::"<<endl;

  //read in the path at run-time
  if(!FileExists(m_pathFile))
    throw ParseException(WHERE, "Path file '" + m_pathFile + "' does not exist.");

  ifstream ifs(m_pathFile.c_str());
  string tmp;
  getline(ifs, tmp);
  getline(ifs, tmp);
  getline(ifs, tmp);
  CfgType c;
  while(ifs >> c) {
    m_path.push_back(c);
  }

  if(!m_mapFile.empty()) {
    if(!FileExists(m_mapFile))
      throw ParseException(WHERE, "Map file '" + m_mapFile + "' does not exist.");

    this->GetMPProblem()->GetRoadmap()->Read(m_mapFile);
  }

  if(this->m_debug) cout<<"\nEnding Initializing ModifyPath"<<endl;
}

////////////////
//Run/Start Phase
////////////////
template<class MPTraits>
void
ModifyPath<MPTraits>::Run() {
  if(this->m_debug) cout << "\nRunning ModifyPath::" << endl;

  //smooth the path
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stats->StartClock(this->GetNameAndLabel());
  if(m_pmLabel == "")
    m_smoothPath = m_path;
  else
    this->GetMPProblem()->GetPathModifier(m_pmLabel)->Modify(m_path, m_smoothPath);
  stats->StopClock(this->GetNameAndLabel());
}

/////////////////////
//Finalization phase
////////////////////
template<class MPTraits>
void
ModifyPath<MPTraits>::Finalize() {
  if(this->m_debug) cout<<"\nFinalizing ModifyPath::"<<endl;

  //output smoothed path
  string outPathFile = this->GetBaseFilename() + ".smooth.path";
  WritePath(outPathFile, m_smoothPath);

  //output stats
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  string str = this->GetBaseFilename() + ".stat";
  ofstream osStat(str.c_str());
  osStat << "\n\nSmoothing Stats for " << this->GetNameAndLabel() << endl << endl;
  //stats->PrintAllStats(osStat, this->GetMPProblem()->GetRoadmap());
  stats->PrintClock(this->GetNameAndLabel(), osStat);

  ClearanceStats pathStats = m_clearanceUtility.PathClearance(m_smoothPath);
  osStat << "PathAvgClr\t" << pathStats.m_avg << endl;
  osStat << "PathMinClr\t" << pathStats.m_min << endl;
  osStat << "PathMaxClr\t" << pathStats.m_max << endl;
  osStat << "PathVarClr\t" << pathStats.m_var << endl;
  osStat << "PathLength\t" << pathStats.m_pathLength << endl;

  osStat.close();

  if(this->m_debug) cout<<"\nEnd Finalizing ModifyPath"<<endl;
}

#endif

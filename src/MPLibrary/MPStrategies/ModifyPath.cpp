#include "ModifyPath.h"

#include "MPLibrary/MPLibrary.h"


ModifyPath::
ModifyPath(const std::string& _pathFile, const std::string& _mapFile,
    const std::string& _pmLabel) :
  m_pathFile(_pathFile), m_mapFile(_mapFile), m_pmLabel(_pmLabel) {
    this->SetName("ModifyPath");
}


ModifyPath::
ModifyPath(XMLNode& _node) :
  MPStrategyMethod(_node) {
    this->SetName("ModifyPath");
    m_pathFile = _node.Read("pathFile", true, "", "Path Filename");
    m_mapFile = _node.Read("mapFile", true, "", "Map Filename");
    m_pmLabel = _node.Read("pmLabel", true, "", "Path modifier label");
}


void
ModifyPath::
Print(std::ostream& _os) const {
  _os << "In Query file: " << m_pathFile << std::endl;
  _os << "Path Modifier: " << m_pmLabel << std::endl;
}


void
ModifyPath::
Initialize() {
  //read in the path at run-time
  if(!FileExists(m_pathFile))
    throw ParseException(WHERE,
        "Path file '" + m_pathFile + "' does not exist.");

  ifstream ifs(m_pathFile.c_str());
  std::string tmp;
  getline(ifs, tmp);
  getline(ifs, tmp);
  getline(ifs, tmp);
  Cfg c(this->GetTask()->GetRobot());
  while(ifs >> c)
    m_path.push_back(c);

  if(!m_mapFile.empty()) {
    if(!FileExists(m_mapFile))
      throw ParseException(WHERE,
          "Map file '" + m_mapFile + "' does not exist.");

    //this->GetRoadmap()->Read(m_mapFile);
    Read(this->GetRoadmap(), m_mapFile);
  }
}


void
ModifyPath::
Iterate() {
  //smooth the path
  StatClass* stats = this->GetStatClass();
  stats->StartClock(this->GetNameAndLabel());
  if(m_pmLabel == "")
    m_smoothPath = m_path;
  else
    this->GetMPLibrary()->GetPathModifier(m_pmLabel)->Modify(m_path, m_smoothPath);
  stats->StopClock(this->GetNameAndLabel());
}


void
ModifyPath::Finalize() {
  //output smoothed path
  std::string outPathFile = this->GetBaseFilename() + ".smooth.path";
  WritePath(outPathFile, m_smoothPath);
}

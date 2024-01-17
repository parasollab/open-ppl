#include "PrintMapEvaluation.h"


PrintMapEvaluation::
PrintMapEvaluation() {
  this->SetName("PrintMapEvaluation");
}


PrintMapEvaluation::
PrintMapEvaluation(string _baseName)
  : m_baseName(_baseName) {
  this->SetName("PrintMapEvaluation");
}


PrintMapEvaluation::
PrintMapEvaluation(XMLNode& _node)
  : MapEvaluatorMethod(_node) {
  this->SetName("PrintMapEvaluation");
  m_baseName = _node.Read("base_name", true, "", "base filename for map output");
}


PrintMapEvaluation::
~PrintMapEvaluation() { }


void
PrintMapEvaluation::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
  _os << "\tbase filename = " << m_baseName << endl;
}

/// Write the map to a file
bool
PrintMapEvaluation::
operator()() {
  int numNodes = this->GetRoadmap()->get_num_vertices();
  int numEdges = this->GetRoadmap()->get_num_edges();
  ostringstream osName;
  osName << m_baseName << "." << numNodes << "." << numEdges << ".map";
  this->GetRoadmap()->Write(osName.str(), this->GetEnvironment());

  int numCollNodes = this->GetBlockRoadmap()->get_num_vertices();
  int numCollEdges = this->GetBlockRoadmap()->get_num_edges();
  if(numCollNodes) {
    ostringstream osCollName;
    osCollName << m_baseName << "." << numCollNodes << "." << numCollEdges << ".block.map";
    this->GetBlockRoadmap()->Write(osCollName.str(), this->GetEnvironment());
  }

  return true;
}

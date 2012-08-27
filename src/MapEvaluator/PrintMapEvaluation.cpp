#include "PrintMapEvaluation.h"
#include "Roadmap.h"

PrintMapEvaluation::PrintMapEvaluation() {
  this->SetName("PrintMapEvaluation");
}

PrintMapEvaluation::PrintMapEvaluation(string _baseName)
  : m_baseName(_baseName) {
  this->SetName("PrintMapEvaluation");
}

PrintMapEvaluation::PrintMapEvaluation(XMLNodeReader& _node, MPProblem* _problem)
  : MapEvaluationMethod(_node, _problem) {
  this->SetName("PrintMapEvaluation");
  m_baseName = _node.stringXMLParameter("base_name", true, "", "base filename for map output");

  if(m_debug) PrintOptions(cout);
}

PrintMapEvaluation::~PrintMapEvaluation() {
}

void PrintMapEvaluation::PrintOptions(ostream& _os) {
  _os << "PrintMapEvalaution" << endl;
  _os << "\tbase filename = " << m_baseName << endl;
}

bool PrintMapEvaluation::operator()() {
  int numNodes = GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices();
  int numEdges = GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_edges();
  int numCollNodes = GetMPProblem()->GetBlockRoadmap()->m_pRoadmap->get_num_vertices();
  int numCollEdges = GetMPProblem()->GetBlockRoadmap()->m_pRoadmap->get_num_edges();

  ostringstream osName;
  osName << m_baseName << "." << numNodes << "." << numEdges << ".map";
  ostringstream osCollName;
  osCollName << m_baseName << "." << numCollNodes << "." << numCollEdges << ".block.map";

  ofstream osMap(osName.str().c_str());
  GetMPProblem()->WriteRoadmapForVizmo(osMap);
  osMap.close();
  ofstream osCollMap(osCollName.str().c_str());
  GetMPProblem()->WriteRoadmapForVizmo(osCollMap, NULL, true);
  osCollMap.close();

  return true;
}

#ifndef PRINTMAPEVALUATION_H
#define PRINTMAPEVALUATION_H

#include "MapEvaluatorMethod.h"

template<class MPTraits>
class PrintMapEvaluation : public MapEvaluatorMethod<MPTraits> {
  public:

    PrintMapEvaluation();
    PrintMapEvaluation(string _baseName);
    PrintMapEvaluation(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node); 
    virtual ~PrintMapEvaluation();
  
    virtual void PrintOptions(ostream& _os);
  
    virtual bool operator()(); 

  protected:
    string m_baseName;
};

template<class MPTraits>
PrintMapEvaluation<MPTraits>::PrintMapEvaluation() {
  this->SetName("PrintMapEvaluation");
}

template<class MPTraits>
PrintMapEvaluation<MPTraits>::PrintMapEvaluation(string _baseName)
  : m_baseName(_baseName) {
  this->SetName("PrintMapEvaluation");
}

template<class MPTraits>
PrintMapEvaluation<MPTraits>::PrintMapEvaluation(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node)
  : MapEvaluatorMethod<MPTraits>(_problem, _node) {
  this->SetName("PrintMapEvaluation");
  m_baseName = _node.stringXMLParameter("base_name", true, "", "base filename for map output");
}

template<class MPTraits>
PrintMapEvaluation<MPTraits>::~PrintMapEvaluation() {
}

template<class MPTraits>
void
PrintMapEvaluation<MPTraits>::PrintOptions(ostream& _os) {
  _os << "PrintMapEvalaution" << endl;
  _os << "\tbase filename = " << m_baseName << endl;
}

template<class MPTraits>
bool
PrintMapEvaluation<MPTraits>::operator()() {
  int numNodes = this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_vertices();
  int numEdges = this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_edges();
  ostringstream osName;
  osName << m_baseName << "." << numNodes << "." << numEdges << ".map";
  ofstream osMap(osName.str().c_str());
  this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
  osMap.close();  
  
  int numCollNodes = this->GetMPProblem()->GetBlockRoadmap()->GetGraph()->get_num_vertices();
  int numCollEdges = this->GetMPProblem()->GetBlockRoadmap()->GetGraph()->get_num_edges();
  if(numCollNodes) {
    ostringstream osCollName;
    osCollName << m_baseName << "." << numCollNodes << "." << numCollEdges << ".block.map";
    ofstream osCollMap(osCollName.str().c_str());
    this->GetMPProblem()->GetBlockRoadmap()->Write(osCollMap, this->GetMPProblem()->GetEnvironment());
    osCollMap.close();
  }

  return true;
}
#endif

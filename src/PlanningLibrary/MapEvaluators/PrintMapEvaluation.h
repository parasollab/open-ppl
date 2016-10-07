#ifndef PRINTMAPEVALUATION_H
#define PRINTMAPEVALUATION_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class PrintMapEvaluation : public MapEvaluatorMethod<MPTraits> {
  public:

    PrintMapEvaluation();
    PrintMapEvaluation(string _baseName);
    PrintMapEvaluation(typename MPTraits::MPProblemType* _problem, XMLNode& _node);
    virtual ~PrintMapEvaluation();

    virtual void Print(ostream& _os) const;

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
PrintMapEvaluation<MPTraits>::PrintMapEvaluation(typename MPTraits::MPProblemType* _problem, XMLNode& _node)
  : MapEvaluatorMethod<MPTraits>(_problem, _node) {
  this->SetName("PrintMapEvaluation");
  m_baseName = _node.Read("base_name", true, "", "base filename for map output");
}

template<class MPTraits>
PrintMapEvaluation<MPTraits>::~PrintMapEvaluation() {
}

template<class MPTraits>
void
PrintMapEvaluation<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
  _os << "\tbase filename = " << m_baseName << endl;
}

template<class MPTraits>
bool
PrintMapEvaluation<MPTraits>::operator()() {
  int numNodes = this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_vertices();
  int numEdges = this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_edges();
  ostringstream osName;
  osName << m_baseName << "." << numNodes << "." << numEdges << ".map";
  this->GetMPProblem()->GetRoadmap()->Write(osName.str(), this->GetMPProblem()->GetEnvironment());

  int numCollNodes = this->GetMPProblem()->GetBlockRoadmap()->GetGraph()->get_num_vertices();
  int numCollEdges = this->GetMPProblem()->GetBlockRoadmap()->GetGraph()->get_num_edges();
  if(numCollNodes) {
    ostringstream osCollName;
    osCollName << m_baseName << "." << numCollNodes << "." << numCollEdges << ".block.map";
    this->GetMPProblem()->GetBlockRoadmap()->Write(osCollName.str(), this->GetMPProblem()->GetEnvironment());
  }

  return true;
}
#endif

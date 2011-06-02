#include "TestEvaluation.h"
#include "MPRegion.h"

TestEvaluation::
TestEvaluation (int s) : size(s) {this->SetName("TestEvaluator");} 


TestEvaluation::
TestEvaluation(XMLNodeReader& in_Node, MPProblem* in_pProblem, int s) 
 : MapEvaluationMethod(in_Node, in_pProblem), size(500) 
{
  this->SetName("TestEvaluator");
  size = in_Node.numberXMLParameter("size", false, 500, 0, 1000000, "number of nodes required in roadmap");
}


TestEvaluation::
~TestEvaluation() 
{}

void
TestEvaluation::
PrintOptions(ostream& out_os)
{
  out_os << this->GetName() << "::  size = " << size << endl;
}


bool
TestEvaluation::
operator() (int in_RegionID) 
{
  PrintOptions(cout);
  return ((int)GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap()->m_pRoadmap->get_num_vertices() >= size);
}


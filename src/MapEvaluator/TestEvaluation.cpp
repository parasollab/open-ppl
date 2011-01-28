#include "TestEvaluation.h"
#include "MPRegion.h"

TestEvaluation::
TestEvaluation (int s) : size(s) {} 


TestEvaluation::
TestEvaluation(XMLNodeReader& in_Node, MPProblem* in_pProblem, int s) 
 : MapEvaluationMethod(in_Node, in_pProblem), size(500) 
{
  size = in_Node.numberXMLParameter("size", false, 500, 0, 1000000, "number of nodes required in roadmap");
}


TestEvaluation::
~TestEvaluation() 
{}


char*
TestEvaluation::
GetName() const 
{ 
  return "test"; 
}


void
TestEvaluation::
PrintOptions(ostream& out_os)
{
  out_os << GetName() << "::  size = " << size << endl;
}


bool
TestEvaluation::
operator() (int in_RegionID) 
{
  PrintOptions(cout);
  return (GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap()->m_pRoadmap->get_num_vertices() >= size);
}


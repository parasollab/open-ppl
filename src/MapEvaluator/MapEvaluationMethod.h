#ifndef _MAP_EVALUATION_METHOD_H
#define _MAP_EVALUATION_METHOD_H

#include "Utilities/util.h"

class MapEvaluationMethod
  : public MPBaseObject 
{
 public:
  MapEvaluationMethod(XMLNodeReader& in_pNode, MPProblem* in_pProblem) 
    :  MPBaseObject(in_pNode, in_pProblem) 
    {}
  virtual ~MapEvaluationMethod() {}
  
  virtual char* GetName() const = 0;
  virtual void PrintOptions(ostream& out_os) = 0;
  
  virtual bool operator() () = 0;
  virtual bool operator() (int in_RegionID) = 0;
};


////////////////////////////////////////
//TEST
//for testing the framework only...
class TestEvaluation 
  : public MapEvaluationMethod 
{
 public:
  int size;
  
  TestEvaluation(XMLNodeReader& in_Node, MPProblem* in_pProblem, int s = 500) 
    : MapEvaluationMethod(in_Node, in_pProblem), size(500) 
    {
      size = in_Node.numberXMLParameter("size", false, 500, 0, 1000000, "number of nodes required in roadmap");
    }
  virtual ~TestEvaluation() {}
  
  virtual char* GetName() const { return "test"; }
  virtual void PrintOptions(ostream& out_os)
  {
    out_os << GetName() << "::  size = " << size << endl;
  }
  
  virtual bool operator() () 
  {
    return operator()(GetMPProblem()->CreateMPRegion());
  }
  virtual bool operator() (int in_RegionID) 
  {
    PrintOptions(cout);
    return (GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap()->m_pRoadmap->get_num_vertices() >= size);
  }
};


#include "QueryEvaluation.h"


/*
#include "CoverageEvaluation.h"
#include "ConnectivityEvaluation.h"
#include "MaxFlowEvaluation.h"
#include "CCDistanceEvaluation.h"
#include "CCDiameterEvaluation.h"
#include "CCExpansionEvaluation.h"
*/

#endif

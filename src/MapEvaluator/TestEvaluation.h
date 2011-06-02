#ifndef _TEST_EVALUATION_H
#define _TEST_EVALUATION_H

#include "MapEvaluationMethod.h"
#include "MPProblem.h"

class TestEvaluation 
  : public MapEvaluationMethod 
{
 public:

  TestEvaluation(int s);
  TestEvaluation(XMLNodeReader& in_Node, MPProblem* in_pProblem, int s = 500); 
  virtual ~TestEvaluation(); 
  
  virtual void PrintOptions(ostream& out_os);
  
  virtual bool operator() () 
  {
    return operator()(GetMPProblem()->CreateMPRegion());
  }
  virtual bool operator() (int in_RegionID); 

 protected:
  int size;
};

#endif

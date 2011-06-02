#ifndef _PRINT_MAP_EVALUATION_H
#define _PRINT_MAP_EVALUATION_H

#include "MapEvaluationMethod.h"
#include "MPProblem.h"


class PrintMapEvaluation 
  : public MapEvaluationMethod 
{
 public:
  PrintMapEvaluation(string _base_name) : base_name(_base_name){};
  PrintMapEvaluation(XMLNodeReader& in_Node, MPProblem* in_pProblem); 
  virtual ~PrintMapEvaluation();
  
  virtual void PrintOptions(ostream& out_os);
  
  virtual bool operator() () 
  {
    return operator()(GetMPProblem()->CreateMPRegion());
  }
  virtual bool operator() (int in_RegionID); 

 protected:
  string base_name;
};

#endif

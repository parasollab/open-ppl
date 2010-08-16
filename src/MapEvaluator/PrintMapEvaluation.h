#ifndef _PRINT_MAP_EVALUATION_H
#define _PRINT_MAP_EVALUATION_H

#include "MapEvaluationMethod.h"
#include "MPProblem.h"


class PrintMapEvaluation 
  : public MapEvaluationMethod 
{
 public:
  PrintMapEvaluation(XMLNodeReader& in_Node, MPProblem* in_pProblem); 
  virtual ~PrintMapEvaluation();
  
  virtual char* GetName() const { return "PrintMap"; }
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

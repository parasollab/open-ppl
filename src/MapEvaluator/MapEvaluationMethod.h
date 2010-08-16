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

/*
#include "CoverageEvaluation.h"
#include "ConnectivityEvaluation.h"
#include "MaxFlowEvaluation.h"
#include "CCDistanceEvaluation.h"
#include "CCDiameterEvaluation.h"
#include "CCExpansionEvaluation.h"
*/

#endif

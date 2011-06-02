#ifndef _MAP_EVALUATION_METHOD_H
#define _MAP_EVALUATION_METHOD_H

#include "Utilities/util.h"

class MapEvaluationMethod
  : public MPBaseObject 
{
 public:
  MapEvaluationMethod() {}
  MapEvaluationMethod(XMLNodeReader& in_pNode, MPProblem* in_pProblem) 
    :  MPBaseObject(in_pNode, in_pProblem) 
    {}
  virtual ~MapEvaluationMethod() {}
  
  virtual void PrintOptions(ostream& out_os) = 0;
  
  virtual bool operator() () = 0;
  virtual bool operator() (int in_RegionID) = 0;
};

class TrueEvaluation : public MapEvaluationMethod
{
 public:
   TrueEvaluation() : MapEvaluationMethod() {this->SetName("TrueEvaluator");}
   TrueEvaluation(XMLNodeReader& in_pNode, MPProblem* in_pProblem)
      : MapEvaluationMethod(in_pNode, in_pProblem){this->SetName("TrueEvaluator");}
   virtual ~TrueEvaluation(){}

   virtual void PrintOptions(ostream& out_os){
      out_os<<"True Evaluator always returns true, no options present."<<endl;
   }

   virtual bool operator()(){return true;}
   virtual bool operator()(int in_RegionID){return true;}
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

#ifndef PRINTMAPEVALUATION_H
#define PRINTMAPEVALUATION_H

#include "MapEvaluationMethod.h"

class PrintMapEvaluation : public MapEvaluationMethod {
  public:

    PrintMapEvaluation();
    PrintMapEvaluation(string _baseName);
    PrintMapEvaluation(XMLNodeReader& _node, MPProblem* _problem); 
    virtual ~PrintMapEvaluation();
  
    virtual void PrintOptions(ostream& _os);
  
    virtual bool operator()(); 

  protected:
    string m_baseName;
};

#endif

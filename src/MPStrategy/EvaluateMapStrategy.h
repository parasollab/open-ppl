#ifndef EvaluateMapStrategy_h
#define EvaluateMapStrategy_h

#include "MPStrategy/MPStrategyMethod.h"

//Container for more readabble MPStrategyMethod constructor
class EMSContainer : public MPSMContainer 
{
 public:
  EMSContainer(MPSMContainer cont = MPSMContainer())
   : MPSMContainer(cont), parent(cont)
  {} 

  string map_filename;
  vector<string> evaluator_labels;
  MPSMContainer parent;
};


class EvaluateMapStrategy : public MPStrategyMethod 
{
 public:
  EvaluateMapStrategy(EMSContainer cont)
   : MPStrategyMethod(cont.parent),
     map_filename(cont.map_filename),
     evaluator_labels(cont.evaluator_labels)
  {}
  EvaluateMapStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml = true);
  virtual ~EvaluateMapStrategy() {}

  virtual void PrintOptions(ostream& out_os);
 
  virtual void ParseXML(XMLNodeReader& in_Node) 
  {
    ParseXML(in_Node, true); 
  }
  virtual void ParseXML(XMLNodeReader& in_Node, bool warn_unknown); 
   
  virtual void Initialize(int region_id) {}
  virtual void Run(int region_id);
  virtual void Finalize(int region_id) {}

protected:
  string map_filename;
  vector<string> evaluator_labels;
};

#endif


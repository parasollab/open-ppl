#include "EvaluateMapStrategy.h"
#include "boost/lambda/lambda.hpp"
#include "MPRegion.h"
#include "MapEvaluator.h"
#include "MPStrategy.h"

EvaluateMapStrategy::
EvaluateMapStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml)
 : MPStrategyMethod(in_Node, in_pProblem) 
{
  if(parse_xml)
    ParseXML(in_Node); 
}


void 
EvaluateMapStrategy::
PrintOptions(ostream& out_os) 
{
  using boost::lambda::_1;
  out_os << "EvaluateMapStrategy::";
  out_os << "\n\tmap file = \"" << map_filename << "\"";
  out_os << "\tevaluation_methods: ";
  for_each(evaluator_labels.begin(), evaluator_labels.end(), cout << _1 << " ");
}


void 
EvaluateMapStrategy::
ParseXML(XMLNodeReader& in_Node, bool warn_unknown) 
{

  string default_map_filename = getBaseFilename() + ".map";
  map_filename = in_Node.stringXMLParameter("map_filename", false, default_map_filename, "Map Filename");
  
  for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) 
    if(citr->getName() == "evaluation_method") 
    {
      string method = citr->stringXMLParameter("Method", true, "", "Map Evaluation Method");
      evaluator_labels.push_back(method);
      citr->warnUnrequestedAttributes();
    } 
    else 
    {
      if(warn_unknown)
        citr->warnUnknownNode();
    }

  in_Node.warnUnrequestedAttributes();
  
}
  

void 
EvaluateMapStrategy::
Run(int region_id) 
{
  PrintOptions(cout);
  
  OBPRM_srand(getSeed()); 

  GetMPProblem()->GetMPRegion(region_id)->GetRoadmap()->ReadRoadmapGRAPHONLY(map_filename.c_str());
  
  for(vector<string>::const_iterator I = evaluator_labels.begin(); I != evaluator_labels.end(); ++I)
  {
    shared_ptr<MapEvaluationMethod> evaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
    cout << "\n\t";
    bool passed = evaluator->operator()(region_id);
    if(passed)
      cout << "\t  (passed)\n";
    else
    {
      cout << "\t  (failed)\n";
      return;
    }
  }
}


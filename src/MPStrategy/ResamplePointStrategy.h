#ifndef ResamplePointStrategy_h
#define ResamplePointStrategy_h

#include "MPStrategyMethod.h"
#include "LocalPlanners.h"
class ResamplePointStrategy : public MPStrategyMethod 
{
 public:   
  ResamplePointStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem); 
  virtual ~ResamplePointStrategy();

  virtual void PrintOptions(ostream& out_os);
  virtual void ParseXML(XMLNodeReader& in_Node);

  virtual void Initialize(int in_RegionID){}
  virtual void Run(int in_RegionID);
  virtual void Finalize(int in_RegionID){}
  

 protected:
  string m_input_path_filename;
  string m_output_path_filename;
  string m_input_map_filename;
  string m_output_map_filename;
  string type_name;
  string m_vc;
  string m_dm;
  string m_lp;
  int m_num_resamples;
  double step_size;
  double user_value;
};

#endif


#ifndef IncrementalPRMStrategy_h
#define IncrementalPRMStrategy_h


#include "MPStrategy/MPStrategyMethod.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/RoadmapGraph.h" //for VID typedef


class IncrementalPRMStrategy : public MPStrategyMethod 
{
 public:

  IncrementalPRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~IncrementalPRMStrategy();

  virtual void ParseXML(XMLNodeReader& in_Node);
  virtual void PrintOptions(ostream& out_os);

  virtual void operator()() 
  {
    (*this)(GetMPProblem()->CreateMPRegion());
  }
  virtual void operator()(int in_RegionID);

 private:
  //helper functions for operator()
  template <typename OutputIterator>
    void generate_nodes(MPRegion<CfgType, WeightType>* region, OutputIterator all_out, OutputIterator this_iteration_out);
  void connect_nodes(MPRegion<CfgType, WeightType>* region, vector<VID>& all_nodes_VID, vector<VID>& this_iteration_nodes_VID);
  void connect_components(MPRegion<CfgType, WeightType>* region);
  bool evaluate_map(int in_RegionID);

  //data
  vector<string> m_vecStrNodeGenerationLabels;
  vector<string> m_vecStrNodeConnectionLabels;
  vector<string> m_vecStrComponentConnectionLabels;
  vector<string> m_vecStrEvaluatorLabels;
  
  string m_strBaseFilename;

  int m_current_iteration;
};


#include "MPProblem/MPRegion.h"
#include "Utilities/Clock_Class.h"
//#include "NodeGenerator/NodeGeneratorMethod.h"
#include "NodeGenerator/Sampler.h"
#include "MPStrategy/MPStrategy.h"

//implentations of template functions
template <typename OutputIterator>
void
IncrementalPRMStrategy::
generate_nodes(MPRegion<CfgType, WeightType>* region, OutputIterator all_out, OutputIterator this_iteration_out)
{
  Clock_Class NodeGenClock;
  CDInfo cdInfo;
  Stat_Class * pStatClass = region->GetStatClass();
  stringstream clock_name; clock_name << "Iteration " << m_current_iteration << ", Node Generation"; 
  NodeGenClock.StartClock(clock_name.str().c_str());
  
  for(vector<string>::iterator I = m_vecStrNodeGenerationLabels.begin(); I != m_vecStrNodeGenerationLabels.end(); ++I) //SLT: should be const_iterator
  {
   // NodeGenerationMethod<CfgType>* pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetGenerateMapNodes()->GetMethod(*I);
    //sjacobs
    Sampler<CfgType>::SamplerPointer pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(*I);
    vector<CfgType> out_nodes;
    vector<CfgType> in_nodes(100);
        
    //generate nodes for this node generator method
    Clock_Class NodeGenSubClock;
    //check this
   // stringstream generator_clock_name; generator_clock_name << "Iteration " << m_current_iteration << ", " << pNodeGenerator->GetName();
   // NodeGenSubClock.StartClock(generator_clock_name.str().c_str());
    
    cout << "\n\t";
   // pNodeGenerator->GenerateNodes(region, nodes);
     pNodeGenerator->GetSampler()->Sample(pNodeGenerator,GetMPProblem()->GetEnvironment(),*pStatClass,in_nodes.begin(),in_nodes.end(),100, back_inserter(out_nodes));
   
    cout << "\n\t";
    NodeGenSubClock.StopPrintClock();
        
    //add valid nodes to roadmap
    for(vector<CfgType>::iterator C = in_nodes.begin(); C != in_nodes.end(); ++C)
    {
      //if((*C).IsLabel("VALID") && ((*C).GetLabel("VALID"))) {
       //sjacobs- start
       if(!(*C).IsLabel("VALID")){
	C->isCollision(GetMPProblem()->GetEnvironment(),*(region->GetStatClass()),GetMPProblem()->GetCollisionDetection(), cdInfo);
     }
      if((*C).IsLabel("VALID") && ((*C).GetLabel("VALID"))) {
	      //sjacobs-end
        VID vid = region->GetRoadmap()->m_pRoadmap->AddVertex(*C);
        //store value and increment iterator
        *this_iteration_out++ = vid;
        *all_out++ = vid;
      }
    }
  }
  NodeGenClock.StopPrintClock();
}


#endif

#ifndef MPStrategyMethod_h
#define MPStrategyMethod_h

#include <sys/time.h>
#include "util.h"
#include "CfgTypes.h"
#include "MPProblem/RoadmapGraph.h" //for VID typedef
#include "MPProblem.h"

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType, WeightType> GRAPH;
typedef GRAPH::vertex_descriptor VID; 

///Will be used to derive IMP,PRM,RRT,metaplanner, etc.
class MPStrategyMethod : public MPBaseObject 
{
  public:
    MPStrategyMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
      MPBaseObject(in_Node,in_pProblem) { 
            
      struct timeval tv;
      gettimeofday(&tv,NULL);
      m_baseSeed = in_Node.numberXMLParameter(string("seed"), false,
                                              int(tv.tv_usec),int(0),int(MAX_INT), 
                                              string("Random Seed Value")); 
  
      m_iterations = in_Node.numberXMLParameter(string("iterations"), true,
                                              int(1),int(0),int(MAX_INT), 
                                              string("Number of Iterations")); 
      num_nodes = in_Node.numberXMLParameter(string("num_samples"), true,
                                              int(100),int(10),int(MAX_INT), 
                                              string("Number of Samples")); 
      
      m_base_filename = in_Node.stringXMLParameter(string("filename"), true,
                                            string(""), 
                                            string("Base output filename"));
      
      
      LOG_DEBUG_MSG("MPStrategyMethod::Seed is " << m_baseSeed);
      LOG_DEBUG_MSG("MPStrategyMethod::No of sample is " << num_nodes);
    };
  virtual ~MPStrategyMethod() {}
  virtual void ParseXML(XMLNodeReader& in_Node)=0;
  void operator()(){
     LOG_DEBUG_MSG("MPStrategyMethod::operator()() ");
     (*this)(GetMPProblem()->CreateMPRegion());
   }
  void operator()(int in_RegionID){
   LOG_DEBUG_MSG("MPStrategyMethod::operator() " << in_RegionID);
   Initialize(in_RegionID);
   Run(in_RegionID);
   Finalize(in_RegionID);
  }
  virtual void Initialize(int in_RegionID)=0;
  virtual void Run(int in_RegionID)=0;
  virtual void Finalize(int in_RegionID)=0;
  virtual void PrintOptions(ostream& out_os)=0;
  long getSeed(){return m_baseSeed;};
  string getBaseFilename(){return m_base_filename;};
  void setSeed(long in_seed){m_baseSeed = in_seed;};
 private:
  
  long m_baseSeed;
  string m_base_filename;

 protected:
  int m_iterations;
  int num_nodes;
  //bool m_reset_stats;
  //bool m_no_output_files;
};

#endif 

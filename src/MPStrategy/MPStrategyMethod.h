#ifndef MPStrategyMethod_h
#define MPStrategyMethod_h

#include <sys/time.h>
#include "util.h"

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
      
      m_base_filename = in_Node.stringXMLParameter(string("filename"), true,
                                            string(""), 
                                            string("Base output filename"));
      
      
      LOG_DEBUG_MSG("MPStrategyMethod::Seed is " << m_baseSeed);
    };
  virtual ~MPStrategyMethod() {}
  virtual void ParseXML(XMLNodeReader& in_Node)=0;
  virtual void operator()()=0;
  virtual void operator()(int in_RegionID)=0;
  virtual void PrintOptions(ostream& out_os)=0;
  long getSeed(){return m_baseSeed;};
  string getBaseFilename(){return m_base_filename;};
  void setSeed(long in_seed){m_baseSeed = in_seed;};
 private:
  
  long m_baseSeed;
  string m_base_filename;

 protected:
  int m_iterations;
  //bool m_reset_stats;
  //bool m_no_output_files;
};

#endif 

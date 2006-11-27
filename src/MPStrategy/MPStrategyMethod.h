#ifndef MPStrategyMethod_h
#define MPStrategyMethod_h

#include "util.h"

///Will be used to derive IMP,PRM,RRT,metaplanner, etc.
class MPStrategyMethod : public MPBaseObject 
{
  public:
    MPStrategyMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
      MPBaseObject(in_pNode,in_pProblem) { 
      m_baseSeed=0;
      int seed;
      if(TIXML_SUCCESS  == in_pNode->ToElement()->QueryIntAttribute("seed",&seed)) {
        m_baseSeed = (long) seed;
      } else {
        LOG_DEBUG_MSG("MPStrategyMethod::No Seed Found");
        struct timeval tv;
        gettimeofday(&tv,NULL);
        m_baseSeed = ((unsigned int) tv.tv_usec);
        cout << "RANDOM SEED = " << m_baseSeed << endl;
      }
  
      int iterations;
      if(TIXML_SUCCESS  == in_pNode->ToElement()->QueryIntAttribute("iterations",&iterations)) {
        m_iterations = iterations;
      } else {
        LOG_DEBUG_MSG("MPStrategyMethod::Iterations Found");
      }
      const char* filename;
      filename= in_pNode->ToElement()->Attribute("filename");
      if(filename) {
        m_base_filename = string(filename);
      } else {
        LOG_DEBUG_MSG("MPStrategyMethod::No filename Found");
      }
      const char* no_output_files;
      no_output_files= in_pNode->ToElement()->Attribute("no_output_files");
      if(no_output_files) {
        m_no_output_files = true;
      } else {
        m_no_output_files = false;
      }
      LOG_DEBUG_MSG("MPStrategyMethod::Seed is " << m_baseSeed);

      m_reset_stats = false;
      int reset_stats;
      if(TIXML_SUCCESS  == in_pNode->ToElement()->QueryIntAttribute("reset_stats",&reset_stats)) {
        if (reset_stats)
          m_reset_stats = true;
      }

    };
  virtual void ParseXML(TiXmlNode* in_pNode)=0;
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
  bool m_reset_stats;
  bool m_no_output_files;
};

#endif 

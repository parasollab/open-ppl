#ifndef MPStrategyMethod_h
#define MPStrategyMethod_h

#include <sys/time.h>
#include "Clock_Class.h"
#include "CfgTypes.h"
#include "MPProblem/RoadmapGraph.h" //for VID typedef
#include "MPProblem.h"

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType, WeightType> GRAPH;
typedef GRAPH::vertex_descriptor VID; 

class MPSMContainer {
public:
  MPSMContainer () {} //Container for more readabble MPStrategyMethod constructor

  long m_baseSeed;
  string m_base_filename;
  int m_iterations;
  int num_nodes;
};


///Will be used to derive IMP,PRM,RRT,metaplanner, etc.
class MPStrategyMethod : public MPBaseObject {
  public:
    MPStrategyMethod() {};
    MPStrategyMethod(MPSMContainer cont) {
      m_baseSeed = cont.m_baseSeed;
      m_base_filename = cont.m_base_filename;
      m_iterations = cont.m_iterations;
      num_nodes = cont.num_nodes;
    }


    MPStrategyMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) : MPBaseObject(in_Node,in_pProblem) { 
      struct timeval tv;
      gettimeofday(&tv,NULL);
      m_baseSeed = in_Node.numberXMLParameter("seed", false, (int)tv.tv_usec, 0, MAX_INT, "Random Seed Value"); 
      m_iterations = in_Node.numberXMLParameter("iterations", true, 1, 0, MAX_INT, "Number of Iterations"); 
      num_nodes = in_Node.numberXMLParameter("num_samples", true, 100, 10, MAX_INT, "Number of Samples"); 
      m_base_filename = in_Node.stringXMLParameter("filename", true, "", "Base output filename");
    };

    virtual ~MPStrategyMethod() {}
    virtual void ParseXML(XMLNodeReader& in_Node)=0;
    void operator()(){
      (*this)(GetMPProblem()->CreateMPRegion());
    }

    void operator()(int in_RegionID){
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
    Clock_Class m_strategyClock;
    //bool m_reset_stats;
    //bool m_no_output_files;
};

#endif 

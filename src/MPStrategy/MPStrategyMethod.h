#ifndef MPSTRATEGYMETHOD_H
#define MPSTRATEGYMETHOD_H

#include <sys/time.h>
#include "MetricUtils.h"
#include "CfgTypes.h"
#include "MPUtils.h"
#include "MPProblem.h"
#include "MPProblem/RoadmapGraph.h"

struct MPSMContainer {
  long m_seed;
  string m_baseFilename;
};

class MPStrategyMethod : public MPBaseObject {
  public:
    MPStrategyMethod(MPSMContainer& _cont) : m_baseSeed(_cont.m_seed), m_baseFilename(_cont.m_baseFilename) {}
    MPStrategyMethod(XMLNodeReader& _node, MPProblem* _problem) : MPBaseObject(_node, _problem) {
      if(m_boundary==NULL)
        m_boundary = GetMPProblem()->GetEnvironment()->GetBoundingBox();
      ParseXML(_node);
    };
    virtual ~MPStrategyMethod() {}
    
    virtual void ParseXML(XMLNodeReader& _node){
      struct timeval tv;
      gettimeofday(&tv,NULL);
      m_baseSeed = _node.numberXMLParameter("seed", false, (int)tv.tv_usec, 0, MAX_INT, "Random Seed Value"); 
      m_baseFilename = _node.stringXMLParameter("filename", true, "", "Base output filename");
      SRand(m_baseSeed); 
    };
    
    void operator()(){
      (*this)(GetMPProblem()->CreateMPRegion());
    }
    void operator()(int _regionID){
      Initialize(_regionID);
      Run(_regionID);
      Finalize(_regionID);
    }

    virtual void Initialize(int _regionID)=0;
    virtual void Run(int _regionID)=0;
    virtual void Finalize(int _regionID)=0;
    virtual void PrintOptions(ostream& _os)=0;

    string GetBaseFilename(){return m_baseFilename;}
    void setBoundary(shared_ptr<BoundingBox> bb){m_boundary=bb;};
    long GetBaseSeed() {return m_baseSeed;} 
  protected:
    ClockClass m_strategyClock;
    typedef RoadmapGraph<CfgType, WeightType>::GRAPH GRAPH;
    typedef RoadmapGraph<CfgType, WeightType>::VID VID; 
    shared_ptr<BoundingBox> m_boundary; 

  private:
    long m_baseSeed;
    string m_baseFilename;
};

#endif 

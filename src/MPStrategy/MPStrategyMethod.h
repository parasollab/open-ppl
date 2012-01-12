#ifndef MPSTRATEGYMETHOD_H
#define MPSTRATEGYMETHOD_H

#include "MetricUtils.h"
#include "CfgTypes.h"
#include "MPUtils.h"
#include "MPProblem/RoadmapGraph.h"

struct MPSMContainer {
  long m_seed;
  string m_baseFilename;
};

class MPProblem;

class MPStrategyMethod : public MPBaseObject {
  public:
    MPStrategyMethod(MPSMContainer& _cont);
    MPStrategyMethod(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~MPStrategyMethod();

    virtual void ParseXML(XMLNodeReader& _node);

    void operator()();
    void operator()(int _regionID);

    virtual void Initialize(int _regionID)=0;
    virtual void Run(int _regionID)=0;
    virtual void Finalize(int _regionID)=0;
    virtual void PrintOptions(ostream& _os)=0;

    string GetBaseFilename(){return m_baseFilename;}
    void SetBoundary(shared_ptr<BoundingBox> bb){m_boundary=bb;};
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

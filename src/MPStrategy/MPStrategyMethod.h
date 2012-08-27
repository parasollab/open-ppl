#ifndef MPSTRATEGYMETHOD_H_
#define MPSTRATEGYMETHOD_H_

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

    virtual void Initialize()=0;
    virtual void Run()=0;
    virtual void Finalize()=0;
    virtual void PrintOptions(ostream& _os)=0;

    string GetBaseFilename(){return m_baseFilename;}
    void SetBoundary(shared_ptr<Boundary> bb){m_boundary=bb;};
    long GetBaseSeed() {return m_baseSeed;} 

    bool EvaluateMap(vector<string> _evaluators);
  
  protected:
    typedef RoadmapGraph<CfgType, WeightType>::GRAPH GRAPH;
    typedef RoadmapGraph<CfgType, WeightType>::VID VID; 
    shared_ptr<Boundary> m_boundary; 

  private:
    long m_baseSeed;
    string m_baseFilename;
};

#endif 

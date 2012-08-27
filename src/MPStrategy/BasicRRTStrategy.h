/////////////////////////
//Class BasicRRTStrategy
////////////////////////

#ifndef BASICRRTSTRATEGY_H_
#define BASICRRTSTRATEGY_H_

#include "IOUtils.h"
#include "MetricUtils.h"
#include "CfgTypes.h"
#include "MPStrategyMethod.h"

class BasicRRTStrategy : public MPStrategyMethod {
  public:
    
    ////////////////
    //Constructors
    ///////////////
    BasicRRTStrategy(XMLNodeReader& _node, MPProblem* _problem, bool _warnXML = true);
    virtual ~BasicRRTStrategy() {}
    
    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void PrintOptions(ostream& _os);

  protected:
    // Helper functions
    CfgType GoalBiasedDirection();
    CfgType SelectDirection();
    virtual VID ExpandTree(CfgType& _dir);
    void ConnectTrees(VID _recentlyGrown);
    void EvaluateGoals();
    RoadmapClearanceStats PathClearance();  
    vector<string> m_evaluators;
    string m_sampler;
    string m_lp;
    string m_dm;
    string m_nf;
    string m_vc;
    string m_query;
    double m_delta, m_minDist, m_growthFocus;
    int m_numRoots, m_currentIteration;
    vector<CfgType> m_goals, m_roots;
    vector<size_t> m_goalsNotFound;
};

#endif

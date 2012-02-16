/**
 * BasicRRTStrategy.h
 * 
 * Description: RRT Strategy header file
 *
 * Author: Kasra Manavi
 * Last Edited: 04/04/2011
 */

#ifndef BasicRRTStrategy_h
#define BasicRRTStrategy_h

#include "IOUtils.h"
#include "CfgTypes.h"
#include "MPStrategyMethod.h"

template<typename CFG, typename WEIGHT> class MPRegion;

class BasicRRTStrategy : public MPStrategyMethod {
  public:
    BasicRRTStrategy(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~BasicRRTStrategy() {}
    
    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize(int _regionID);
    virtual void Run(int _regionID);
    virtual void Finalize(int _regionID);
    virtual void PrintOptions(ostream& _os);

  protected:
    // Helper functions
    CfgType GoalBiasedDirection(int _regionID);
    CfgType SelectDirection(int _regionID);
    VID ExpandTree(int _regionID, CfgType& _dir);
    void ConnectTrees(int _regionID, VID _recentlyGrown);
    void EvaluateGoals(int _regionID);
    bool EvaluateMap(int _regionID);

  private:
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

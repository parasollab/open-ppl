/**
 * MARRTStrategy.h
 * 
 * Description: MARRT Strategy header file
 *
 * Author: Evan Greco
 * Last Edited: 01/24/2012
 */

#ifndef MARRTSTRATEGY_H_
#define MARRTSTRATEGY_H_

#include "MetricUtils.h"
#include "IOUtils.h"
#include "CfgTypes.h"
#include "MPStrategyMethod.h"

class MARRTStrategy : public MPStrategyMethod {
  public:
    MARRTStrategy(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~MARRTStrategy() {}
    
    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void PrintOptions(ostream& _os);

  protected:
    // Helper functions
    void ConnectComponents();
    RoadmapClearanceStats PathClearance();
  private:
    bool m_queryFound;
    vector<string> m_componentConnectors;
    vector<string> m_evaluators;
    bool m_findQuery;
    string m_sampler;
    string m_lp;
    string m_nf;
    string m_query;
    double m_eps;
    int m_hLen;
    double m_delta, m_minDist, m_growthFocus;
    int m_roots, m_currentIteration;
    vector<CfgType> m_goals;
    vector<CfgType> m_root;
    ClearanceParams m_cParams;
};

#endif

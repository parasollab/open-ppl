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

template<typename CFG, typename WEIGHT> class MPRegion;

class MARRTStrategy : public MPStrategyMethod {
  public:
    MARRTStrategy(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~MARRTStrategy() {}
    
    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize(int _regionID);
    virtual void Run(int _regionID);
    virtual void Finalize(int _regionID);
    virtual void PrintOptions(ostream& _os);

  protected:
    // Helper functions
    void ConnectComponents(int _regionID);
    bool EvaluateMap(int _regionID);
    RoadmapClearanceStats PathClearance(int _regionID);
  private:
    bool m_queryFound;
    vector<string> m_componentConnectors;
    vector<string> m_evaluators;
    bool m_findQuery;
    string m_sampler;
    string m_lp;
    string m_dm;
    string m_nf;
    string m_vc;
    string m_query;
    bool m_exact;
    int m_rayCount;
    int m_penetration;
    bool m_useBbx;
    int m_hLen;
    bool m_positional;
    double m_delta, m_minDist, m_growthFocus;
    int m_roots, m_currentIteration;
    vector<CfgType> m_goals;
    vector<CfgType> m_root;
};

#endif

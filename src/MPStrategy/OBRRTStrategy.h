/**
 * OBRRTStrategy.h
 * 
 * Description: OBRRT Strategy header file
 *
 * Author: Evan Greco
 * Last Edited: 04/04/2011
 */

#ifndef OBRRTSTRATEGY_H_
#define OBRRTSTRATEGY_H_

#include "IOUtils.h"
#include "CfgTypes.h"
#include "BasicRRTStrategy.h"

template<typename CFG, typename WEIGHT> class MPRegion;

class OBRRTStrategy : public BasicRRTStrategy {
  public:
    OBRRTStrategy(XMLNodeReader& _node, MPProblem* _problem) : BasicRRTStrategy(_node, _problem){};
    virtual ~OBRRTStrategy() {}
    
    virtual void ParseXML(XMLNodeReader& _node);

  protected:
    // Helper functions
    VID ExpandTree(int _regionID, CfgType& _dir);
    CfgType g0(int _regionID, CfgType& _dir);
    CfgType g1(int _regionID, CfgType& _dir);
    CfgType g8(int _regionID, CfgType& _dir);
  private:
    vector<string> m_componentConnectors;
    vector<string> m_evaluators;
    int m_numRoots;  
    string m_sampler;
    string m_lp;
    string m_dm;
    string m_nf;
    string m_vc;
    string m_query;
    //MAPRM information
    bool m_exact;
    int m_rayCount;
    int m_penetration;
    bool m_useBbx;
    int m_hLen;
    bool m_positional;
 
    double m_delta, m_minDist, m_growthFocus;
    int m_roots, m_currentIteration;
    double m_g0, m_g1, m_g2, m_g3, m_g4, m_g5, m_g6, m_g7, m_g8; //Growth method probabilities; can be found in OBRRT paper/TR
};

#endif

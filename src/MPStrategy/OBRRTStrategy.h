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
    OBRRTStrategy(XMLNodeReader& _node, MPProblem* _problem) : 
        BasicRRTStrategy(_node, _problem){
	   ParseXML(_node);
	};
    virtual ~OBRRTStrategy() {}
    
    virtual void ParseXML(XMLNodeReader& _node);

  protected:
    // Helper functions
    VID ExpandTree(int _regionID, CfgType& _dir);
    CfgType g0(int _regionID, CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g1(int _regionID, CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g2(int _regionID, CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g3(int _regionID, CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g4(int _regionID, CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g5(int _regionID, CfgType& _near, CfgType& _dir, bool& _verifiedValid, bool _maintainSrcOri);
    //g6 is the same as g5 but with ori changes
    CfgType g7(int _regionID, CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g8(int _regionID, CfgType& _near, CfgType& _dir, bool& _verifiedValid);
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
    double m_g0N, m_g1N, m_g2N, m_g3N, m_g4N, m_g5N, m_g6N, m_g7N, m_g8N; 
};

#endif

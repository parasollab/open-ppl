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
    OBRRTStrategy(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~OBRRTStrategy() {}
    
    virtual void ParseXML(XMLNodeReader& _node);

  protected:
    // Helper functions
    VID ExpandTree(CfgType& _dir);
    CfgType g0(CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g1(CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g2(CfgType& _near, CfgType& _dir, bool& _verifiedValid, bool _maintainSrcOri=false);
    //g3 is the same as g2 but with same orientation
    CfgType g4(CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g5(CfgType& _near, CfgType& _dir, bool& _verifiedValid, bool _maintainSrcOri=false);
    //g6 is the same as g5 but with same orientation
    CfgType g7(CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g8(CfgType& _near, CfgType& _dir, bool& _verifiedValid);
  
  private:
    int m_hLen;
    ClearanceParams m_cParams;
 
    double m_g0, m_g1, m_g2, m_g3, m_g4, m_g5, m_g6, m_g7, m_g8; //Growth method probabilities; can be found in OBRRT paper/TR
    double m_g0N, m_g1N, m_g2N, m_g3N, m_g4N, m_g5N, m_g6N, m_g7N, m_g8N; 
};

#endif

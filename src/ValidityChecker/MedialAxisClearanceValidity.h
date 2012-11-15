#ifndef MEDIALAXISCLEARANCEVALIDITY_H_
#define MEDIALAXISCLEARANCEVALIDITY_H_

#include "ValidityCheckerMethod.h"

class MedialAxisClearanceValidity : public ValidityCheckerMethod {
  public:
    MedialAxisClearanceValidity(const ClearanceParams& cParams = ClearanceParams(),
      int _historyLen = 5 , double _epsilon = 0.1, double _clearance = 0.0);
    MedialAxisClearanceValidity(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~MedialAxisClearanceValidity() {}

    virtual bool IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
        CDInfo& _cdInfo, string *_callName);

    vector< pair<CfgType,CfgType> > GetHistory();
    void ClearHistory();
    void ParseXML(XMLNodeReader& _node);
  
  private:  
    ClearanceParams m_cParams;
    int m_historyLength;
    double m_epsilon;
    double m_clearance;
    vector< pair<CfgType,CfgType> > m_history;

};

#endif

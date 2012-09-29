#ifndef MEDIALAXISCLEARANCEVALIDITY_H_
#define MEDIALAXISCLEARANCEVALIDITY_H_

#include "ValidityCheckerMethod.hpp"

class MedialAxisClearanceValidity : public ValidityCheckerMethod {
  public:
    MedialAxisClearanceValidity();
    MedialAxisClearanceValidity(string _dmLabel, string _vcLabel, bool _useBBX, bool _cExact, bool _pExact, 
        int _cRay, int _pRay, int _historyLen, double _epsilon, double _clearance);
    MedialAxisClearanceValidity(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~MedialAxisClearanceValidity() {}

    virtual bool IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
        CDInfo& _cdInfo, string *_callName);

    vector< pair<CfgType,CfgType> > GetHistory();
    void ClearHistory();

  private:
    string m_dmLabel;
    string m_vcLabel;
    bool m_useBBX,m_positional;
    bool m_cExact, m_pExact;
    int m_cRay;
    int m_pRay;
    int m_historyLen;
    double m_epsilon;
    double m_clearance;
    vector< pair<CfgType,CfgType> > m_history;
};

#endif

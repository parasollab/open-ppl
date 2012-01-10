#ifndef MEDIALAXISCLEARANCEVALIDITY_H_
#define MEDIALAXISCLEARANCEVALIDITY_H_

#include "ValidityCheckerMethod.hpp"
#include <string>
#include <map>
using namespace std;

class MedialAxisClearanceValidity : public ValidityCheckerMethod {
  public:
    MedialAxisClearanceValidity() { }
    MedialAxisClearanceValidity(string _dmLabel, string _vcLabel, bool _useBBX, string _cType, string _pType, 
      int _cRay, int _pRay, int historyLen, double _epsilon, double _clearance);
    MedialAxisClearanceValidity(XMLNodeReader& _node, MPProblem* _problem);
    ~MedialAxisClearanceValidity() { }

    virtual bool 
      IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
          CDInfo& _cdInfo, bool _enablePenetration, string *_callName);

    virtual vector< pair<CfgType,CfgType> > GetHistory();
    virtual void ClearHistory();

    vector< pair<CfgType,CfgType> > m_history;

  private:
    string m_dmLabel;
    string m_vcLabel;
    bool m_useBBX;
    string m_cType;
    string m_pType;
    int m_cRay;
    int m_pRay;
    int m_historyLen;
    double m_epsilon;
    double m_clearance;
};

#endif

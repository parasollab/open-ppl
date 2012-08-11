#ifndef OBSTACLECLEARANCEVALIDITY_H_
#define OBSTACLECLEARANCEVALIDITY_H_

#include "ValidityCheckerMethod.hpp"

class ObstacleClearanceValidity : public ValidityCheckerMethod {
  public:
    ObstacleClearanceValidity(string _dmLabel = "", string _vcLabel = "", 
        double _clearance = 0.05, bool _useBBX = true, bool _cExact = true, 
        size_t _clearanceRays = 20, bool _positional = true);
    ObstacleClearanceValidity(XMLNodeReader& _node, MPProblem* _problem);

    ~ObstacleClearanceValidity() { }

    virtual bool IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
        CDInfo& _cdInfo, bool _enablePenetration, string* _callName);

  private:
    string m_dmLabel;
    string m_vcLabel;
    double m_clearance;
    bool m_useBBX;
    bool m_cExact;
    size_t m_clearanceRays;
    bool m_positional;
};

#endif

#ifndef NODECLEARANCEVALIDITY_H_
#define NODECLEARANCEVALIDITY_H_

#include "ValidityCheckerMethod.hpp"
#include <string>

using namespace std;

class NodeClearanceValidity : public ValidityCheckerMethod {
  public:
    NodeClearanceValidity() { }
    NodeClearanceValidity(double _delta, string _dmLabel, string _nfLabel);
    NodeClearanceValidity(XMLNodeReader& _node, MPProblem* _problem);
    ~NodeClearanceValidity() { }

    virtual bool 
      IsValid(Cfg& _cfg, Environment* _env, Stat_Class& _stats, 
          CDInfo& _cdInfo, bool _enablePenetration, string *_callName);

  private:
    string m_dmLabel;
    string m_nfLabel;
    double m_delta;
};

#endif

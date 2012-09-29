#ifndef NODECLEARANCEVALIDITY_H
#define NODECLEARANCEVALIDITY_H

#include "ValidityCheckerMethod.hpp" 

class NodeClearanceValidity : public ValidityCheckerMethod {
  public:
    NodeClearanceValidity(); 
    NodeClearanceValidity(double _delta, string _dmLabel, string _nfLabel);
    NodeClearanceValidity(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~NodeClearanceValidity();

    virtual bool IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
                         CDInfo& _cdInfo, string *_callName);

  private:
    double m_delta;
    string m_dmLabel;
    string m_nfLabel;
};

#endif

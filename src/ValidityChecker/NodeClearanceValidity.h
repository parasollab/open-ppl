// This class checks the validity of the sampled nodes.

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

    virtual bool //IsValid returns true if the provided _cfg is valid, false otherwise
      IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
          CDInfo& _cdInfo, bool _enablePenetration, string *_callName);

  private:
    double m_delta;
    string m_dmLabel;
    string m_nfLabel;
};

#endif

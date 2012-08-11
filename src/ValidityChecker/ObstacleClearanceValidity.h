#ifndef OBSTACLECLEARANCEVALIDITY_H_
#define OBSTACLECLEARANCEVALIDITY_H_

#include "CollisionDetectionValidity.hpp"
#include "ValidityCheckerMethod.hpp"
#include <string>
#include <map>
using namespace std;

class ObstacleClearanceValidity : public ValidityCheckerMethod {
  public:
    ObstacleClearanceValidity();
    ObstacleClearanceValidity(string _dmLabel, string _vcLabel, bool _useBBX, string _cType, double _clearance, bool _positional);
    ObstacleClearanceValidity(XMLNodeReader& _node, MPProblem* _problem);
    
    ~ObstacleClearanceValidity() { }

    virtual bool IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
          CDInfo& _cdInfo, bool _enablePenetration, string *_callName);
			

  private:
    string m_dmLabel;
    string m_vcLabel;
    bool m_useBBX;
    bool m_positional;
    string m_cType;
    double m_clearance;
};

#endif

/**
 * LocalPlannerMethod.h
 * This class is a base class for all local planner methods
 * 
 * Last Updated      : 1/18/12
 * Last Update Author: Aditya Mahadevan
 */
#ifndef LOCALPLANNERMETHOD_H_
#define LOCALPLANNERMETHOD_H_

#include "Roadmap.h"
#include "Environment.h"
#include "MultiBody.h"
#include "MetricUtils.h"
#include "MPUtils.h"
#include "ValidityChecker.hpp"

class DistanceMetricMethod;
template <class CFG, class WEIGHT> struct LPOutput;

template <class CFG, class WEIGHT> class LocalPlannerMethod : public MPBaseObject {
  public:

    LocalPlannerMethod() { }
    LocalPlannerMethod(XMLNodeReader& _node, MPProblem* _problem):MPBaseObject(_node, _problem) { }
    virtual ~LocalPlannerMethod();

    virtual void PrintOptions(ostream& out_os) { };
    virtual string GetVCMethod() { };
    virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy() = 0;
    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod> _dm,
        const CFG &_c1, const CFG &_c2,
        CFG &_col, LPOutput<CFG, WEIGHT>* _lpOutput,
        double _posRes, double _oriRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false) = 0;

    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod> _dm,
        const CFG &_c1, const CFG &_c2,
        LPOutput<CFG, WEIGHT>* _lpOutput,
        double _posRes, double _oriRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false) {
      CFG col;
      return IsConnected(_env,_stats,_dm,
          _c1,_c2,col,_lpOutput,
          _posRes,_oriRes,_checkCollision,
          _savePath,_saveFailedPath);
    }

};

template <class CFG, class WEIGHT> 
LocalPlannerMethod<CFG, WEIGHT>::~LocalPlannerMethod() { }

#endif

/* LocalPlannerMethod.h
 * This class is a base class for all local planner methods
 */

#ifndef LOCALPLANNERMETHOD_H_
#define LOCALPLANNERMETHOD_H_

#include "MPProblem/Environment.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

template<class MPTraits> struct LPOutput;

template<class MPTraits> 
class LocalPlannerMethod : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    LocalPlannerMethod(bool _saveIntermediates = false) : m_saveIntermediates(_saveIntermediates){}

    LocalPlannerMethod(MPProblemType* _problem, XMLNodeReader& _node) : MPBaseObject<MPTraits>(_problem, _node) {
      m_saveIntermediates = _node.boolXMLParameter("saveIntermediates", false, false, "Save intermediate nodes");
    }

    virtual ~LocalPlannerMethod(){}

    virtual void PrintOptions(ostream& _os){};

    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2,
        CfgType& _col, LPOutput<MPTraits>* _lpOutput,
        double _posRes, double _oriRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false) = 0;

    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2,
        LPOutput<MPTraits>* _lpOutput,
        double _posRes, double _oriRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false) {
      CfgType col;
      return IsConnected(_env,_stats,_dm,
          _c1,_c2,col,_lpOutput,
          _posRes,_oriRes,_checkCollision,
          _savePath,_saveFailedPath);
    }

    virtual vector<CfgType> ReconstructPath(Environment* _env, DistanceMetricPointer _dm, 
        const CfgType& _c1, const CfgType& _c2, const vector<CfgType>& _intermediates, 
        double _posRes, double _oriRes) {
      StatClass dummyStats;
      LPOutput<MPTraits>* lpOutput = new LPOutput<MPTraits>();
      IsConnected(_env, dummyStats, _dm, _c1, _c2, lpOutput, _posRes, _oriRes, false, true, false);
      vector<CfgType> path = lpOutput->path;
      delete lpOutput;
      return path;
    }

  protected:
    bool m_saveIntermediates; 
};

#endif

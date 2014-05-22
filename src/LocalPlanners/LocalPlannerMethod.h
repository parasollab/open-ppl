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

    LocalPlannerMethod(bool _saveIntermediates = false) :
        m_saveIntermediates(_saveIntermediates) {}

    LocalPlannerMethod(MPProblemType* _problem, XMLNodeReader& _node) :
        MPBaseObject<MPTraits>(_problem, _node) {
      m_saveIntermediates = _node.boolXMLParameter("saveIntermediates", false,
          false, "Save intermediate nodes");
    }

    virtual ~LocalPlannerMethod() {}

    virtual void Print(ostream& _os) const;

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false,
        bool _saveFailedPath = false) = 0;

    virtual bool IsConnected(const CfgType& _c1, const CfgType& _c2,
        LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false,
        bool _saveFailedPath = false);

    virtual vector<CfgType> ReconstructPath(
        const CfgType& _c1, const CfgType& _c2,
        const vector<CfgType>& _intermediates,
        double _posRes, double _oriRes);

  protected:
    bool m_saveIntermediates;
};

template<class MPTraits>
bool
LocalPlannerMethod<MPTraits>::IsConnected(const CfgType& _c1, const CfgType& _c2,
    LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  CfgType col;
  return IsConnected(_c1, _c2, col, _lpOutput, _posRes,
      _oriRes, _checkCollision, _savePath, _saveFailedPath);
}

template<class MPTraits>
vector<typename MPTraits::CfgType>
LocalPlannerMethod<MPTraits>::ReconstructPath(
    const CfgType& _c1, const CfgType& _c2,
    const vector<CfgType>& _intermediates,
    double _posRes, double _oriRes) {
  LPOutput<MPTraits>* lpOutput = new LPOutput<MPTraits>();
  IsConnected(_c1, _c2, lpOutput, _posRes, _oriRes,
      false, true, false);
  vector<CfgType> path = lpOutput->m_path;
  delete lpOutput;
  return path;
}

template<class MPTraits>
void
LocalPlannerMethod<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel()
      << "\n\tsave intermediates : " << m_saveIntermediates
      << endl;
}
#endif

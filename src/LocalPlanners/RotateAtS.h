/**
 * RotateAtS.h
 * This class defines the rotate at s local planner, which performs
 * a translation to the location "s" percent along the straight line
 * path, change all orientation DoFs, then translate to the goal
 */

#ifndef ROTATEATS_H_
#define ROTATEATS_H_

#include "TransformAtS.h"

template<class MPTraits>
class RotateAtS : public TransformAtS<MPTraits> {
  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    RotateAtS(double _s = 0.5);
    RotateAtS(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~RotateAtS();

  protected:

    virtual bool IsReversible() {
      return fabs(this->m_sValue - 0.5) < std::numeric_limits<double>::epsilon();
    }

    virtual void GetSequenceNodes(const CfgType& _c1, const CfgType& _c2,
        double _s, vector<CfgType>& _sequence, bool _reverse = false);
};

template<class MPTraits>
RotateAtS<MPTraits>::RotateAtS(double _s) : TransformAtS<MPTraits>(_s) {
  this->SetName("RotateAtS");
}

template<class MPTraits>
RotateAtS<MPTraits>::RotateAtS(MPProblemType* _problem, XMLNodeReader& _node):
    TransformAtS<MPTraits>(_problem, _node) {
  this->SetName("RotateAtS");
  this->m_sValue = _node.numberXMLParameter("s", true, 0.5, 0.0, 1.0, "Rotate at s value");
  _node.warnUnrequestedAttributes();
}

template<class MPTraits>
RotateAtS<MPTraits>::~RotateAtS() {}

template<class MPTraits>
void
RotateAtS<MPTraits>::GetSequenceNodes(const CfgType& _c1, const CfgType& _c2,
    double _s, vector<CfgType>& _sequence, bool _reverse) {

  CfgType thisCopy;
  vector<double> _v1 = _c1.GetData();
  thisCopy.SetData(_v1);
  _sequence.push_back(thisCopy);

  CfgType weightedSum;
  weightedSum.SetData(_v1);
  weightedSum.WeightedSum(_c1, _c2, _s);

  CfgType s1;
  s1.SetData(_v1);
  s1.GetPositionOrientationFrom2Cfg(weightedSum, _c1);
  _sequence.push_back(s1);

  CfgType s2;
  s2.SetData(_v1);
  s2.GetPositionOrientationFrom2Cfg(weightedSum, _c2);
  _sequence.push_back(s2);

  CfgType otherCopy;
  vector<double> _v2 = _c2.GetData();
  otherCopy.SetData(_v2);
  _sequence.push_back(otherCopy);
}

#endif

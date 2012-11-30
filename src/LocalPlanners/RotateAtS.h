/**
 * RotateAtS.h
 * This class defines the rotate at s local planner, which performs
 * a translation to the location "s" percent along the straight line
 * path, change all orientation DoFs, then translate to the goal
 */

#ifndef ROTATEATS_H_
#define ROTATEATS_H_

#include "TransformAtS.h"

template <class CFG, class WEIGHT> 
class RotateAtS : public TransformAtS<CFG, WEIGHT> {

  public:

    RotateAtS(double _s = 0.5);
    RotateAtS(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~RotateAtS();

  protected:

    virtual bool IsReversible() {return fabs(this->m_sValue - 0.5) < std::numeric_limits<double>::epsilon();} 
    virtual void GetSequenceNodes(const CFG& _c1, const CFG& _c2, double _s, 
        vector<CFG>& _sequence, bool _reverse = false); 
};

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::RotateAtS(double _s):
  TransformAtS<CFG, WEIGHT>(_s) {
    this->SetName("RotateAtS"); 
  }

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::RotateAtS(XMLNodeReader& _node, MPProblem* _problem): 
  TransformAtS<CFG, WEIGHT>(_node, _problem) {

    this->SetName("RotateAtS");
    this->m_sValue = _node.numberXMLParameter("s", true, 0.5, 0.0, 1.0, "Rotate at s value");

    _node.warnUnrequestedAttributes();
    
    if(this->m_debug)
      this->PrintOptions(cout);
  }

template <class CFG, class WEIGHT> 
RotateAtS<CFG, WEIGHT>::~RotateAtS() {}

template <class CFG, class WEIGHT> 
void
RotateAtS<CFG, WEIGHT>::GetSequenceNodes(const CFG& _c1, const CFG& _c2, double _s,
    vector<CFG>& _sequence, bool _reverse) {

  CFG thisCopy;
  vector<double> _v1 = _c1.GetData();
  thisCopy.SetData(_v1);
  _sequence.push_back(thisCopy);

  CFG weightedSum;
  weightedSum.SetData(_v1);
  weightedSum.WeightedSum(_c1, _c2, _s);

  CFG s1;
  s1.SetData(_v1);
  s1.GetPositionOrientationFrom2Cfg(weightedSum, _c1);
  _sequence.push_back(s1);

  CFG s2;
  s2.SetData(_v1);
  s2.GetPositionOrientationFrom2Cfg(weightedSum, _c2);
  _sequence.push_back(s2);

  CFG otherCopy;
  vector<double> _v2 = _c2.GetData();
  otherCopy.SetData(_v2);
  _sequence.push_back(otherCopy);
}

#endif

/*
 * =============================================================================
 *
 *       Filename:  BasicExtender.h
 *
 *    Description:  This is the standard way of expanding a tree toward a random
 *                  configuration. In this extend method, xrand is set to xrand.
 *
 * =============================================================================
 */
#ifndef BASICEXTENDER_H_
#define BASICEXTENDER_H_

#include "ExtenderMethod.h"

template<class MPTraits>
class BasicExtender : public ExtenderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    BasicExtender(const string& _dmLabel = "", const string& _vcLabel = "",
        double _delta = 1.0, bool _randomOrientation = true);
    BasicExtender(MPProblemType* _problem, XMLNodeReader& _node);

    void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os) const;

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, vector<CfgType>& _innerNodes);

    bool Expand(const CfgType& _start, const CfgType& _dir, CfgType& _newCfg,
        double _delta, int& _weight, double _posRes, double _oriRes);
    bool Expand(const CfgType& _start, const CfgType& _dir, CfgType& _newCfg,
        double _delta, int& _weight, CDInfo& _cdInfo, double _posRes, double _oriRes);

  protected:
    string m_dmLabel;
    string m_vcLabel;
    double m_delta;
    bool m_randomOrientation;
};

template<class MPTraits>
BasicExtender<MPTraits>::BasicExtender(const string& _dmLabel, const string& _vcLabel,
    double _delta, bool _randomOrientation) :
  ExtenderMethod<MPTraits>(), m_dmLabel(_dmLabel), m_vcLabel(_vcLabel), m_delta(_delta),
  m_randomOrientation(_randomOrientation) {
    this->SetName("BasicExtender");
  }

template<class MPTraits>
BasicExtender<MPTraits>::BasicExtender(MPProblemType* _problem, XMLNodeReader& _node) :
  ExtenderMethod<MPTraits>(_problem, _node) {
    this->SetName("BasicExtender");
    ParseXML(_node);
  }

template<class MPTraits>
void
BasicExtender<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_dmLabel = _node.stringXMLParameter("dmLabel",true,"","Distance metric label");
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity checker label");
  m_delta = _node.numberXMLParameter("delta", false, 1.0, 0.0, MAX_DBL, "Delta distance");
  m_randomOrientation = _node.boolXMLParameter("randomOrientation", false, true, "Random orientation");
}

template<class MPTraits>
void
BasicExtender<MPTraits>::PrintOptions(ostream& _os) const {
  ExtenderMethod<MPTraits>::PrintOptions(_os);
  _os << "\tdistance metric : \"" << m_dmLabel << "\"" << endl;
  _os << "\tvalidity checker : \"" << m_vcLabel << "\"" << endl;
  _os << "\tdelta = " << m_delta << endl;
  if(!m_randomOrientation)
    _os << "\torientation :\"Same\"" << endl;
}

template<class MPTraits>
bool
BasicExtender<MPTraits>::Extend(const CfgType& _near, const CfgType& _dir,
    CfgType& _new, vector<CfgType>& _innerNodes) {
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CfgType newDir = _dir;
  int weight;

  // Random/Same Orientation
  if(!m_randomOrientation)
    for(size_t i = newDir.PosDOF(); i < _dir.DOF(); i++)
      newDir[i] = _near[i];

  // Expand
  if(this->m_debug)
    cout << "expand" << endl;
  return Expand(_near, newDir, _new, m_delta, weight,
      env->GetPositionRes(), env->GetOrientationRes());
}

/*
 *  Expand
 *
 *  Basic utility for "extend" a RRT tree. Assumed to be given a start node,
 *  and he goal node to grow towards. Resulting node extended towards the goal
 *  is passed by reference and modified. Returned boolean relays whether the
 *  growth was succesful or not.
 *
 *  start  *> Location of Cfg on tree to grow from
 *  dir  *> Direction to grow towards
 *  new_cfg  *> New Cfg to be added to the tree; passed by reference
 *  delta  *> Maximum distance to grow
 *
 */
template<class MPTraits>
bool
BasicExtender<MPTraits>::Expand(const CfgType& _start, const CfgType& _dir, CfgType& _newCfg,
    double _delta, int& _weight, double _posRes, double _oriRes) {
  CDInfo cdInfo;
  return Expand(_start, _dir, _newCfg, _delta, _weight, cdInfo, _posRes, _oriRes);
}

template<class MPTraits>
bool
BasicExtender<MPTraits>::Expand(const CfgType& _start, const CfgType& _dir, CfgType& _newCfg,
    double _delta, int& _weight, CDInfo& _cdInfo, double _posRes, double _oriRes) {

  //Setup...primarily for collision checks that occur later on
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  string callee("BasicExtender::Expand");

  typename vector<CfgType>::iterator startCIterator;
  CfgType incr, tick = _start, previous = _start;
  bool collision = false;
  int nTicks, ticker = 0;

  incr.FindIncrement(tick,_dir,&nTicks, _posRes, _oriRes);
  _weight = nTicks;

  //Move out from start towards dir, bounded by number of ticks allowed at a
  //given resolution and the distance _delta: the maximum distance to grow
  while(!collision && dm->Distance(_start, tick) <= _delta && ticker <= nTicks) {
    previous = tick;
    tick += incr;
    if(!env->InBounds(tick) || !(vc->IsValid(tick, _cdInfo, callee)))
      collision = true; //return previous tick, as it is collision-free
    ++ticker;
  }
  if(previous != _start) {
    _newCfg = previous;//Last Cfg pushed back is the final tick allowed
    return true;
  }
  else{
    if(this->m_debug)
      cout << "Could not expand !" << endl;
    return false;
  }
}

#endif

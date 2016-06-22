#ifndef TRANSFORM_AT_S_H_
#define TRANSFORM_AT_S_H_

#include "StraightLine.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief Translates to \f$s\f$ along the path, then changes orientation and
///        joint @dofs one by one, then translates to goal.
/// @tparam MPTraits Motion planning universe
///
/// Translates to the location \f$s\f$ percent along the straight line path,
/// changes all orientation DoFs one by one, then translates to the goal.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class TransformAtS : public StraightLine<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    TransformAtS(double _s = 0.5, const string& _vcLabel = "", bool _evalation = false,
        bool _saveIntermediates = false);
    TransformAtS(MPProblemType* _problem, XMLNode& _node);
    virtual ~TransformAtS();

    virtual void Print(ostream& _os) const;

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    virtual vector<CfgType> ReconstructPath(
        const CfgType& _c1, const CfgType& _c2,
        const vector<CfgType>& _intermediates,
        double _posRes, double _oriRes);

  protected:

    virtual bool IsReversible() {return false;}

    virtual void GetSequenceNodes(const CfgType& _c1, const CfgType& _c2,
        double _s, vector<CfgType>& _sequence, bool _reverse = true);

    virtual bool IsConnectedOneWay(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false,
        bool forward = true);

    double m_sValue;
};

template <class MPTraits>
TransformAtS<MPTraits>::TransformAtS(double _s, const string& _vcLabel,
    bool _evalation, bool _saveIntermediates) :
  StraightLine<MPTraits>(_vcLabel, _evalation, _saveIntermediates), m_sValue(_s) {
  this->SetName("TransformAtS");
}

template <class MPTraits>
TransformAtS<MPTraits>::TransformAtS(MPProblemType* _problem, XMLNode& _node) :
    StraightLine<MPTraits>(_problem, _node) {
  this->SetName("TransformAtS");
  m_sValue = _node.Read("s", true, 0.5, 0.0, 1.0, "Transform at s value");
}

template <class MPTraits>
TransformAtS<MPTraits>::~TransformAtS() { }

// Prints options
template <class MPTraits>
void
TransformAtS<MPTraits>::Print(ostream& _os) const {
  StraightLine<MPTraits>::Print(_os);
  _os << "\tbinary evaluation = " << this->m_binaryEvaluation
      << "\n\ts = " << m_sValue
      << endl;
}

// Checks if two configurations can be connected, in both directions if
// necessary
template <class MPTraits>
bool
TransformAtS<MPTraits>::IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath) {

  // Clear _lpOutput
  _lpOutput->Clear();
  // Check first direction
  bool connected = this->IsConnectedOneWay(_c1, _c2, _col, _lpOutput,
      _posRes, _oriRes, _checkCollision, _savePath, true);
  // Check opposite direction if necessary and applicable
  if(!connected && !this->IsReversible())
    connected = IsConnectedOneWay(_c2, _c1, _col, _lpOutput,
        _posRes, _oriRes, _checkCollision, _savePath, false);

  // Output any good results
  if(connected) {
    _lpOutput->SetLPLabel(this->GetLabel());
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);
  }
  return connected;
}

template <class MPTraits>
void
TransformAtS<MPTraits>::GetSequenceNodes(const CfgType& _c1, const CfgType& _c2,
    double _s, vector<CfgType>& _sequence, bool _reverse) {
  CfgType thisCopy;
  vector<double> _v1 = _c1.GetData();
  thisCopy.SetData(_v1);
  _sequence.push_back(thisCopy);
  vector<double> translateData = _c1.GetData();

  if(_c1.PosDOF() > 0) {
    // Translate the robot base s way between start and goal, keeping
    // orientation fixed
    CfgType cfgAverage = thisCopy;
    cfgAverage.WeightedSum(_c1, _c2, _s);
    vector<double> averageData = cfgAverage.GetData();
    for(size_t i = 0; i < _c1.PosDOF(); i++)
      translateData[i] = averageData[i];
    CfgType cfgTranslate = thisCopy;
    cfgTranslate.SetData(translateData);
    _sequence.push_back(cfgTranslate);

    // Change all orientation DoF back to those of _c1
    translateData = _c1.GetData();
    for(size_t i = 0; i < _c1.PosDOF(); i++)
      translateData[i] = cfgTranslate.GetData()[i];
  }

  // Create intermediate configurations, replacing DoF i with goal DoF, order
  // depending on direction
  if(_reverse) {
    for(size_t i = _c1.PosDOF(); i < _c1.DOF(); i++) {
      translateData[i] = _c2.GetData()[i];
      thisCopy.SetData(translateData);
      _sequence.push_back(thisCopy);
    }
  }
  else {
    for(size_t i = _c1.DOF() - 1; i > _c1.PosDOF() - 1; i--) {
      translateData[i] = _c2.GetData()[i];
      thisCopy.SetData(translateData);
      _sequence.push_back(thisCopy);
    }
  }
  vector<double> _v2 = _c2.GetData();
  thisCopy.SetData(_v2);
  _sequence.push_back(thisCopy);
}

// Checks if two configurations can be connected in one direction
template<class MPTraits>
bool
TransformAtS<MPTraits>::IsConnectedOneWay(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, bool _forward) {
  string callee = this->GetNameAndLabel() + "::IsConnectedOneWay()";
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(this->m_vcLabel);
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  if(this->m_debug)
    cout << "Start CFG positional DOF: " << _c1.PosDOF() << endl;

  vector<CfgType> sequence;
  bool connected = true;
  int cdCounter = 0;
  GetSequenceNodes(_c1, _c2, this->m_sValue, sequence, _forward);

  if(this->m_debug) {
    //vector<double> tmp = _c1.GetData();
    for(typename vector<CfgType>::iterator J = sequence.begin(); J != sequence.end(); J++) {
      cout << "C" << distance(sequence.begin(), J) << ": " << *J << "end" << endl;
    }
  }

  // Check sequence nodes
  if(_checkCollision) {
    for(typename vector<CfgType>::iterator I = sequence.begin() + 1; I != sequence.end() - 1; I++)
    { // _c1 and _c2 not double checked
      cdCounter++;
      if(env->InBounds(*I)) {
        if(!vcm->IsValid(*I, callee)) {
          _col = *I;
          connected = false;
          break;
        }
      } else {
        connected = false;
        break;
      }
    }
  }

  // Check between sequence nodes
  for(typename vector<CfgType>::iterator I = sequence.begin();
      connected && I != sequence.end() - 1; I++) {
    if(this->m_binaryEvaluation)
      connected = this->IsConnectedSLBinary(*I, *(I + 1), _col, _lpOutput,
          cdCounter, _posRes, _oriRes,_checkCollision, _savePath);
    else
      connected = this->IsConnectedSLSequential(*I, *(I + 1), _col, _lpOutput,
          cdCounter, _posRes, _oriRes,_checkCollision, _savePath);
    // Save path if desired
    if((_savePath) &&
        (distance(sequence.begin(), I) + 1 != (int)sequence.size() - 1)) //Don't put _c2 on end
      _lpOutput->m_path.push_back(*(I + 1));
  }

  // Output any good results
  if(connected)
    for(typename vector<CfgType>::iterator I = sequence.begin(); I != sequence.end() - 1; I++) {
      _lpOutput->m_intermediates.push_back(*(I + 1));
    }
  if(connected)
    stats->IncLPConnections(this->GetNameAndLabel());
  stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);

  return connected;
}

// Returns the path
template<class MPTraits>
vector<typename TransformAtS<MPTraits>::CfgType>
TransformAtS<MPTraits>::ReconstructPath(
    const CfgType& _c1, const CfgType& _c2,
    const vector<CfgType>& _intermediates,
    double _posRes, double _oriRes) {

  int dummyCntr;
  LPOutput<MPTraits>* lpOutput = new LPOutput<MPTraits>();
  CfgType col;

  // Generate path between start, intermediates, and goal
  vector<CfgType> cfgList;
  cfgList.push_back(_c1);
  cfgList.insert(cfgList.end(), _intermediates.begin(), _intermediates.end());
  cfgList.push_back(_c2);

  for(typename vector<CfgType>::iterator I = cfgList.begin(); I != cfgList.end() - 1; I++) {
    if(this->m_binaryEvaluation)
      this->IsConnectedSLBinary(*I, *(I + 1), col, lpOutput,
          dummyCntr, _posRes, _oriRes, false, true);
    else
      this->IsConnectedSLSequential(*I, *(I + 1), col, lpOutput,
          dummyCntr, _posRes, _oriRes, false, true);
    if(distance(cfgList.begin(), I) != (int)cfgList.size() - 2)
      lpOutput->m_path.push_back(*(I + 1));
  }

  // Return final path
  vector<CfgType> path = lpOutput->m_path;
  delete lpOutput;
  return path;
}
#endif

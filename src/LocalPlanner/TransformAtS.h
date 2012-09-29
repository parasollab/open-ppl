// Translates to the location "s" percent along the straight line path,
// change all orientation DoFs one by one, then translate to the goal

#ifndef TRANSFORMATS_H_
#define TRANSFORMATS_H_

#include "StraightLine.h"

template <class CFG, class WEIGHT>
class TransformAtS : public StraightLine<CFG, WEIGHT> {

  public:

    TransformAtS(double _s = 0.5);
    TransformAtS(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~TransformAtS();

    virtual void PrintOptions(ostream& _os);

    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod> _dm,
        const CFG& _c1, const CFG& _c2,
        CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false, bool _saveFailedPath = false);

    virtual vector<CFG> ReconstructPath(Environment* _env, shared_ptr<DistanceMetricMethod> _dm, 
        const CFG& _c1, const CFG& _c2, const vector<CFG>& _intermediates, double _posRes, double _oriRes);

  protected:

    virtual bool IsReversible() {return false;}
    virtual void GetSequenceNodes(const CFG& _c1, const CFG& _c2, double _s,
        vector<CFG>& _sequence, bool _reverse = true);
    virtual bool IsConnectedOneWay(Environment* _env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod> _dm,
        const CFG& _c1, const CFG& _c2, CFG& _col, 
        LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false,
        bool _saveFailedPath = false, bool forward = true);

    double m_sValue;
};

template <class CFG, class WEIGHT>
TransformAtS<CFG, WEIGHT>::TransformAtS(double _s): 
  StraightLine<CFG, WEIGHT>(), m_sValue(_s) {
    this->SetName("TransformAtS");
  }

template <class CFG, class WEIGHT>
TransformAtS<CFG, WEIGHT>::TransformAtS(XMLNodeReader& _node, MPProblem* _problem):
  StraightLine<CFG, WEIGHT>(_node, _problem) {
    this->SetName("TransformAtS");
    m_sValue = _node.numberXMLParameter("s", true, 0.5, 0.0, 1.0, "Transform at s value");

    _node.warnUnrequestedAttributes();
    if(this->m_debug)
      PrintOptions(cout);
  }

template <class CFG, class WEIGHT> 
TransformAtS<CFG, WEIGHT>::~TransformAtS() { }

// Prints options
template <class CFG, class WEIGHT>
void
TransformAtS<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << this->GetName() <<  endl;
  _os << "\tbinarySearch = " << this->m_binarySearch << endl;
  _os << "\tvcMethod = " << this->m_vcMethod << endl;
  _os << "\tsValue = " << m_sValue << endl;
}

// Checks if two configurations can be connected, in both directions if necessary
template <class CFG, class WEIGHT>
bool
TransformAtS<CFG, WEIGHT>::IsConnected(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, CFG& _col, 
    LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {

  // Clear _lpOutput
  _lpOutput->Clear();
  // Check first direction
  bool connected = IsConnectedOneWay(_env, _stats, _dm, _c1, _c2, _col,
      _lpOutput, _posRes, _oriRes, _checkCollision, _savePath, _saveFailedPath, true);
  // Check opposite direction if necessary and applicable
  if(!connected && !IsReversible())
    connected = IsConnectedOneWay(_env, _stats, _dm, _c2, _c1, _col,
        _lpOutput, _posRes, _oriRes, _checkCollision, _savePath, _saveFailedPath, false);

  // Output any good results
  if(connected) {
    _lpOutput->SetLPLabel(this->GetLabel());
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);
  }
  return connected;
}

template <class CFG, class WEIGHT>
void
TransformAtS<CFG, WEIGHT>::GetSequenceNodes(const CFG& _c1, const CFG& _c2, double _s, 
    vector<CFG>& _sequence, bool _reverse) {
  CFG thisCopy;
  vector<double> _v1 = _c1.GetData();
  thisCopy.SetData(_v1);
  _sequence.push_back(thisCopy);
  vector<double> translateData = _c1.GetData();

  if(_c1.PosDOF() > 0) {
    // Translate the robot base s way between start and goal, keeping orientation fixed
    CFG cfgAverage = thisCopy;
    cfgAverage.WeightedSum(_c1, _c2, _s);
    vector<double> averageData = cfgAverage.GetData();
    for(size_t i = 0; i < _c1.PosDOF(); i++)
      translateData[i] = averageData[i];
    CFG cfgTranslate = thisCopy;
    cfgTranslate.SetData(translateData);
    _sequence.push_back(cfgTranslate);

    // Change all orientation DoF back to those of _c1
    translateData = _c1.GetData();
    for(size_t i = 0; i < _c1.PosDOF(); i++)
      translateData[i] = cfgTranslate.GetData()[i];
  }

  // Create intermediate configurations, replacing DoF i with goal DoF, order depending on direction
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
template <class CFG, class WEIGHT>
bool
TransformAtS<CFG,WEIGHT>::IsConnectedOneWay(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, CFG& _col, 
    LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath, bool _forward) {
  string callee = this->GetNameAndLabel() + "::IsConnectedOneWay()";
  typename ValidityChecker::ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker()->GetMethod(this->m_vcMethod);
  CDInfo cdInfo;

  if(this->m_debug)
    cout << "Start CFG positional DOF: " << _c1.PosDOF() << endl; 

  vector<CFG> sequence; 
  bool connected = true;
  int cdCounter = 0;
  GetSequenceNodes(_c1, _c2, m_sValue, sequence, _forward);

  if(this->m_debug) {
    //vector<double> tmp = _c1.GetData();
    for(typename vector<CFG>::iterator J = sequence.begin(); J != sequence.end(); J++) {
      cout << "C" << distance(sequence.begin(), J) << ": " << *J;
      cout << "end" << endl;
    }
  }

  // Check sequence nodes
  if(_checkCollision) {
    for(typename vector<CFG>::iterator I = sequence.begin()+1; I != sequence.end()-1; I++) { // _c1 and _c2 not double checked
      cdCounter++;
      if((*I).InBoundary(_env)) {
        if(!vcm->IsValid(*I, _env, _stats, cdInfo, &callee)) {
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
  for(typename vector<CFG>::iterator I = sequence.begin(); connected && I != sequence.end()-1; I++) {
    if(this->m_binarySearch)
      connected = this->IsConnectedSLBinary(_env, _stats, _dm, *I,
          *(I+1), _col, _lpOutput, cdCounter,
          _posRes, _oriRes,_checkCollision,
          _savePath, _saveFailedPath);
    else
      connected = this->IsConnectedSLSequential(_env, _stats, _dm, *I,
          *(I+1), _col, _lpOutput, cdCounter,
          _posRes, _oriRes,_checkCollision,
          _savePath, _saveFailedPath);
    // Save path if desired
    if((_savePath || _saveFailedPath) && (distance(sequence.begin(), I)+1 != (int)sequence.size() - 1)) //Don't put _c2 on end
      _lpOutput->path.push_back(*(I+1));
  }

  // Output any good results
  if(connected)
    for(typename vector<CFG>::iterator I = sequence.begin(); I != sequence.end()-1; I++) {
      _lpOutput->intermediates.push_back(*(I+1));
    }
  if(this->m_recordKeep) {
    if(connected)
      _stats.IncLPConnections(this->GetNameAndLabel());
    _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  }

  return connected;
}

// Returns the path
template <class CFG, class WEIGHT>
vector<CFG> 
TransformAtS<CFG, WEIGHT>::ReconstructPath(Environment* _env, shared_ptr<DistanceMetricMethod> _dm, 
    const CFG& _c1, const CFG& _c2, const vector<CFG>& _intermediates, double _posRes, double _oriRes) {

  StatClass dummyStats;
  int dummyCntr;
  LPOutput<CFG, WEIGHT>* lpOutput = new LPOutput<CFG, WEIGHT>();
  CFG col;

  // Generate path between start, intermediates, and goal
  vector<CFG> cfgList;
  cfgList.push_back(_c1);
  cfgList.insert(cfgList.end(), _intermediates.begin(), _intermediates.end());
  cfgList.push_back(_c2);

  for(typename vector<CFG>::iterator I = cfgList.begin(); I != cfgList.end()-1; I++) {
    if(this->m_binarySearch)
      this->IsConnectedSLBinary(_env, dummyStats, _dm, *I, *(I+1),
          col, lpOutput, dummyCntr, _posRes, _oriRes, false, true, false);
    else
      this->IsConnectedSLSequential(_env, dummyStats, _dm, *I, *(I+1),
          col, lpOutput, dummyCntr, _posRes, _oriRes, false, true, false);
    if(distance(cfgList.begin(), I) != (int)cfgList.size() - 2)
      lpOutput->path.push_back(*(I+1));
  }

  // Return final path
  vector<CFG> path = lpOutput->path;
  delete lpOutput;
  return path;
}

#endif

/**
 * RotateAtS.h
 * This class defines the rotate at s local planner, which performs
 * a translation to the location "s" percent along the straight line
 * path, change all orientation DoFs, then translate to the goal
 */

#ifndef ROTATEATS_H_
#define ROTATEATS_H_

#include "StraightLine.h"

template <class CFG, class WEIGHT> class RotateAtS:
public StraightLine<CFG, WEIGHT> {

  public:

    RotateAtS(double _s = 0.5);
    RotateAtS(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~RotateAtS();

    virtual void PrintOptions(ostream& _os);

    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod > _dm,
        const CFG& _c1, const CFG& _c2, CFG& _col, 
        LPOutput<CFG, WEIGHT>* _lpOutput,
        double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false, bool _saveFailedPath = false);

    virtual vector<CFG> ReconstructPath(Environment* _env, shared_ptr<DistanceMetricMethod> _dm, 
        const CFG& _c1, const CFG& _c2, const vector<CFG>& _intermediates, double _posRes, double _oriRes);
 
  protected:
  
    virtual bool IsConnectedOneWay(Environment* _env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod >_dm,
        const CFG& _c1, const CFG& _c2, CFG& _col, 
        LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false, bool _saveFailedPath = false);

    double m_sValue;
};

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::RotateAtS(double _s):
  StraightLine<CFG, WEIGHT>(), m_sValue(_s) { 
    this->SetName("RotateAtS"); 
  }

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::RotateAtS(XMLNodeReader& _node, MPProblem* _problem): 
  StraightLine<CFG, WEIGHT>(_node, _problem) {

    this->SetName("RotateAtS");
    m_sValue = _node.numberXMLParameter("s", true, 0.5, 0.0, 1.0, "Rotate at s value");

    _node.warnUnrequestedAttributes();
    if(this->m_debug)
      PrintOptions(cout);
  }

template <class CFG, class WEIGHT> RotateAtS<CFG, WEIGHT>::~RotateAtS() {}

template <class CFG, class WEIGHT> void
RotateAtS<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << this->GetName() <<  endl;
  _os << "\tm_binarySearch = " << this->m_binarySearch << endl;
  _os << "\tm_vcMethod = " << this->m_vcMethod << endl;
  _os << "\tm_sValue = " << m_sValue << endl;
}

// Checks if two configurations can be connected, in both directions if necessary
template <class CFG, class WEIGHT> bool
RotateAtS<CFG, WEIGHT>::IsConnected(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, CFG& _col,
    LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) { 

  // Clear _lpOutput
  _lpOutput->Clear();
  // Check first direction
  bool connected = IsConnectedOneWay(_env, _stats, _dm, _c1, _c2, _col,
      _lpOutput, _posRes, _oriRes, _checkCollision, _savePath, _saveFailedPath);
  // Check opposite direction if necessary and not symmetric
  if(!connected && fabs(m_sValue - 0.5) > 0.000001) {
    connected = IsConnectedOneWay(_env, _stats, _dm, _c2, _c1, _col,
        _lpOutput, _posRes, _oriRes, _checkCollision, _savePath, _saveFailedPath);
    // Reverse path and intermediates since they were generated in wrong direction
    // If nothing was generated (because connected == false), trivially do nothing
    if(_savePath)
      reverse(_lpOutput->path.begin(), _lpOutput->path.end());
    reverse(_lpOutput->intermediates.begin(), _lpOutput->intermediates.end());
  }

  // Output any good results
  if(connected) {
    _lpOutput->SetLPLabel(this->GetLabel());
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);
  }
  return connected;
}

// Checks if two configurations can be connected in a single direction
template <class CFG, class WEIGHT> bool
RotateAtS<CFG,WEIGHT>::IsConnectedOneWay(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, CFG& _col,
    LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {

  string callee = this->GetNameAndLabel() + "::IsConnectedOneWay()";
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->m_vcMethod);
  CDInfo cdInfo;

  if(this->m_recordKeep) 
    _stats.IncLPAttempts(this->GetNameAndLabel());

  vector<Cfg*> sequence;
  vector<double> sValues;
  sValues.push_back(m_sValue);
  _c1.GetMovingSequenceNodes(_c2, sValues, sequence);
  bool connected = true;
  int cdCounter = 0;

  // Check sequence nodes
  if(_checkCollision) {
    for(size_t i = 1; i < sequence.size() - 1; i++) { // _c1 and _c2 not double checked
      cdCounter++;
      if(sequence[i]->InBoundary(_env)) {
        if(!vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, false, &callee)) {
          _col = *sequence[i];
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
  for(size_t i = 0; connected && i < sequence.size() - 1; i++) {
    if(this->m_binarySearch)
      connected = this->IsConnectedSLBinary(_env, _stats, _dm, *sequence[i], 
          *sequence[i+1], _col, _lpOutput, cdCounter,
          _posRes, _oriRes, _checkCollision,
          _savePath, _saveFailedPath);
    else
      connected = this->IsConnectedSLSequential(_env, _stats, _dm, *sequence[i],
          *sequence[i+1], _col, _lpOutput, cdCounter,
          _posRes, _oriRes, _checkCollision,
          _savePath, _saveFailedPath);
    // Save path if desired
    if((_savePath || _saveFailedPath) && (i+1 != sequence.size() - 1)) // Don't put _c2 on end
      _lpOutput->path.push_back(*sequence[i+1]);
  }

  // Output any good results

  if(connected) {
    _lpOutput->intermediates.push_back(*sequence[1]);
    _lpOutput->intermediates.push_back(*sequence[2]);
  }
  if(this->m_recordKeep) {
    if(connected) 
      _stats.IncLPConnections(this->GetNameAndLabel());  
    _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  }

  // Since we use Vector<Cfg*>, we need to delete it
  for(size_t i = 0; i < sequence.size(); i++) 
    if(sequence[i] != NULL)
      delete sequence[i];
  return connected;
}

// Returns the path
template <class CFG, class WEIGHT> vector<CFG> 
RotateAtS<CFG, WEIGHT>::ReconstructPath(Environment* _env, shared_ptr<DistanceMetricMethod> _dm, 
    const CFG& _c1, const CFG& _c2, const vector<CFG>& _intermediates, double _posRes, double _oriRes) {

  StatClass dummyStats;
  int dummyCntr;
  LPOutput<CFG, WEIGHT>* lpOutput = new LPOutput<CFG, WEIGHT>();
  CFG col;

  // A list of important configurations: start, before rotation, after rotation, goal
  const CFG* cfgArr[4] = {&_c1, &_intermediates[0], &_intermediates[1], &_c2};
  // Generate the path between each pair of important configurations
  for(int i = 0; i < 3; i++) {
    if(this->m_binarySearch)
      this->IsConnectedSLBinary(_env, dummyStats, _dm, *cfgArr[i], *cfgArr[i+1],
          col, lpOutput, dummyCntr, _posRes, _oriRes, false, true, false);
    else
      this->IsConnectedSLSequential(_env, dummyStats, _dm, *cfgArr[i], *cfgArr[i+1],
          col, lpOutput, dummyCntr, _posRes, _oriRes, false, true, false);
    // The intermediate configurations haven't been added yet; do so now
    if(i < 2)
      lpOutput->path.push_back(_intermediates[i]);
  }

  // Return final path
  vector<CFG> path = lpOutput->path;
  delete lpOutput;
  return path;
}

#endif

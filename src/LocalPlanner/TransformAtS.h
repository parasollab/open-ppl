#ifndef TRANSFORMATS_H_
#define TRANSFORMATS_H_

#include "StraightLine.h"

template <class CFG, class WEIGHT>
class TransformAtS: public StraightLine<CFG, WEIGHT> {

  public:
    TransformAtS();
    TransformAtS(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~TransformAtS();

    virtual void PrintOptions(ostream& _os);
    virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

    virtual bool IsConnected(Environment *_env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod >_dm,
        const CFG &_c1, const CFG &_c2,
        CFG &_col, LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false);

  protected:

    virtual bool IsConnectedOneWay(Environment *_env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod >_dm,
        const CFG &_c1, const CFG &_c2, CFG &_col, 
        LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false);

    virtual bool IsConnectedOtherWay(Environment *_env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod >_dm,
        const CFG &_c1, const CFG &_c2,
        CFG &_col, LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false);  

    double m_sValue;
    //std::string vcMethod;
};

template <class CFG, class WEIGHT>
TransformAtS<CFG, WEIGHT>::TransformAtS(): 
StraightLine<CFG, WEIGHT>(){
  this->SetName("TransformAtS");
}

template <class CFG, class WEIGHT>
TransformAtS<CFG, WEIGHT>::
TransformAtS(XMLNodeReader& _node, MPProblem* _problem) : StraightLine<CFG, WEIGHT>(_node, _problem) {
  this->SetName("TransformAtS");
  m_sValue = _node.numberXMLParameter("s", true, 0.5, 0.0, 1.0,"transform at s value");
}

template <class CFG, class WEIGHT> TransformAtS<CFG, WEIGHT>::~TransformAtS() { }

template <class CFG, class WEIGHT>
void
TransformAtS<CFG, WEIGHT>::
PrintOptions(ostream& _os) {
  _os << "    " << this->GetName() << "::  ";
  _os << "binary search = " << " " << this->m_binarySearch << " ";
  _os << "vcMethod = " << " " << this->m_vcMethod << " ";
  _os << "s_value = " << m_sValue;
  _os << endl;
}

template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
TransformAtS<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new TransformAtS<CFG, WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT> bool
TransformAtS<CFG, WEIGHT>:: IsConnected(Environment *_env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod > _dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
    LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  //clear _lpOutput
  _lpOutput->Clear();
  bool connected = false;
  connected = IsConnectedOneWay(_env,_stats,
                  _dm,_c1,_c2,_col,_lpOutput,
                  _posRes,_oriRes,_checkCollision,
                  _savePath,_saveFailedPath);
  return connected;
}

template <class CFG, class WEIGHT> bool
TransformAtS<CFG,WEIGHT>::
IsConnectedOneWay(Environment *_env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod > _dm,
    const CFG &_c1, const CFG &_c2, CFG &_col, 
    LPOutput<CFG, WEIGHT>* _lpOutput,
    double _posRes, double _oriRes,
    bool _checkCollision,
    bool _savePath,
    bool _saveFailedPath) {  
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->m_vcMethod);

  int cdCounter = 0;
  vector<double> startData = _c1.GetData();
  vector<double> goalData = _c2.GetData();
  vector<double> tmp = _c1.GetData();
  vector<double> halfPosition = _c1.GetData();

  cout << "Start CFG positional DOF: " << _c1.PosDOF() << "\n" << flush; 

  vector<Cfg*> sequence;  

  sequence.push_back(_c1.CreateNewCfg());

  vector<double> translateData = startData;
  if (_c1.PosDOF() > 0 ) {
    // Translate the robot base s_value way between start and goal, keeping orientation fixed
    Cfg* cfgAverage = _c1.CreateNewCfg();
    cfgAverage->WeightedSum(_c1, _c2, m_sValue);
    vector<double> averageData = cfgAverage->GetData();
    for ( int i=0; i<_c1.PosDOF(); ++i )
      translateData[i] = averageData[i];

    Cfg* cfgTranslate = _c1.CreateNewCfg(translateData);
    sequence.push_back(cfgTranslate);

    halfPosition = cfgTranslate->GetData();
  }
  translateData = startData;

  for(int i=_c1.PosDOF(); i<_c1.DOF(); ++i) {
    //create intermediate cfg, replacing dof i with goal dof
    vector<double> intermediateData = translateData;
    intermediateData[0] = halfPosition[0];
    intermediateData[1] = halfPosition[1];
    intermediateData[2] = halfPosition[2];
    intermediateData[i] = goalData[i];
    Cfg* cfgIntermediate = _c1.CreateNewCfg(intermediateData);
    sequence.push_back(cfgIntermediate);

    //save change of dof to translateData
    translateData[i] = goalData[i];
  }

  sequence.push_back(_c2.CreateNewCfg());
  for (size_t j=0; j<sequence.size(); j++) {
    tmp = sequence[j]->GetData();
    cout << "C" << j << ": ";
    for (size_t k=0; k<tmp.size(); k++)
      cout << tmp[k] << ", ";
    cout << "end \n" << flush;
  }

  bool connected = true;

  if (_checkCollision) {
    string callee = this->GetName();
    string method = "-transform_at_s::IsConnected";
    CDInfo cdInfo;
    callee = callee + method;
    for(size_t i=1; i<sequence.size()-1; ++i) {
      cdCounter++;
      if ( (!sequence[i]->InBoundingBox(_env)) || 
          (!vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, true, &callee)))
        connected = IsConnectedOtherWay(_env,_stats,_dm,
            _c1,_c2,_col,_lpOutput,
            _posRes,_oriRes,_checkCollision,
            _savePath,_saveFailedPath);
    }
  }

  if (connected) {
    for ( size_t i=0; i<sequence.size()-1; ++i) {
      if ( this->m_binarySearch )
        connected = IsConnectedSLBinary(_env, _stats, _dm,
            *sequence[i], *sequence[i+1],
            _col, _lpOutput, cdCounter,
            _posRes, _oriRes,_checkCollision,
            _savePath, _saveFailedPath);
      else
        connected = IsConnectedSLSequential(_env, _stats, _dm,
            *sequence[i], *sequence[i+1],
            _col, _lpOutput, cdCounter,
            _posRes, _oriRes,_checkCollision,
            _savePath, _saveFailedPath);

      if ((_savePath || _saveFailedPath) && (i+1 != sequence.size()-1))
        _lpOutput->path.push_back(*sequence[i+1]);

      if ( !connected )
        break;
    }
  }

  if (connected)
    _stats.IncLPConnections(this->GetNameAndLabel());
  
  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);

  for(size_t i=0; i<sequence.size(); ++i)
    if(sequence[i] != NULL)
      delete sequence[i];

  return connected;
}

template <class CFG, class WEIGHT> bool
TransformAtS<CFG, WEIGHT>::
IsConnectedOtherWay(Environment *_env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod > _dm,
    const CFG &_c1, const CFG &_c2,
    CFG &_col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _posRes, double _oriRes,
    bool _checkCollision,
    bool _savePath,
    bool _saveFailedPath) {
  cout << "Check the other direction:\n" << flush;
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->m_vcMethod);

  int cdCounter = 0;
  vector<double> startData    = _c1.GetData();
  vector<double> goalData     = _c2.GetData();
  vector<double> tmp           = _c1.GetData();
  vector<double> halfPosition = _c1.GetData();

  cout << "Start CFG positional DOF: " << _c1.PosDOF() << "\n" << flush;

  vector<Cfg*> sequence;

  sequence.push_back(_c1.CreateNewCfg());

  vector<double> translateData = startData;
  if(_c1.PosDOF() > 0) {
    //translate the robot base s_value way between start and goal, keeping orientation fixed
    Cfg* cfgAverage = _c1.CreateNewCfg();
    cfgAverage->WeightedSum(_c1, _c2, m_sValue);
    vector<double> averageData = cfgAverage->GetData();
    for(int i=0; i<_c1.PosDOF(); ++i)
      translateData[i] = averageData[i];

    Cfg* cfgTranslate = _c1.CreateNewCfg(translateData);
    sequence.push_back(cfgTranslate);

    halfPosition = cfgTranslate->GetData();
  }

  translateData = startData;

  for(int i=_c1.DOF()-1; i>_c1.PosDOF()-1; --i) {
    //create intermediate cfg, replacing dof i with goal dof
    vector<double> intermediateData = translateData;
    intermediateData[0] = halfPosition[0];
    intermediateData[1] = halfPosition[1];
    intermediateData[2] = halfPosition[2];
    intermediateData[i] = goalData[i];
    Cfg* cfgIntermediate = _c1.CreateNewCfg(intermediateData);
    sequence.push_back(cfgIntermediate);

    //save change of dof to translate data
    translateData[i] = goalData[i];
  }

  sequence.push_back(_c2.CreateNewCfg());
  for(size_t j=0; j<sequence.size(); j++) {
    tmp = sequence[j]->GetData();
    cout << "C" << j << ": ";
    for(size_t k=0; k<tmp.size(); k++)
      cout << tmp[k] << ", ";

    cout << "end \n" << flush;
  }

  bool connected = true;

  if (_checkCollision) {
    string callee = this->GetName();
    string method = "-transform_at_s::IsConnected";
    CDInfo cdInfo;
    callee = callee + method;
    for(size_t i=1; i<sequence.size()-1; ++i) {
      cdCounter++;
      if ( (!sequence[i]->InBoundingBox(_env)) || 
          (!vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, true, &callee))) {
        if ( (sequence[i]->InBoundingBox(_env)) && 
            (!vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, true, &callee)))
          _col = *sequence[i];
        connected = false;
        break;
      }
    }
  }

  if (connected) {
    for(size_t i=0; i<sequence.size()-1; ++i) {
      if (this->m_binarySearch)
        connected = IsConnectedSLBinary(_env, _stats, _dm,
            *sequence[i], *sequence[i+1],
            _col, _lpOutput, cdCounter,
            _posRes, _oriRes, _checkCollision,
            _savePath, _saveFailedPath);
      else
        connected = IsConnectedSLSequential(_env, _stats, _dm,
            *sequence[i], *sequence[i+1],
            _col, _lpOutput, cdCounter,
            _posRes, _oriRes,_checkCollision,
            _savePath, _saveFailedPath);

      if ((_savePath || _saveFailedPath) && (i+1 != sequence.size()-1))
        _lpOutput->path.push_back(*sequence[i+1]);

      if ( !connected )
        break;
    }
  }

  if (connected)
    _stats.IncLPConnections(this->GetNameAndLabel());
  
  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);

  for(size_t i=0; i<sequence.size(); ++i)
    if(sequence[i] != NULL)
      delete sequence[i];

  return connected;
};
#endif

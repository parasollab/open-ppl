#ifndef TRANSFORMATS_H_
#define TRANSFORMATS_H_

#include "StraightLine.h"

template <class CFG, class WEIGHT>
class TransformAtS: public StraightLine<CFG, WEIGHT> {

 public:
  TransformAtS();
  TransformAtS(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~TransformAtS();

  virtual void PrintOptions(ostream& out_os);
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  virtual bool IsConnected(Environment *env, Stat_Class& Stats,
    shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
    LPOutput<CFG, WEIGHT>* lpOutput, double positionRes, double orientationRes,
    bool checkCollision=true, bool savePath=false, bool saveFailedPath=false);

 protected:

  virtual bool IsConnectedOneWay(Environment *env, Stat_Class& Stats,
    shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
    LPOutput<CFG, WEIGHT>* lpOutput, double positionRes, double orientationRes,
    bool checkCollision=true, bool savePath=false, bool saveFailedPath=false);

  virtual bool IsConnectedOtherWay(Environment *env, Stat_Class& Stats,
	  shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
    LPOutput<CFG, WEIGHT>* lpOutput, double positionRes, double orientationRes,
	  bool checkCollision=true, bool savePath=false, bool saveFailedPath=false);  

  double s_value;
  //std::string vcMethod;
};

template <class CFG, class WEIGHT>
TransformAtS<CFG, WEIGHT>::TransformAtS(): 
StraightLine<CFG, WEIGHT>(){
  this->SetName("TransformAtS");
}

template <class CFG, class WEIGHT>
TransformAtS<CFG, WEIGHT>::
TransformAtS(XMLNodeReader& in_Node, MPProblem* in_pProblem) : StraightLine<CFG, WEIGHT>(in_Node, in_pProblem) {
  this->SetName("TransformAtS");
  s_value = in_Node.numberXMLParameter("s", true, 0.5, 0.0, 1.0,"transform at s value");
}

template <class CFG, class WEIGHT> TransformAtS<CFG, WEIGHT>::~TransformAtS() { }

template <class CFG, class WEIGHT>
void
TransformAtS<CFG, WEIGHT>::
PrintOptions(ostream& out_os) {
  out_os << "    " << this->GetName() << "::  ";
  out_os << "line segment length = " << " " << this->lineSegmentLength << " ";
  out_os << "binary search = " << " " << this->binarySearch << " ";
  out_os << "vcMethod = " << " " << this->m_vcMethod << " ";
  out_os << "s_value = " << s_value;
  out_os << endl;
}

template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
TransformAtS<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new TransformAtS<CFG, WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT> bool
TransformAtS<CFG, WEIGHT>:: IsConnected(Environment *_env, Stat_Class& _stats,
shared_ptr<DistanceMetricMethod > _dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  //clear _lpOutput
  _lpOutput->path.clear();
  _lpOutput->edge.first.SetWeight(0);
  _lpOutput->edge.second.SetWeight(0);
  _lpOutput->savedEdge.clear();
  bool connected = false;
  connected = IsConnectedOneWay(_env,_stats,_dm,_c1,_c2,_col,_lpOutput,_posRes,_oriRes,_checkCollision,_savePath,_saveFailedPath);
  return connected;
}

template <class CFG, class WEIGHT> bool
TransformAtS<CFG,WEIGHT>:: IsConnectedOneWay(Environment *_env, Stat_Class& _stats,
shared_ptr<DistanceMetricMethod > _dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
bool _checkCollision, bool _savePath, bool _saveFailedPath) {  
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->m_vcMethod);

  int cd_cntr = 0;
  vector<double> start_data = _c1.GetData();
  vector<double> goal_data = _c2.GetData();
  vector<double> tmp = _c1.GetData();
  vector<double> half_position = _c1.GetData();

  cout << "Start CFG positional DOF: " << _c1.PosDOF() << "\n" << flush; 

  vector<Cfg*> sequence;  

  sequence.push_back(_c1.CreateNewCfg());

  vector<double> translate_data = start_data;
  if (_c1.PosDOF() > 0 ) {
    // Translate the robot base s_value way between start and goal, keeping orientation fixed
    Cfg* cfg_average = _c1.CreateNewCfg();
    cfg_average->WeightedSum(_c1, _c2, s_value);
    vector<double> average_data = cfg_average->GetData();
    for ( int i=0; i<_c1.PosDOF(); ++i )
      translate_data[i] = average_data[i];

    Cfg* cfg_translate = _c1.CreateNewCfg(translate_data);
    sequence.push_back(cfg_translate);

    half_position = cfg_translate->GetData();
  }
  translate_data = start_data;

  for(int i=_c1.PosDOF(); i<_c1.DOF(); ++i)
  {
    //create intermediate cfg, replacing dof i with goal dof
    vector<double> intermediate_data = translate_data;
    intermediate_data[0] = half_position[0];
    intermediate_data[1] = half_position[1];
    intermediate_data[2] = half_position[2];
    intermediate_data[i] = goal_data[i];
    Cfg* cfg_intermediate = _c1.CreateNewCfg(intermediate_data);
    sequence.push_back(cfg_intermediate);

    //save change of dof to translate_data
    translate_data[i] = goal_data[i];
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

  if ( _checkCollision ) {
    string Callee(this->GetName());
    string Method("-transform_at_s::IsConnected");
    CDInfo cdInfo;
    Callee = Callee + Method;
    for(size_t i=1; i<sequence.size()-1; ++i) {
      cd_cntr++;
      if ( (!sequence[i]->InBoundingBox(_env)) || 
           (!vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, true, &Callee)))
        connected = IsConnectedOtherWay(_env,_stats,_dm,_c1,_c2,_col,_lpOutput,_posRes,_oriRes,_checkCollision,_savePath,_saveFailedPath);
    }
  }

  if ( connected ) {
    for ( size_t i=0; i<sequence.size()-1; ++i) {
      if ( this->binarySearch )
        connected = IsConnectedSLBinary(_env, _stats, _dm, *sequence[i], *sequence[i+1],
                                        _col, _lpOutput, cd_cntr, _posRes, _oriRes,
                                        _checkCollision, _savePath, _saveFailedPath);
      else
        connected = IsConnectedSLSequential(_env, _stats, _dm, *sequence[i], *sequence[i+1],
                                            _col, _lpOutput, cd_cntr, _posRes, _oriRes,
                                            _checkCollision, _savePath, _saveFailedPath);

      if ((_savePath || _saveFailedPath) && (i+1 != sequence.size()-1))
        _lpOutput->path.push_back(*sequence[i+1]);

      if ( !connected )
        break;
    }
  }

  if ( connected ) _stats.IncLPConnections("Transform_At_S");
  _stats.IncLPCollDetCalls("Transform_At_Se", cd_cntr);

  for(size_t i=0; i<sequence.size(); ++i)
    if(sequence[i] != NULL)
      delete sequence[i];

  return connected;
}

template <class CFG, class WEIGHT> bool
TransformAtS<CFG, WEIGHT>::IsConnectedOtherWay(Environment *_env, Stat_Class& _stats,
shared_ptr<DistanceMetricMethod > _dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
LPOutput<CFG, WEIGHT>* _lpOutput, double _posRes, double _oriRes,
bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  cout << "Check the other direction:\n" << flush;
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->m_vcMethod);

  int cd_cntr = 0;
  vector<double> start_data    = _c1.GetData();
  vector<double> goal_data     = _c2.GetData();
  vector<double> tmp           = _c1.GetData();
  vector<double> half_position = _c1.GetData();

  cout << "Start CFG positional DOF: " << _c1.PosDOF() << "\n" << flush;

  vector<Cfg*> sequence;

  sequence.push_back(_c1.CreateNewCfg());

  vector<double> translate_data = start_data;
  if(_c1.PosDOF() > 0) {
    //translate the robot base s_value way between start and goal, keeping orientation fixed
    Cfg* cfg_average = _c1.CreateNewCfg();
    cfg_average->WeightedSum(_c1, _c2, s_value);
    vector<double> average_data = cfg_average->GetData();
    for(int i=0; i<_c1.PosDOF(); ++i)
      translate_data[i] = average_data[i];

    Cfg* cfg_translate = _c1.CreateNewCfg(translate_data);
    sequence.push_back(cfg_translate);

    half_position = cfg_translate->GetData();
  }

  translate_data = start_data;

  for(int i=_c1.DOF()-1; i>_c1.PosDOF()-1; --i)
  {
    //create intermediate cfg, replacing dof i with goal dof
    vector<double> intermediate_data = translate_data;
    intermediate_data[0] = half_position[0];
    intermediate_data[1] = half_position[1];
    intermediate_data[2] = half_position[2];
    intermediate_data[i] = goal_data[i];
    Cfg* cfg_intermediate = _c1.CreateNewCfg(intermediate_data);
    sequence.push_back(cfg_intermediate);

    //save change of dof to translate data
    translate_data[i] = goal_data[i];
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
  
  if ( _checkCollision ) {
    string Callee(this->GetName());
    string Method("-transform_at_s::IsConnected");
    CDInfo cdInfo;
    Callee = Callee + Method;
    for(size_t i=1; i<sequence.size()-1; ++i) {
      cd_cntr++;
      if ( (!sequence[i]->InBoundingBox(_env)) || 
           (!vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, true, &Callee))) {
        if ( (sequence[i]->InBoundingBox(_env)) && 
             (!vc->IsValid(vcm, *sequence[i], _env, _stats, cdInfo, true, &Callee)))
          _col = *sequence[i];
        connected = false;
        break;
      }
    }
  }

  if ( connected ) {
    for(size_t i=0; i<sequence.size()-1; ++i) {
      if (this->binarySearch)
        connected = IsConnectedSLBinary(_env, _stats, _dm, *sequence[i], *sequence[i+1],
                                        _col, _lpOutput, cd_cntr, _posRes, _oriRes,
                                        _checkCollision, _savePath, _saveFailedPath);
      else
        connected = IsConnectedSLSequential(_env, _stats, _dm, *sequence[i], *sequence[i+1],
                                            _col, _lpOutput, cd_cntr, _posRes, _oriRes,
                                            _checkCollision, _savePath, _saveFailedPath);

      if ((_savePath || _saveFailedPath) && (i+1 != sequence.size()-1))
        _lpOutput->path.push_back(*sequence[i+1]);

      if ( !connected )
        break;
    }
  }

  if ( connected ) _stats.IncLPConnections("Transform_At_S");
  _stats.IncLPCollDetCalls("Transform_At_S", cd_cntr);

  for(size_t i=0; i<sequence.size(); ++i)
    if(sequence[i] != NULL)
      delete sequence[i];

  return connected;
};
#endif

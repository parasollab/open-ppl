#ifndef TransformAtS_h
#define TransformAtS_h

#include "StraightLine.h"

template <class CFG, class WEIGHT>
class TransformAtS: public StraightLine<CFG, WEIGHT> {

 public:
  
  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{

  ///Default Constructor.
  TransformAtS(cd_predefined _cdtype, int s = 0.5);
  TransformAtS(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warnUnrequestedXml = true);
  ///Destructor.  
  virtual ~TransformAtS();

  //@}
  //////////////////////
  // Access
  virtual char* GetName() const;
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual void PrintOptions(ostream& out_os);

  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  virtual bool IsConnected(Environment *env, Stat_Class& Stats,
         shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
         LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision=true, 
         bool savePath=false, bool saveFailedPath=false);


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

 protected:

  virtual 
    bool IsConnectedOneWay(Environment *env, Stat_Class& Stats,
         shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, 
         LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision=true, 
         bool savePath=false, bool saveFailedPath=false);

  virtual 
    bool IsConnectedOtherWay(Environment *env, Stat_Class& Stats,
	 shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2,
	 LPOutput<CFG, WEIGHT>* lpOutput,
	 double positionRes, double orientationRes,
	 bool checkCollision=true,
	 bool savePath=false, bool saveFailedPath=false);  

  //@{
    double s_value;
  //@}

};

/////////////////////////////////////////////////////////////////////
//
//  definitions for class RotateAtS declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
TransformAtS<CFG, WEIGHT>::
TransformAtS(cd_predefined _cdtype, int s) : StraightLine<CFG, WEIGHT>(_cdtype), s_value(s) {
  SetDefault(); 
}

template <class CFG, class WEIGHT>
TransformAtS<CFG, WEIGHT>::
TransformAtS(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warnUnrequestedXml) : StraightLine<CFG, WEIGHT>(_cdtype, in_Node, in_pProblem, false) 
{
  this->cdtype = _cdtype;
  LOG_DEBUG_MSG("TransformAtS::TransformAtS()");
  s_value = in_Node.numberXMLParameter(string("s"), true, 0.5, 0.0, 1.0, string("transform at s value"));
  if(warnUnrequestedXml)
    in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~TransformAtS::TransformAtS()");
}

template <class CFG, class WEIGHT>
TransformAtS<CFG, WEIGHT>::
~TransformAtS() {
}



template <class CFG, class WEIGHT>
void
TransformAtS<CFG, WEIGHT>::
PrintOptions(ostream& out_os) {
  out_os << "    " << GetName() << "::  ";
  out_os << "line segment length = " << " " << this->lineSegmentLength << " ";
  out_os << "binary search = " << " " << this->binarySearch << " ";
  out_os << "vcMethod = " << " " << this->vcMethod << " ";
  out_os << "s_value = " << s_value;
  out_os << endl;
}



template <class CFG, class WEIGHT>
char*
TransformAtS<CFG, WEIGHT>::
GetName() const {
  return "transform_at_s";
}

template <class CFG, class WEIGHT>
void
TransformAtS<CFG, WEIGHT>::
SetDefault() {
  StraightLine<CFG, WEIGHT>::SetDefault();
  s_value = 0.5;
}



template <class CFG, class WEIGHT>
void
TransformAtS<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t" << this->lineSegmentLength;
  _os << "\n\t" << this->binarySearch;
  _os << "\n\t" << s_value;
 
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
TransformAtS<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os << GetName() << " ";
  _os << "ineSegmentLength" << " " << this->lineSegmentLength << " ";
  _os << "binarySearch" << " " << this->binarySearch << " ";
  _os << "s" << " " << s_value << " ";
 _os << endl;
}


template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
TransformAtS<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new TransformAtS<CFG, WEIGHT>(*this);
  return _copy;

}

template <class CFG, class WEIGHT>
bool
TransformAtS<CFG, WEIGHT>::
IsConnected(Environment *_env, Stat_Class& Stats,
     shared_ptr<DistanceMetricMethod >dm,
     const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
     double positionRes, double orientationRes,
     bool checkCollision,
     bool savePath, bool saveFailedPath) {
  bool connected = false;
  connected = IsConnectedOneWay(_env, Stats, dm, _c1, _c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
  return connected;
}

template <class CFG, class WEIGHT>
bool
TransformAtS<CFG,WEIGHT>::
IsConnectedOneWay(Environment *_env, Stat_Class& Stats,
      shared_ptr<DistanceMetricMethod >dm,
      const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
      double positionRes, double orientationRes,
      bool checkCollision, 
      bool savePath, bool saveFailedPath) {  
  CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection();

  int cd_cntr = 0;
  vector<double> start_data = _c1.GetData();
  vector<double> goal_data = _c2.GetData();
  vector<double> tmp = _c1.GetData();
  vector<double> half_position = _c1.GetData();

  cout << "Start CFG positional DOF: " << _c1.posDOF() << "\n" << flush; 

  vector<Cfg*> sequence;  

  sequence.push_back(_c1.CreateNewCfg());

  vector<double> translate_data = start_data;
  if(_c1.posDOF() > 0)
  {
    //translate the robot base s_value way between start and goal, keeping orientation fixed
    Cfg* cfg_average = _c1.CreateNewCfg();
    cfg_average->WeightedSum(_c1, _c2, s_value);
    vector<double> average_data = cfg_average->GetData();
    for(int i=0; i<_c1.posDOF(); ++i)
    {
      translate_data[i] = average_data[i];
    }
    Cfg* cfg_translate = _c1.CreateNewCfg(translate_data);
    sequence.push_back(cfg_translate);

    half_position = cfg_translate->GetData();
  }
  translate_data = start_data;

  for(int i=_c1.posDOF(); i<_c1.DOF(); ++i)
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
  for(size_t j=0; j<sequence.size(); j++)
  {
    tmp = sequence[j]->GetData();
    cout << "C" << j << ": ";
    for(size_t k=0; k<tmp.size(); k++) {
      cout << tmp[k] << ", ";
    }
    cout << "end \n" << flush;
  }

  bool connected = true;

  if(checkCollision)
  {
    std::string Callee(GetName());
    std::string Method("-transform_at_s::IsConnected");
    Callee = Callee + Method;
    for(size_t i=1; i<sequence.size()-1; ++i)
    {
      cd_cntr++;
      if((!sequence[i]->InBoundingBox(_env)) || (sequence[i]->isCollision(_env, Stats, cd, *this->cdInfo, true, &(Callee))))
      {
	connected = IsConnectedOtherWay(_env, Stats, dm, _c1, _c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
      }
    }
  }

  if(connected)
  {
    for(size_t i=0; i<sequence.size()-1; ++i)
    {
      if(this->binarySearch)
        connected = IsConnectedSLBinary(_env, Stats, dm, *sequence[i], *sequence[i+1],
					lpOutput, cd_cntr, positionRes, orientationRes,
      					checkCollision, savePath, saveFailedPath);
      else
        connected = IsConnectedSLSequential(_env, Stats, dm, *sequence[i], *sequence[i+1],
					   lpOutput, cd_cntr, positionRes, orientationRes,
					   checkCollision, savePath, saveFailedPath);

      if((savePath || saveFailedPath) && (i+1 != sequence.size()-1))
        lpOutput->path.push_back(*sequence[i+1]);

      if(!connected)
        break;
    }
  }

  if(connected)
  {
    Stats.IncLPConnections("Transform_At_S");
  }
  Stats.IncLPCollDetCalls("Transform_At_Se", cd_cntr);

  for(size_t i=0; i<sequence.size(); ++i)
  {
    if(sequence[i] != NULL)
      delete sequence[i];
  }

  return connected;

}

template <class CFG, class WEIGHT>
bool
TransformAtS<CFG, WEIGHT>::
IsConnectedOtherWay(Environment *_env, Stat_Class& Stats,
      shared_ptr<DistanceMetricMethod >dm,
      const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
      double positionRes, double orientationRes,
      bool checkCollision,
      bool savePath, bool saveFailedPath) {
  cout << "Check the other direction:\n" << flush;
  CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection();

  int cd_cntr = 0;
  vector<double> start_data = _c1.GetData();
  vector<double> goal_data = _c2.GetData();
  vector<double> tmp = _c1.GetData();
  vector<double> half_position = _c1.GetData();

  cout << "Start CFG positional DOF: " << _c1.posDOF() << "\n" << flush;

  vector<Cfg*> sequence;

  sequence.push_back(_c1.CreateNewCfg());

  vector<double> translate_data = start_data;
  if(_c1.posDOF() > 0) 
  {
    //translate the robot base s_value way between start and goal, keeping orientation fixed
    Cfg* cfg_average = _c1.CreateNewCfg();
    cfg_average->WeightedSum(_c1, _c2, s_value);
    vector<double> average_data = cfg_average->GetData();
    for(int i=0; i<_c1.posDOF(); ++i)
    {
      translate_data[i] = average_data[i];
    }
    Cfg* cfg_translate = _c1.CreateNewCfg(translate_data);
    sequence.push_back(cfg_translate);

    half_position = cfg_translate->GetData();
  }

  translate_data = start_data;

  for(int i=_c1.DOF()-1; i>_c1.posDOF()-1; --i)
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
  for(size_t j=0; j<sequence.size(); j++)
  {
    tmp = sequence[j]->GetData();
    cout << "C" << j << ": ";
    for(int k=0; k<tmp.size(); k++) {
      cout << tmp[k] << ", ";
    }
    cout << "end \n" << flush;
  }

  bool connected = true;
  
  if(checkCollision)
  {
    std::string Callee(GetName());
    std::string Method("-transform_at_s::IsConnected");
    Callee = Callee + Method;
    for(size_t i=1; i<sequence.size()-1; ++i)
    {
      cd_cntr++;
      if((!sequence[i]->InBoundingBox(_env)) || (sequence[i]->isCollision(_env, Stats, cd, *this->cdInfo, true, &(Callee))))
      {
        connected = false;
        break;
      }
    }
  }

  if(connected)
  {
    for(size_t i=0; i<sequence.size()-1; ++i)
    {
      if(this->binarySearch)
        connected = IsConnectedSLBinary(_env, Stats, dm, *sequence[i], *sequence[i+1],
					lpOutput, cd_cntr, positionRes, orientationRes,
      					checkCollision, savePath, saveFailedPath);
      else
        connected = IsConnectedSLSequential(_env, Stats, dm, *sequence[i], *sequence[i+1],
					   lpOutput, cd_cntr, positionRes, orientationRes,
					   checkCollision, savePath, saveFailedPath);

      if((savePath || saveFailedPath) && (i+1 != sequence.size()-1))
        lpOutput->path.push_back(*sequence[i+1]);

      if(!connected)
        break;
    }
  }

  if(connected)
  {
    Stats.IncLPConnections("Transform_At_S");
  }
  Stats.IncLPCollDetCalls("Transform_At_S", cd_cntr);

  for(size_t i=0; i<sequence.size(); ++i)
  {
    if(sequence[i] != NULL)
      delete sequence[i];
  }

  return connected;

};

#endif

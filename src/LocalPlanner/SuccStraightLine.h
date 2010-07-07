#ifndef SuccStraightLine_h
#define SuccStraightLine_h

#include "StraightLine.h"

template <class CFG, class WEIGHT>
class SuccStraightLine: public StraightLine<CFG, WEIGHT> {

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
  SuccStraightLine(cd_predefined _cdtype);
  SuccStraightLine(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warnUnrequestedXml = true);
  ///Destructor.  
  virtual ~SuccStraightLine();

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
         DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
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
         DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
         LPOutput<CFG, WEIGHT>* lpOutput,
         double positionRes, double orientationRes,
         bool checkCollision=true, 
         bool savePath=false, bool saveFailedPath=false);

  //@{
    //double s_value;
    //vector<double> s_values;
    bool isSymmetric;
  //@}


  int binarySearch;
  int lineSegmentLength;
  std::string vcMethod;
};

/////////////////////////////////////////////////////////////////////
//
//  definitions for class RotateAtS declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
SuccStraightLine<CFG, WEIGHT>::
SuccStraightLine(cd_predefined _cdtype) : StraightLine<CFG, WEIGHT>(_cdtype) {
  SetDefault(); 
}

template <class CFG, class WEIGHT>
SuccStraightLine<CFG, WEIGHT>::
SuccStraightLine(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warnUnrequestedXml) : StraightLine<CFG, WEIGHT>(_cdtype, in_Node, in_pProblem, false) 
{
  this->cdtype = _cdtype;
  LOG_DEBUG_MSG("SuccStraightLine::SuccStraightLine()");
  //double nSValue = in_Node.numberXMLParameter(string("s"), true, 0.5, 0.0, 1.0, string("rotate at s value"));
  //s_values.push_back(nSValue);
  lineSegmentLength = in_Node.numberXMLParameter(string("length"), false, 0, 0, 5000, string("lineSegmentLength"));
  binarySearch = in_Node. numberXMLParameter(string("binary_search"), false, 0, 0, 1, string("binary search"));
  vcMethod = in_Node.stringXMLParameter(string("vc_method"), false, string(""""), string("Validity Test Method"));
  if(warnUnrequestedXml)
    in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~SuccStraightLine::SuccStraightLine()");
}

template <class CFG, class WEIGHT>
SuccStraightLine<CFG, WEIGHT>::
~SuccStraightLine() {
}



template <class CFG, class WEIGHT>
void
SuccStraightLine<CFG, WEIGHT>::
PrintOptions(ostream& out_os) {
  out_os << "    " << GetName() << "::  ";
  out_os << "line segment length = " << " " << this->lineSegmentLength << " ";
  out_os << "binary search = " << " " << this->binarySearch << " ";
  out_os << "vcMethod = " << " " << this->vcMethod << " ";
  //out_os << "s_value = " << s_values[0];
  out_os << endl;
}



template <class CFG, class WEIGHT>
char*
SuccStraightLine<CFG, WEIGHT>::
GetName() const {
  return "successive_straight_line";
}

template <class CFG, class WEIGHT>
void
SuccStraightLine<CFG, WEIGHT>::
SetDefault() {
  StraightLine<CFG, WEIGHT>::SetDefault();
  //s_values.clear();
  //s_values.push_back(sValue); 
  isSymmetric = true;
}



template <class CFG, class WEIGHT>
void
SuccStraightLine<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t" << this->lineSegmentLength;
  _os << "\n\t" << this->binarySearch;
  //_os << "\n\t" << s_values[0];
 
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
SuccStraightLine<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os << GetName() << " ";
  _os << "ineSegmentLength" << " " << this->lineSegmentLength << " ";
  _os << "binarySearch" << " " << this->binarySearch << " ";
/*
  for(int i=0; i<s_values.size(); i++){
   _os << "s" << " " << s_values[i] << " ";
   }
*/
 _os << endl;
}


template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
SuccStraightLine<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new SuccStraightLine<CFG, WEIGHT>(*this);
  return _copy;

}

template <class CFG, class WEIGHT>
bool
SuccStraightLine<CFG, WEIGHT>::
IsConnected(Environment *_env, Stat_Class& Stats,
     DistanceMetric *dm,
     const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
     double positionRes, double orientationRes,
     bool checkCollision,
     bool savePath, bool saveFailedPath) {
  bool connected = false;
  connected = IsConnectedOneWay(_env, Stats, dm, _c1, _c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
  if(!connected && !isSymmetric) {  //try the other way
    connected = IsConnectedOneWay(_env, Stats, dm, _c2, _c1, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    if(savePath)
      reverse(lpOutput->path.begin(), lpOutput->path.end());
  }
  return connected;
}

template <class CFG, class WEIGHT>
bool
SuccStraightLine<CFG,WEIGHT>::
IsConnectedOneWay(Environment *_env, Stat_Class& Stats,
      DistanceMetric *dm,
      const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
      double positionRes, double orientationRes,
      bool checkCollision, 
      bool savePath, bool saveFailedPath) {  
  CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection();

  int cd_cntr = 0;
  vector<double> start_data = _c1.GetData();
  vector<double> goal_data = _c2.GetData();

  vector<Cfg*> sequence;  

  sequence.push_back(_c1.CreateNewCfg());

  vector<double> translate_data = start_data;
  if(_c1.posDOF() > 0)
  {
    //translate the robot base 1/2 way between start and goal, keeping orientation fixed
    Cfg* cfg_average = _c1.CreateNewCfg();
    cfg_average->WeightedSum(_c1, _c2, 0.5);
    vector<double> average_data = cfg_average->GetData();
    for(int i=0; i<_c1.posDOF(); ++i)
    {
      translate_data[i] = average_data[i];
    }
    Cfg* cfg_translate = _c1.CreateNewCfg(translate_data);
    sequence.push_back(cfg_translate);
  }

  for(int i=0; i<_c1.posDOF(); ++i)
  {
    //create intermediate cfg, replacing dof i with goal dof
    vector<double> intermediate_data = translate_data;
    intermediate_data[i] = goal_data[i];
    Cfg* cfg_intermediate = _c1.CreateNewCfg(intermediate_data);
    sequence.push_back(cfg_intermediate);
    //save change of dof to translate_data
    translate_data[i] = goal_data[i];
  }

  sequence.push_back(_c2.CreateNewCfg());

  bool connected = true;

  if(checkCollision)
  {
    std::string Callee(GetName());
    std::string Method("-successive_straight_line::IsConnected");
    Callee = Callee + Method;
    for(int i=1; i<sequence.size()-1; ++i)
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
    for(int i=0; i<sequence.size()-1; ++i)
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
    Stats.IncLPConnections("Successive_Straightline");
  }
  Stats.IncLPCollDetCalls("Successive_Straightline", cd_cntr);

  for(int i=0; i<sequence.size(); ++i)
  {
    if(sequence[i] != NULL)
      delete sequence[i];
  }

  return connected;

};

#endif

#ifndef RotateAtS_h
#define RotateAtS_h

#include "StraightLine.h"

template <class CFG, class WEIGHT>
class RotateAtS: public StraightLine<CFG, WEIGHT> {

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
  RotateAtS(cd_predefined _cdtype);
  ///Destructor.	
  virtual ~RotateAtS();

  //@}
  virtual bool SameParameters(const LocalPlannerMethod<CFG,WEIGHT> &other) const;

  //////////////////////
  // Access
  virtual char* GetName() const;
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  virtual bool IsConnected(Environment *env, Stat_Class& Stats,
			   CollisionDetection *,
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
			   CollisionDetection *cd,
			   DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
			   LPOutput<CFG, WEIGHT>* lpOutput,
			   double positionRes, double orientationRes,
			   bool checkCollision=true, 
			   bool savePath=false, bool saveFailedPath=false);

  //@{
    num_param<double> sValue;
    double s_value;
    //vector<double> s_values_in;
    vector<double> s_values;
    bool isSymmetric;
  //@}
};

/////////////////////////////////////////////////////////////////////
//
//  definitions for class RotateAtS declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::
RotateAtS(cd_predefined _cdtype) : StraightLine<CFG, WEIGHT>(_cdtype),
  sValue ("s", 0.5, 0, 1) {
  sValue.PutDesc("FLOAT	", "(def, s=0.5)");
  SetDefault(); 
}

template <class CFG, class WEIGHT>
RotateAtS<CFG, WEIGHT>::
~RotateAtS() {
}


template <class CFG, class WEIGHT>
bool
RotateAtS<CFG, WEIGHT>::
SameParameters(const LocalPlannerMethod<CFG,WEIGHT> &other) const {
  bool result = false;
  if (sValue.GetValue() == ((RotateAtS<CFG,WEIGHT>&) other).sValue.GetValue())
    result = true;
  return result;
}


template <class CFG, class WEIGHT>
char*
RotateAtS<CFG, WEIGHT>::
GetName() const {
  return "rotate_at_s";
}

template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
SetDefault() {
  StraightLine<CFG, WEIGHT>::SetDefault();
  sValue.PutValue(0.5);
  s_values.clear();
  s_values.push_back(sValue.GetValue()); 
  //s_values_in = s_values;
  isSymmetric = true;
}

template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
ParseCommandLine(int argc, char **argv) {
  s_values.clear();

  for (int i = 1; i < argc; ++i) {
    if( lineSegmentLength.AckCmdLine(&i, argc, argv) ) {
    } else if( binarySearch.AckCmdLine(&i, argc, argv) ) {
    } else if( sValue.AckCmdLine(&i, argc, argv) ) {
      s_values.push_back(sValue.GetValue());

    } else {
      cerr << "\nERROR ParseCommandLine: Don\'t understand \"";
      for(int j=0; j<argc; j++)
        cerr << argv[j] << " ";
      cerr << "\"\n\n";
      PrintUsage(cerr);
      cerr << endl;
      exit (-1);
    }
  }
  if (s_values.size()<1){
    SetDefault();
  }
  std::sort(s_values.begin (), s_values.end());
  //s_values_in = s_values;
  
  if (s_values.size()>=1) {
    if (s_values[0] < 0.0 || s_values[s_values.size()-1] > 1.0) {
      cerr << "\nERROR ParseCommandLine: s values out of range (0.0,1.0) \"";
      PrintUsage(cerr);
      cerr << endl;
      exit(-1);
    }
  } else {
      cerr << "\nERROR ParseCommandLine: no value of s inserted \"";
      PrintUsage(cerr);
      cerr << endl;
      exit(-1);
  }

  vector<double> reverse = s_values;
  transform(reverse.begin(), reverse.end(), reverse.begin(), bind1st(minus<double>(), 1));
  std::sort(reverse.begin(), reverse.end());
  if(reverse == s_values)
    isSymmetric = true;
  else
    isSymmetric = false;
}


template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; sValue.PrintUsage(_os);
 
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os << GetName() << " ";
  for(int i=0; i<s_values.size(); i++){
   _os << sValue.GetFlag() << " " << s_values[i] << " ";
   }
 _os << endl;
}


template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
RotateAtS<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new RotateAtS<CFG, WEIGHT>(*this);
  return _copy;

}

template <class CFG, class WEIGHT>
bool
RotateAtS<CFG,WEIGHT>::
IsConnected(Environment *_env, Stat_Class& Stats,
	    CollisionDetection *cd, DistanceMetric *dm,
	    const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
	    double positionRes, double orientationRes,
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {  
  bool connected = false;
  connected = IsConnectedOneWay(_env, Stats, cd, dm, _c1, _c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
  if (!connected && !isSymmetric) { //try the other way
/*
    for(int i=0; i<s_values.size(); i++){
      s_values[i] = 1 - s_values[i];
    }
    std::sort(s_values.begin(),s_values.end());
    connected = IsConnectedOneWay(_env, Stats, cd, dm, _c1, _c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
*/
    connected = IsConnectedOneWay(_env, Stats, cd, dm, _c2, _c1, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    if (savePath)
      reverse(lpOutput->path.begin(), lpOutput->path.end());
/*
    for(int i=0; i<s_values.size(); i++){
      s_values[i] = 1 - s_values[i];
    }
    std::sort(s_values.begin(),s_values.end());
*/
  }
  return connected;
}

template <class CFG, class WEIGHT>
bool
RotateAtS<CFG,WEIGHT>::
IsConnectedOneWay(Environment *_env, Stat_Class& Stats, CollisionDetection *cd, DistanceMetric *dm,
	    const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
	    double positionRes, double orientationRes,
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {  
  char RatS[50] = "Rotate_at_s";
/*
  sprintf(RatS,"%s=%3.1f",RatS, s_values_in[0]);
  for(int i=1; i<s_values_in.size(); i++){
    sprintf(RatS,"%s,%3.1f",RatS, s_values_in[i]);
  }
*/
  sprintf(RatS,"%s=%3.1f",RatS, s_values[0]);
  for(int i=1; i<s_values.size(); ++i)
    sprintf(RatS,"%s,%3.1f",RatS, s_values[i]);
  Stats.IncLPAttempts( RatS );
  int cd_cntr= 0;
   
  if(this->lineSegmentLength.GetValue() && lineSegmentInCollision(_env, Stats, cd, dm, _c1, _c2, lpOutput, cd_cntr, positionRes)) {
    Stats.IncLPCollDetCalls( RatS, cd_cntr );
    return false;
  }
  
  vector<Cfg *> sequence;
  _c1.GetMovingSequenceNodes(_c2, s_values, sequence);
  
  bool connected = true;
  int i;
  for(i=0; i<sequence.size()-1; ++i) {
    bool flag;
    if(this->binarySearch.GetValue()) 
      flag = IsConnectedSLBinary(_env, Stats, cd, dm, *sequence[i], *sequence[i+1], lpOutput, cd_cntr, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    else
      flag = IsConnectedSLSequential(_env, Stats, cd, dm, *sequence[i], *sequence[i+1], lpOutput, cd_cntr, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    if(!flag) {
      connected = false;
      break;
    }
  }
  if(connected)
    Stats.IncLPConnections( RatS );
  
  Stats.IncLPCollDetCalls( RatS, cd_cntr );
  
  // Since we use vector<Cfg*>, we need to delete it
  for(i=0; i<sequence.size(); ++i) 
    if (sequence[i] != NULL)
      delete sequence[i];
  
  return connected;
};
#endif

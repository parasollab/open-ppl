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

  virtual bool IsConnected(Environment *env,CollisionDetection *,
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
    bool IsConnectedOneWay(Environment *env,CollisionDetection *cd,
			   DistanceMetric *dm, const CFG &_c1, const CFG &_c2, 
			   LPOutput<CFG, WEIGHT>* lpOutput,
			   double positionRes, double orientationRes,
			   bool checkCollision=true, 
			   bool savePath=false, bool saveFailedPath=false);

  //@{
  //    double sValue;  ///< Should in [0,1]. This is the s for planner.
    num_param<double> sValue;
    double s_value;
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
}

template <class CFG, class WEIGHT>
void
RotateAtS<CFG, WEIGHT>::
ParseCommandLine(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {
    if( sValue.AckCmdLine(&i, argc, argv) ) {
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
  _os << sValue.GetFlag() << " " << sValue.GetValue() << " ";
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
IsConnected(Environment *_env, CollisionDetection *cd, DistanceMetric *dm,
	    const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
	    double positionRes, double orientationRes,
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {  
  bool connected = false;
  s_value = sValue.GetValue();
  connected = IsConnectedOneWay(_env, cd, dm, _c1, _c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
  if (!connected) { //try the other way
    s_value = 1 - sValue.GetValue();
    connected = IsConnectedOneWay(_env, cd, dm, _c2, _c1, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    if (savePath)
      reverse(lpOutput->path.begin(), lpOutput->path.end());
  }
  return connected;
  
}

template <class CFG, class WEIGHT>
bool
RotateAtS<CFG,WEIGHT>::
IsConnectedOneWay(Environment *_env, CollisionDetection *cd, DistanceMetric *dm,
	    const CFG &_c1, const CFG &_c2, LPOutput<CFG, WEIGHT>* lpOutput,
	    double positionRes, double orientationRes,
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {  
  char RatS[20] = "Rotate_at_s";
  sprintf(RatS,"%s=%3.1f",RatS, sValue.GetValue());
  Stats.IncLPAttempts( RatS );
  int cd_cntr= 0;
  
  if(lineSegmentLength.GetValue() && lineSegmentInCollision(_env, cd, dm, _c1, _c2, lpOutput, cd_cntr, positionRes)) {
    Stats.IncLPCollDetCalls( RatS, cd_cntr );
    return false;
  }
  
  vector<Cfg *> sequence;
  _c1.GetMovingSequenceNodes(_c2, s_value, sequence);
  
  bool connected = true;
  int i;
  for(i=0; i<sequence.size()-1; ++i) {
    bool flag;
    if(binarySearch.GetValue()) 
      flag = IsConnectedSLBinary(_env, cd, dm, *sequence[i], *sequence[i+1], lpOutput, cd_cntr, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    else
      flag = IsConnectedSLSequential(_env, cd, dm, *sequence[i], *sequence[i+1], lpOutput, cd_cntr, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    if(!flag) {
      connected = false;
      break;
    }
  }
  if(connected)
    Stats.IncLPConnections( RatS );
  
  Stats.IncLPCollDetCalls( RatS, cd_cntr );
  
  // Since we use vector<Cfg*>, we need to delete it
  for(i=0; i<sequence.size();i++) {
    if (sequence[i] != NULL)
      delete sequence[i];
  }
  
  return connected;
};

#endif

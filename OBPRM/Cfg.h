// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Cfg.h
//
//  General Description
//	Configuration Data Class, it has all the interface needed
//	by other Motion Planning classes. Since it is abstract, it
//	will have to 'ask' a helper class called CfgManager to 
//	provide implementation to some specific functions.
//
//  Created
//	08/31/99	Guang Song
//
//  Last Modified By:
//      08/31/99        Lucia K. Dale - name change for a method
//
/////////////////////////////////////////////////////////////////////

#ifndef Cfg_h
#define Cfg_h

#include <math.h>
#include "Transformation.h"
#include "VectorConstantSize.h"
#include "Vectors.h"
#include "CollisionDetection.h"
// brc changes below
#include "Input.h"
#include <stdio.h>
#ifdef HPUX
#include <sys/io.h>
#endif

class Body;
class AttEnvironment;
class CollisionDetection;
class CfgManager;


//
// Information about the cfg, for example the obstacle info.
//
class  InfoCfg {
public:
   // Constructors and Destructor
   InfoCfg():obst(NULL_INFO){};
   InfoCfg(int i):obst(i){};

   bool operator== (const InfoCfg&tmp) const{return obst==tmp.obst;};
   bool operator!= (const InfoCfg&tmp) const{return obst!=tmp.obst;};

   // Methods
   void SetObst(int i){obst = i;};
   int GetObst() const {return obst;};

   // Data
   static const int NULL_INFO =-1;//= -1;
   int obst;
};


class Cfg {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg();
  Cfg(double x,double y,double z);
  Cfg(double x,double y,double z, double roll,double pitch,double yaw);
  Cfg(const Vector6<double> &v2);
  Cfg(const vector<double> &v2);
  ~Cfg();

  //===================================================================
  //  operators
  //===================================================================

    bool operator== (const Cfg&) const;
    bool operator!= (const Cfg&) const;
    Cfg operator+ (const Cfg&) const;
    Cfg operator- (const Cfg&) const;
    Cfg operator- () const;
    Cfg operator* (double);
    friend ostream& operator<< (ostream&, const Cfg &);
    friend istream& operator>> (istream&, Cfg &);

    static Cfg WeightedSum(const Cfg&,
	   const Cfg&, double weight = 0.5);
    bool AlmostEqual(const Cfg& _c);
    bool isWithinResolution(const Cfg &c, double positionRes, double orientationRes);

  //===================================================================
  //  Other Methods
  //===================================================================
  vector<double> GetData() const;
  Vector3D GetRobotCenterPosition();

  // Return the number of degrees of freedom for the configuration class
  int DOFs();

  // Return the range of a single parameter of the configuration (i.e., range of x)
  // param = the parameter to get the range for
  // In the future, this function should get the range for x,y,z by the bounding box
  // Currently it assumes the range for a position parameter to be -10000 to 10000
  // and the range for an orientation parameter to be 0 to 1, which should
  // also be changed to reflect any self collision in a linked robot
  pair<double,double> SingleParamRange(int param);

  // Set a single parameter in the configuration (i.e., x,y,z,roll...)
  // param = the parameter number to set
  // value = the value to set the parameter as
  int SetSingleParam(int param, double value);

  // Increment a single parameter in the configuration (i.e., x,y,z,roll...)
  // param = the parameter number to set
  // value = the value to increment the parameter by
  int IncSingleParam(int param, double value);
 
  // Retreive a single parameter in the configuration (i.e., x,y,z,roll...)
  // param = the parameter number to retreive
  double GetSingleParam(int param);

  Cfg ClosestPtOnLineSegment(const Cfg&, const Cfg&) const;
  static Cfg InvalidData();
  static Cfg c1_towards_c2(Cfg cfg1, Cfg cfg2, double d);
  static Cfg GetRandomRay(double incr);

  // for rotate-at-s Local Planner.
  vector<Cfg> GetMovingSequenceNodes(const Cfg& other, double s) const; 

  // methods for nodes connection.
  vector<Cfg> FindNeighbors(AttEnvironment *env, 
			const Cfg& increment,CollisionDetection *,
	int noNeighbors, SID  _cdsetid);
  vector<Cfg> FindNeighbors(AttEnvironment *env, const Cfg& goal,
	const Cfg& increment, CollisionDetection *,int noNeighbors, SID  _cdsetid);
  void IncrementTowardsGoal(const Cfg &goal, const Cfg &increment);
  Cfg  FindIncrement(const Cfg& _goal, int * n_ticks,
	                          double positionRes, double orientationRes);
  Cfg  FindIncrement(const Cfg& _goal, int  n_ticks);
  void Increment(const Cfg& _increment);

  // methods for Distance Metric.
  double  OrientationMagnitude();
  double  PositionMagnitude();


  // generates a random configuration without consideration of bounding box restrictions
  static Cfg GetRandomCfg(double R, double rStep);

  // generates random configuration where workspace robot's CENTER OF MASS (COM)
  // is guaranteed to lie within the environment specified bounding box
  static Cfg GetRandomCfg_COM(double *boundingBox);

  // generates random configuration where workspace robot's EVERY VERTEX
  // is guaranteed to lie within the environment specified bounding box
  static Cfg GetRandomCfg(AttEnvironment *env);

  // tests whether or not robot in this configuration has every vertex inside
  // the environment specified bounding box
  bool InBoundingBox(AttEnvironment *env);


  // methods for Cfg generation and collision checking.
  double Clearance(AttEnvironment *env,CollisionDetection* cd);
  bool ConfigEnvironment(AttEnvironment *env);
  bool isCollision(AttEnvironment *env,CollisionDetection* cd, SID _cdsetid);
  bool isCollision(AttEnvironment *env, CollisionDetection *cd,int robot, int obs, SID _cdsetid);
  static bool GenerateOverlapCfg(AttEnvironment *env, int robot, 
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg);  // OBPRM and BasicOBPRM
  static vector<Cfg> GenSurfaceCfgs4ObstNORMAL(AttEnvironment * env,
         CollisionDetection *,int obstacle, int nCfgs, SID _cdsetid);

  // new: printing methods.
  void print_vizmo_format_to_file(AttEnvironment *env, FILE *_fp);
  static void print_preamble_to_file(AttEnvironment *env, FILE *_fp, int numofCfg);

  //===================================================================
  //  Data          
  //===================================================================
  protected:
    Cfg operator/ (double);

    void Normalize_orientation();

  private:

    vector<double> v;	
    static CfgManager * CfgHelper;

  public:
    InfoCfg info;

    friend Input;
    friend CfgManager;



}; // class Cfg


#endif

// $Id$
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

/**
 * @file Cfg.h
 * @date 8/31/1999
 * @author Guang Song
 */

#ifndef Cfg_h
#define Cfg_h

#include <math.h>
#include "Transformation.h"
#include "VectorConstantSize.h"
#include "Vectors.h"
#include "CollisionDetection.h"
#include "Input.h"
#include <stdio.h>

#ifdef HPUX
  #include <sys/io.h>
#endif

class Body;
class Environment;
class CollisionDetection;
class CfgManager;


class DistanceMetric;
struct CDInfo;


//
/// Information about the cfg, for example the obstacle info.
//
class  InfoCfg {
public:
   /// Constructors and Destructor
   InfoCfg():obst(NULL_INFO),tag(NULL_INFO),clearance(NULL_INFO){};
   InfoCfg(int _obst):obst(_obst),tag(NULL_INFO),clearance(NULL_INFO){};
   InfoCfg(int _obst,int _tag):obst(_obst),tag(_tag),clearance(NULL_INFO){};
   InfoCfg(int _obst,int _tag,double _cl):obst(_obst),tag(_tag),clearance(_cl){};

   bool operator== (const InfoCfg&tmp) const{return obst==tmp.obst && tag==tmp.tag && clearance==tmp.clearance;};
   bool operator!= (const InfoCfg&tmp) const{return obst!=tmp.obst || tag!=tmp.tag || clearance!=tmp.clearance;};

   /// Methods
   void SetObst(int _obst){obst = _obst;};
   int GetObst() const {return obst;};
   void SetClearance(double _cl){clearance = _cl;};
   double GetClearance() const {return clearance;};

   void Write(ostream &os) const;
   void Read(istream &is);

   /// Data
   static const int NULL_INFO =-1;
   int obst;
   int tag;
   double clearance;
};
//---------------------------------------------
/// Input/Output operators for InfoCfg
//---------------------------------------------
istream& operator>> (istream&s, InfoCfg &_c);
ostream& operator<< (ostream&s, const InfoCfg &_c);


class Cfg {
public:

  //===================================================================
  ///  Constructors and Destructor
  //===================================================================

  Cfg();
  Cfg(double x,double y,double z);
  Cfg(double x,double y,double z, double roll,double pitch,double yaw);
  Cfg(const Vector6<double> &v2);
  Cfg(const vector<double> &v2);
  ~Cfg();

  //===================================================================
  ///  operators
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
  ///  Other Methods
  //===================================================================
  void Write(ostream &os) const;
  void Read(istream  &is);
  vector<double> GetData() const;
  Vector3D GetRobotCenterPosition();

  /// Return the number of degrees of freedom for the configuration class
  static int DOFs();

  /// Return the range of a single parameter of the configuration (i.e., range of x)
  /// param = the parameter to get the range for
  /// In the future, this function should get the range for x,y,z by the bounding box
  /// Currently it assumes the range for a position parameter to be -10000 to 10000
  /// and the range for an orientation parameter to be 0 to 1, which should
  /// also be changed to reflect any self collision in a linked robot
  pair<double,double> SingleParamRange(int param);

  /// Set a single parameter in the configuration (i.e., x,y,z,roll...)
  /// param = the parameter number to set
  /// value = the value to set the parameter as
  int SetSingleParam(int param, double value);

  /// Increment a single parameter in the configuration (i.e., x,y,z,roll...)
  /// param = the parameter number to set
  /// value = the value to increment the parameter by
  int IncSingleParam(int param, double value);

  /// Retreive a single parameter in the configuration (i.e., x,y,z,roll...)
  /// param = the parameter number to retreive
  double GetSingleParam(int param);

  Cfg ClosestPtOnLineSegment(const Cfg&, const Cfg&) const;
  static Cfg InvalidData();
  static Cfg c1_towards_c2(Cfg cfg1, Cfg cfg2, double d);
  static Cfg GetRandomRay(double incr);

  /// for rotate-at-s Local Planner.
  vector<Cfg> GetMovingSequenceNodes(const Cfg& other, double s) const;

  /// methods for nodes connection.
  vector<Cfg> FindNeighbors(
	Environment *env, const Cfg& increment,
	CollisionDetection *,
	int noNeighbors, 
	SID  _cdsetid, CDInfo& _cdInfo);

  vector<Cfg> FindNeighbors(
	Environment *env, const Cfg& goal, const Cfg& increment, 
	CollisionDetection *,
	int noNeighbors, 
	SID  _cdsetid, CDInfo& _cdInfo);

  void IncrementTowardsGoal(const Cfg &goal, const Cfg &increment);
  Cfg  FindIncrement(const Cfg& _goal, int * n_ticks,
	                          double positionRes, double orientationRes);
  Cfg  FindIncrement(const Cfg& _goal, int  n_ticks);
  void Increment(const Cfg& _increment);

  vector<double>  GetPosition();
  vector<double>  GetOrientation();
  /// methods for Distance Metric.
  double  OrientationMagnitude();
  double  PositionMagnitude();
  


  /// generates a random configuration without consideration of bounding box restrictions
  static Cfg GetRandomCfg(double R, double rStep);

  /// generates random configuration where workspace robot's CENTER OF MASS
  /// is guaranteed to lie within the environment specified bounding box
  static Cfg GetRandomCfg_CenterOfMass(double *boundingBox);

  /// generates random configuration where workspace robot's EVERY VERTEX
  /// is guaranteed to lie within the environment specified bounding box
  static Cfg GetRandomCfg(Environment *env, int maxTries);

  /// ditto, but with a default number of tries (10)
  static Cfg GetRandomCfg(Environment *env);

  static Cfg GetFreeRandomCfg(Environment *env,CollisionDetection* cd,
        SID _cdsetid, CDInfo& _cdInfo);

  /// tests whether or not robot in this configuration has every vertex inside
  /// the environment specified bounding box
  bool InBoundingBox(Environment *env);


  /// methods for Cfg generation and collision checking.
  double Clearance(Environment *env,CollisionDetection* cd);

  /// Approximate C-Space Clearance
  double ApproxCSpaceClearance(Environment *env, CollisionDetection *cd, 
	SID cdsetid, CDInfo& cdInfo, 
	DistanceMetric * dm, SID dmsetid, int n);

  bool ConfigEnvironment(Environment *env);
  bool isCollision(Environment *env,CollisionDetection* cd, 
	SID _cdsetid, CDInfo& _cdInfo);
  bool isCollision(Environment *env, CollisionDetection *cd,int robot, int obs, 	SID _cdsetid, CDInfo& _cdInfo);
  static bool GenerateOverlapCfg(Environment *env, int robot,
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg);  // OBPRM and BasicOBPRM
  static vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs, 
	SID _cdsetid, CDInfo& _cdInfo);

  /// printing methods.
  void printLinkConfigurations(Environment *env, vector<Vector6D> &cfigs);
  static void print_preamble_to_file(Environment *env, FILE *_fp, int numofCfg);

  /// return a configuration(conformation)'s potential.
  double Potential(Environment *env) const;
  //===================================================================
  ///  Data
  //===================================================================
  protected:
    Cfg operator/ (double);

    void Normalize_orientation();

  private:

    vector<double> v;	

  public:
    static CfgManager * CfgHelper;
    InfoCfg info;

    friend CfgManager;



}; // class Cfg


#endif

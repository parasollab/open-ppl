// $Id$
/////////////////////////////////////////////////////////////////////
//
//  CfgManager.h
//
//  General Description
//	This class provides some implmentations which could be 
//	different for different Cfg types, and some methods do require
//	a derived Cfg provide its own version(pure virtual methods).
//	This is an abstract class.
//	         
//		  CfgManager   <--------  Cfg
//		  /   |     \ 
//		_/    |      \_
//	Cfg_free      |         Cfg_free_serial<int>
//		  Cfg_fixed_PRR
//
//  BE AWARE: 
//      It is ASSUMED that all position values are packed together
//      in the first 'posDof' spots of a Cfg data. Other choices 
//	would require the corrrsponding derived class have its own
//	version of most member functions listed here. 
//
//  Created
//	08/31/99	Guang Song
//
//  Last Modified By:
//
/////////////////////////////////////////////////////////////////////

#ifndef CfgManager_h
#define CfgManager_h

#include <math.h>
#include "Transformation.h"
#include "VectorConstantSize.h"
#include "Vectors.h"
#include "CollisionDetection.h"


class Body;
class Environment;
class Cfg;

class CfgManager {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  CfgManager(int _dof, int _posDof) : dof(_dof), posDof(_posDof) {}
  ~CfgManager();


  //===================================================================
  //  Other Methods
  //===================================================================
  inline int GetDOF() { return dof; }
  virtual Vector3D GetRobotCenterPosition(const Cfg & c) const = 0;
  virtual pair<double,double> SingleParamRange(int param);
  virtual void Normalize_orientation(Cfg &c);
  virtual bool AlmostEqual(const Cfg&c1, const Cfg& c2);
  virtual bool isWithinResolution(const Cfg&c1, const Cfg&c2, 
                            double positionRes, double orientationRes);
  virtual Cfg InvalidData();
  virtual Cfg GetRandomCfg(double R, double rStep) = 0;
  virtual Cfg GetRandomCfg_COM(double *boundingBox) = 0;
  virtual Cfg GetRandomRay(double incr) = 0;
  virtual Cfg GetPositionOrientationFrom2Cfg(
	const Cfg& c1, const Cfg& c2);  // rotate-at-s helper
  virtual vector<Cfg> GetMovingSequenceNodes(const Cfg& c1, const Cfg& c2,
		      double s); // for rotate-at-s Local Planner.

  // methods for nodes connection.
  virtual vector<Cfg> FindNeighbors(const Cfg& c, Environment *env, 
			const Cfg& increment,CollisionDetection *,
			int noNeighbors, SID  _cdsetid);
  virtual vector<Cfg> FindNeighbors(const Cfg& c, Environment *env, const Cfg& goal,
	const Cfg& increment, CollisionDetection *,int noNeighbors, SID  _cdsetid);
  virtual void IncrementTowardsGoal(Cfg& c, const Cfg &goal, const Cfg &increment);
  virtual Cfg FindIncrement(const Cfg& c, const Cfg& _goal, int * n_ticks, 
                            double positionRes, double orientationRes);
  virtual Cfg FindIncrement(const Cfg& c, const Cfg& _goal, int  n_ticks);

  // methods for Distance Metric.
  virtual double  OrientationMagnitude(const Cfg& c);
  virtual double  PositionMagnitude(const Cfg& c);

  // methods for Cfg generation and collision checking.
  virtual bool ConfigEnvironment(const Cfg &c, Environment *env) = 0;
  virtual bool isCollision(const Cfg &c, Environment *env, CollisionDetection *cd,
                           SID _cdsetid, MultiBody * onflyRobot);

  // Node Generation methods
  virtual bool GenerateOverlapCfg(Environment *env, int robot, 
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg) = 0; // OBPRM 
  virtual vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs, SID _cdsetid) = 0; // NORMAL

  // printing methods.
  virtual void writeTransformation(FILE *_fp, Transformation & tmp);
  virtual void print_vizmo_format_to_file(const Cfg &c, Environment *env, FILE *_fp);
  virtual void print_preamble_to_file(Environment *env, FILE *_fp, int numofCfg);

  
  protected:
	// dof: Degree of Freedom, posDof: DOF for positions.
	int dof;
	int posDof;

  private:



} ; 
#endif

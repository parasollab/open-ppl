// $Id$
/////////////////////////////////////////////////////////////////////
//
/**@file CfgManager.h

   General Description
    This class provides some implmentations which could be
    different for different Cfg types, and some methods do require
    a derived Cfg provide its own version(pure virtual methods).
    This is an abstract class.

          CfgManager   <--------  Cfg
          /   |     \
        _/    |      \_
    Cfg_free      |         Cfg_free_tree<int>
          Cfg_fixed_PRR

  BE AWARE:
      It is ASSUMED that all position values are packed together
      in the first 'posDof' spots of a Cfg data. Other choices
    would require the corrrsponding derived class have its own
    version of most member functions listed here.

  @date 8/31/99
  @author Guang Song
*/

////////////////////////////////////////////////////////////////////////////////////////////
#ifndef CfgManager_h
#define CfgManager_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include standard headers
#include <math.h>
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "Vectors.h"
#include "OBPRM.h"

////////////////////////////////////////////////////////////////////////////////////////////
class Environment;
class Cfg;
class CollisionDetection;
class CDInfo;
class MultiBody;
////////////////////////////////////////////////////////////////////////////////////////////

class CfgManager {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  //===================================================================
  /**@name  Constructors and Destructor*/
  //===================================================================
  //@{
  CfgManager(int _dof, int _posDof) : dof(_dof), posDof(_posDof) {}
  ~CfgManager();
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name I/O */
  //@{
  
  /**Get Robot (Links) configuration.
    *This method get configurations of all links of Robot in environment 
    *and put them into vector<Vector6D>&.
    *
    *@param c This Cfg will be used to update configuration of Robot
    *in environment.
    *@see ConfigEnvironment
    */
  virtual void printLinkConfigurations(const Cfg &c, Environment *env, vector<Vector6D>&);

  ///Print out information for *VIZMO*
  virtual void print_preamble_to_file(Environment *env, FILE *_fp, int numofCfg);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  
  /**@name Access Methods*/
  //@{
  inline int GetDOF() { return dof; } ///< Return Degree of Freedom of this Cfg
  inline int PosDOF() { return posDof; }
  /// Get the center poistion of Robot. Abstract method.
  virtual Vector3D GetRobotCenterPosition(const Cfg & c) const = 0;

  ///Abstract methods
  virtual Cfg GetRandomCfg(double R, double rStep) = 0;
  virtual Cfg GetRandomCfg_CenterOfMass(double *boundingBox) = 0;
  virtual Cfg GetRandomRay(double incr) = 0;

  /// Get Position part of configuration according to posDof
  virtual vector<double>  GetPosition(const Cfg& c);
  /// Get orientation part of configuration according to posDof
  virtual vector<double>  GetOrientation(const Cfg& c);

  // methods for Distance Metric.
  ///Eulidian distance for (normalized) orientation from origin to this Cfg
  virtual double  OrientationMagnitude(const Cfg& c);

  ///Eulidian distance for position from origin to this Cfg
  virtual double  PositionMagnitude(const Cfg& c);

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    methods for nodes generation : Most of them are abstract
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Node Generation*/
  //@{

  ///Abstract Cfg Gerneration
  virtual bool GenerateOverlapCfg(Environment *env, int robot,
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg) = 0; // OBPRM
  virtual vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs, SID _cdsetid,
    CDInfo& _cdInfo) = 0; // NORMAL

  /**Create Cfg whose vertices are inside bounding box of env and without colliding with obstacles.
    *This function calls Cfg::GetRandomCfg
    */
  virtual Cfg GetFreeRandomCfg(Environment *env, CollisionDetection *cd, SID _cdsetid, CDInfo& _cdInfo);

  ///Create Cfg whose poisiton part is from c1 and orientation part is from c2
  virtual Cfg GetPositionOrientationFrom2Cfg(const Cfg& c1, const Cfg& c2);  // rotate-at-s helper

  /**Return 4 Cfgs, c1, s1, s2, and c2. Poisiton which is interpolated by parameter, s,
    *of s1 and s2 are the same. Orientation of s1 is the same as that of c1. 
    *Orientation of s2 is the same as that of c2.
    */
  virtual vector<Cfg> GetMovingSequenceNodes(const Cfg& c1, const Cfg& c2,double s); // for rotate-at-s Local Planner.
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    methods for nodes connection
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Node Connection*/
  //@{

  /**Return (free Cfg) neightbors aound a given Cfg.
    *Neighbors are defined by increment. 
    *One Neighbor is (c+increment). 
    *Two Neighbors are (c+ (poisiotn part of increment) ) and (c+ (orientation part of increment) )
    *dof number of Neighbors are (c+(each dimension of increment))
    *Then collision detection is checked for evey neighbor.
    *@param c start point of looking up
    *@param env for Collision Detection
    *@param increment used to define cfgs of neighbors
    *@param noNeighbors number of neighbors given by caller. if number of valid neighbor is less then 
    *no Neighbors then noNeighbors will be changed to number of valid neighbor that found by this method.
    *@param _cdsetid for Collision Detection
    *@param _cdInfo for Collision Detection
    */
  virtual vector<Cfg> FindNeighbors(const Cfg& c, Environment *env,
            const Cfg& increment,CollisionDetection *,
            int noNeighbors, SID  _cdsetid, CDInfo& _cdInfo);
  
  /**Return (free Cfg) neightbors aound a given Cfg with consideration of goal.
    *Neighbors are defined by increment. 
    *One Neighbor is (c+increment). 
    *Two Neighbors are (c+ (poisiotn part of increment) ) and (c+ (orientation part of increment) )
    *dof number of Neighbors are (c+(each dimension of increment))
    *Then collision detection is checked for evey neighbor.
    *@param c start point of looking up
    *@param goal goal
    *@param env for Collision Detection
    *@param increment used to define cfgs of neighbors
    *@param noNeighbors number of neighbors given by caller. if number of valid neighbor is less then 
    *no Neighbors then noNeighbors will be changed to number of valid neighbor that found by this method.
    *@param _cdsetid for Collision Detection
    *@param _cdInfo for Collision Detection
    *
    *@note the only difference between this method and the other one is this method called
    *IncrementTowardsGoal to generate neighbors.
    */
  virtual vector<Cfg> FindNeighbors(const Cfg& c, Environment *env, 
    const Cfg& goal, const Cfg& increment, 
    CollisionDetection *,int noNeighbors, SID  _cdsetid, CDInfo& _cdInfo);

  /**increament c by considering goal.
    *if the differences between c and goal are smaller than increment, 
    *then the increment will be the differences. Otherwise c will be 
    *increased by parameter, increment.
    */
  virtual void IncrementTowardsGoal(Cfg& c, const Cfg &goal, const Cfg &increment);

  /**First compute n_ticks by orientationRes and positionRes 
    *and then call FindIncrement(const Cfg&, const Cfg&, int).
    */
  virtual Cfg FindIncrement(const Cfg& c, const Cfg& _goal, int * n_ticks,
                            double positionRes, double orientationRes);

  ///Calculate increment by computing the difference between c and _goal and divide it by n_ticks.
  virtual Cfg FindIncrement(const Cfg& c, const Cfg& _goal, int  n_ticks);

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helper functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /*@name Helper functions*/
  //@{

  ///Move the robot in enviroment to the given configuration. Abstract.
  virtual bool ConfigEnvironment(const Cfg &c, Environment *env) = 0;

  /**If param indicates position, then return -10000~10000, otherwise,
    *param indeicates orientation, then return 0~1
    */
  virtual pair<double,double> SingleParamRange(int param);
  
  /**Map the orientation to [0,1)
    *if index is an orientation index, normalize it *only*. 
    *if index == -1, normalize all orientations. Otherwise do nothing.
    */
  virtual void Normalize_orientation(Cfg &c, int index);

  ///roughly check if configurations of these wo Cfg instance are the same.
  virtual bool AlmostEqual(const Cfg&c1, const Cfg& c2);

  ///Retrun true if the difference between c1 and c2 are smaller then positionRes and orientationRes.
  virtual bool isWithinResolution(const Cfg&c1, const Cfg&c2,
                            double positionRes, double orientationRes);

  ///Create a Cfg with Invalide data member value.
  virtual Cfg InvalidData();

  /**updating the environment(multibodies), then ask CollisionDetection to check collision 
    *@see CollisionDetection:IsInCollision
    */
  virtual bool isCollision(Cfg &c, Environment *env, CollisionDetection *cd,
                           int robot, int obs, SID _cdsetid, CDInfo& _cdInfo);

  /**updating the environment(multibodies), then ask CollisionDetection to check collision 
    *@see CollisionDetection:IsInCollision
    */
  virtual bool isCollision(Cfg &c, Environment *env, CollisionDetection *cd,
                           SID _cdsetid, CDInfo& _cdInfo);

  /**updating the environment(multibodies), then ask CollisionDetection to check collision 
    *@see CollisionDetection:IsInCollision
    */
  virtual bool isCollision(Cfg &c, Environment *env, CollisionDetection *cd,
                           SID _cdsetid, CDInfo& _cdInfo, MultiBody * onflyRobot);


  /// Return a configuration(conformation)'s potential. default value is 0.
  virtual double GetPotential(const Cfg &c, Environment *env) { return 0.0; } 
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  protected:
    // dof: Degree of Freedom, posDof: DOF for positions.
    int dof;    ///< Degree of Fredoom
    int posDof; ///< Degree of Fredoom for position part

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  private:

} ;

#endif

// $Id$

/**@file Cfg_free_serial.h
   A derived template class from CfgManager. It provides some
   specific implementation directly related to a multiple joints
   serial robot.
   @author Guang Song
   @date 08/31/99
*/
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_free_serial_h
#define Cfg_free_serial_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "CfgManager.h"

class Cfg_free_serial : public CfgManager {
public:


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  
  //===================================================================
  /**@name  Constructors and Destructor*/
  //===================================================================
  //@{
  ///Degree of freedom is (6+_numofJoints) and degree of freedom for position part is 3.
  Cfg_free_serial(int _numofJoints);
  ///Do nothing
  ~Cfg_free_serial();
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods*/
  //@{

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition(const Cfg & c) const;

  /**Randomly generate a Cfg.
    *@param R This new Cfg will have distance (position of its base link) R from origin
    *@param rStep
	*@return the first 6 components in Cgf is the configuration for base link.
	*and the other (NumofJoints) components are angles between joints
	*@todo what is rStep?
	*/
  virtual Cfg GetRandomCfg(double R, double rStep);

  /** Randomly generate a Cfg whose center positon of base link is inside a 
    * given bounding box.(rotation, don't care!)
    * @todo this is not EXACTLY accurate, ok with most cases ... TO DO
    * To be accurate, one has to make sure every link is inside the given BB,
    * but here only the base link is taken care of. It is almost fine since
    * a little 'bigger' BB will contain all links.
	*/
  virtual Cfg GetRandomCfg_CenterOfMass(double *boundingBox);

  ///Get a random vector whose magnitude is incr (ie. the orienatation of of this Cfg is 0)
  virtual Cfg GetRandomRay(double incr);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	methods for nodes generation : Most of them are abstract
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Node Generation*/
  //@{

  /** Node Generation methods: OBPRM
    * Generate a new Cfg, and put it in resultCfg.
	* The position of new cfg is from (robot_goal-robot_start)
	* The orientation (including joint angles) of new cfg is generated randomly.
    */
  virtual bool GenerateOverlapCfg(Environment *env, int robot,
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg);

  /** Node Generation methods: NORMAL
    *generate nodes by overlapping two triangles' normal.
    *@todo Document this
    */
  virtual vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs,
    SID _cdsetid, CDInfo& _cdInfo);

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Helper functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /*@name Helper functions*/
  //@{
	  /** methods for Cfg generation and collision checking.
	    *@see GetWorldTransformation
	    */
	  virtual bool ConfigEnvironment(const Cfg &c, Environment *env);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  protected:
     int NumofJoints;	///<Number of Joint

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	private Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  private:

};

#endif

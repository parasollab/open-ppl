// $Id$

/**@file Cfg_fixed_PRR.h
  *A derived class from CfgManager. It provides some specific
  *implementation closely related to a PRR robot.
  *Degree of Freedom: 3
  *@author Guang Song
  *@date 08/31/99
  */

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_fixed_PRR_h
#define Cfg_fixed_PRR_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "CfgManager.h"

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from CfgManager. It provides some specific
  *implementation closely related to a PRR robot.
  *Degree of Freedom: 3
  */
class Cfg_fixed_PRR : public CfgManager {
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

  /**Creae an instance of Cfg_fixed_PRR.
    *Degree of freedom is 3 and Degree of freedom for position part is 1.
    */
  Cfg_fixed_PRR();
  ///Do nothing
  ~Cfg_fixed_PRR();

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

  ///The center position is (0, 0, c[0])
  virtual Vector3D GetRobotCenterPosition(const Cfg & c) const;
  
  /**Randomly generate a Cfg
    *the new Cfg whose poisiotn will be R or -R, and its orientations 
    *are generated randomly between [0,1)
    *@param rStep
    *@todo what is rStep?
    */
  virtual Cfg GetRandomCfg(double R, double rStep);

  /**Randomly generate a Cfg whose center positon (z only) is inside a given bounding box.
    *(rotation, don't care!)
    */
  virtual Cfg GetRandomCfg_CenterOfMass(Environment *env);

  ///Get a random vector whose magnitude is incr.
  virtual Cfg GetRandomRay(double incr);
  //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    methods for nodes generation 
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  
  /**@name Node Generation*/
  //@{

  /** Node Generation methods: OBPRM
    *@todo Document this
    */
  virtual bool GenerateOverlapCfg(Environment *env, int robot,
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg);

  /** Node Generation methods: NORMAL
    * Not implemented yet.
    */
  virtual vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs,
    SID _cdsetid, CDInfo& _cdInfo);
 
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
  /// methods for Cfg generation and collision checking.
  virtual bool ConfigEnvironment(const Cfg &c, Environment *env);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    private Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  private:



} ;
#endif

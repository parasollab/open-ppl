// $Id$

/**@file Cfg_fixed_PRR.h
  *A derived class from Cfg. It provides some specific
  *implementation closely related to a PRR robot.
  *Degree of Freedom: 3
  *@author Guang Song
  *@date 08/31/99
  *
  * Last Modified
  */

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_fixed_PRR_h
#define Cfg_fixed_PRR_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg.h"

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from Cfg. It provides some specific
  *implementation closely related to a PRR robot.
  *Degree of Freedom: 3
  */
class Cfg_fixed_PRR : public Cfg {
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
  Cfg_fixed_PRR(double zz, double ceta1, double ceta2);
  Cfg_fixed_PRR(double x, double y, double z, double roll, double pitch, double yaw);
  Cfg_fixed_PRR(const Vector3<double>& _v);
  Cfg_fixed_PRR(const Cfg& c);

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

  virtual void equals(const Cfg&);
  
  ///The center position is (0, 0, c[0])
  virtual Vector3D GetRobotCenterPosition() const;
  
  virtual const char* GetName() const;
  
  /**Randomly generate a Cfg
    *the new Cfg whose poisiotn will be R or -R, and its orientations 
    *are generated randomly between [0,1)
    *@param rStep
    *@todo what is rStep?
    */
  virtual void GetRandomCfg(double R, double rStep);

  ///Get a random vector whose magnitude is incr.
  virtual void GetRandomRay(double incr);
  //@}

  virtual void GetRandomCfg(Environment* env);
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
  virtual void GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs,
    CDInfo& _cdInfo, vector<Cfg*>&) const;
 
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
  virtual bool ConfigEnvironment(Environment *env) const;
  //@}

  virtual Cfg* CreateNewCfg() const;
  virtual Cfg* CreateNewCfg(vector<double>&) const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  protected:
  /**Randomly generate a Cfg whose center positon (z only) is inside a given bounding box.
    *(rotation, don't care!)
    */
  virtual void GetRandomCfg_CenterOfMass(Environment *env);
	    

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

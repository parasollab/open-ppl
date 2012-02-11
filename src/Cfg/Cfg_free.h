// $Id$

/**@file Cfg_free.h
  *A derived class from CfgManager. It provides some specific
  *implementation for a 6-dof rigid-body moving in a 3-D work space.
  *
  *@date 08/31/99
  *@author Guang Song
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_free_h
#define Cfg_free_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg.h"

////////////////////////////////////////////////////////////////////////////////////////////
class GMSPolyhedron;

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from CfgManager. It provides some specific
  *implementation for a 6-dof rigid-body moving in a 3-D work space.
  */
class Cfg_free : public Cfg {
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
  ///Degree of freedom is 6 and Degree of freedom for position part is 3.
  Cfg_free();
  Cfg_free(double x, double y, double z, double roll, double pitch, double yaw);
  Cfg_free(const Vector6D& _v);
  Cfg_free(const Cfg& c);
  ///Do nothing
  virtual ~Cfg_free();
  //@}
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg::define_type(t);
    }
#endif

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods*/
  //@{

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition() const;

  virtual const char* GetName() const;

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(Environment*) const;

  /**Randomly generate a Cfg
    *@param R This new Cfg will have distance (position) R from origin
    *@param rStep
    *@todo what is rStep?
    */
  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment* _env);
  virtual void GetRandomCfg(Environment *_env,shared_ptr<Boundary> _bb);

  ///Get a random vector whose magnitude is incr (note. the orienatation of of this Cfg is 0)
  virtual void GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm, bool _norm=true);
  //@}
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

 protected:
  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual void GetRandomCfg_CenterOfMass(Environment* _env);
  virtual void GetRandomCfg_CenterOfMass(Environment* _env,shared_ptr<Boundary> _bb);
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    private Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  private:

};

#endif

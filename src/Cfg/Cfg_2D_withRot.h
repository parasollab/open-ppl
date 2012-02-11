// $Id: Cfg_2D_withRot.h 2836 2011-06-02 15:58:23Z sthomas $

/**@file Cfg_2D_withRot.h
  *A derived class from CfgManager. It provides some specific
  *implementation for a 6-dof rigid-body moving in a 3-D work space.
  *
  *@date 08/31/99
  *@author Guang Song
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_2D_withRot_h
#define Cfg_2D_withRot_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg_2D.h"

////////////////////////////////////////////////////////////////////////////////////////////
class GMSPolyhedron;

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from CfgManager. It provides some specific
  *implementation for a 6-dof rigid-body moving in a 3-D work space.
  */
class Cfg_2D_withRot : public Cfg_2D {
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
  Cfg_2D_withRot();
  Cfg_2D_withRot(double x, double y, double theta);
  Cfg_2D_withRot(const Vector3d& _v);
  Cfg_2D_withRot(const Cfg& c);
  Cfg_2D_withRot(const Point2d _p, double theta);
  
  ///Do nothing
  virtual ~Cfg_2D_withRot();
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
  ///Write configuration to output stream
  virtual void Write(ostream& os) const;
  ///Read configuration from input stream
  virtual void Read(istream& is);
 
  double GetRot() const {return m_v[2];}
  void SetRot(double d) {m_v[2]=d;}

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

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
  Cfg_free(const Cfg& c);
  ///Do nothing
  virtual ~Cfg_free();
  //@}
 
  virtual vector<Robot> GetRobots(int _num);

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

  virtual const string GetName() const;

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(Environment*) const;
  
  ///Get a random vector whose magnitude is incr (note. the orienatation of of this Cfg is 0)
  template<class DistanceMetricPointer>
  void GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm=true);
  //@}
  
  virtual Cfg* CreateNewCfg() const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

 protected:
  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual void GetRandomCfgCenterOfMass(Environment* _env,shared_ptr<Boundary> _bb);
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    private Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  private:

};

template<class DistanceMetricPointer>
void
Cfg_free::GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm) {
  //randomly sample params
  m_v.clear();
  for(size_t i=0; i<m_dof; ++i)
    m_v.push_back(2.0*DRand()-1.0);

  //scale to appropriate length
  Cfg_free origin;
  _dm->ScaleCfg(_env, _incr, origin, *this, _norm);
  if(_norm)
    NormalizeOrientation();
}

#ifdef _PARALLEL
namespace stapl {
template <typename Accessor>
class proxy<Cfg_free, Accessor> 
: public Accessor {
private:
  friend class proxy_core_access;
  typedef Cfg_free target_t;
  
public:
  explicit proxy(Accessor const& acc) : Accessor(acc) { }
  operator target_t() const { return Accessor::read(); }
  proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
  proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
}; //struct proxy
}
#endif

#endif

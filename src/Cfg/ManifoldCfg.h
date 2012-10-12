/////////////////////////////////////////////////////////////////////
//
//  ManifoldCfg.cpp
//
//  General Description
//	A derived class from Cfg. It provides for a generalized 
//      d-dimension configuration space and any robot system
//	should be representable within it.
//
/////////////////////////////////////////////////////////////////////

#ifndef MANIFOLDCFG_H_
#define MANIFOLDCFG_H_

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg.h"

////////////////////////////////////////////////////////////////////////////////////////////
/**
 *A derived class from CfgManager. It provides some specific
 *implementation for a 6-dof rigid-body moving in a 3-D work space.
 */
class ManifoldCfg : public Cfg {
  public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ManifoldCfg();
    ManifoldCfg(const Cfg& _c);
    virtual ~ManifoldCfg();

#ifdef _PARALLEL
    void 
      define_type(stapl::typer& _t) {
        Cfg::define_type(_t);
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
    virtual vector<Robot> GetRobots(int) {return vector<Robot>();};

    ///Move the (the first link of)  robot in enviroment to the given configuration.
    virtual bool ConfigEnvironment(Environment*) const;

    ///Get a random vector whose magnitude is incr (note. the orienatation of this Cfg is 0)
    virtual void GetRandomRay(double _incr, Environment* _env, shared_ptr<DistanceMetricMethod> _dm, bool _norm = true);
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
    virtual void GetRandomCfgCenterOfMass(Environment* _env, shared_ptr<Boundary> _bb);

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

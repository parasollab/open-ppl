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

#include "Cfg.h"

class ManifoldCfg : public Cfg {
  public:
    ManifoldCfg();
    ManifoldCfg(const Cfg& _c);
    virtual ~ManifoldCfg();

#ifdef _PARALLEL
    void 
      define_type(stapl::typer& _t) {
        Cfg::define_type(_t);
      }
#endif

    ///The center position is get from param, c, configuration. (The position part of c)
    virtual Vector3D GetRobotCenterPosition() const;

    virtual Vector3D GetRobotCenterofMass(Environment*) const;
    virtual const string GetName() const;
    virtual vector<Robot> GetRobots(int) {return vector<Robot>();};

    ///Move the (the first link of)  robot in enviroment to the given configuration.
    virtual bool ConfigEnvironment(Environment*) const;

    ///Get a random vector whose magnitude is incr (note. the orienatation of this Cfg is 0)
    template<class DistanceMetricPointer>
    void GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm = true);

    virtual Cfg* CreateNewCfg() const;

  protected:
    ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
    virtual void GetRandomCfgCenterOfMass(Environment* _env, shared_ptr<Boundary> _bb);
};

///Get a random vector whose magnitude is incr (note. the orienatation of this Cfg is 0)
template<class DistanceMetricPointer>
void
ManifoldCfg::GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm) {
  //randomly sample params
  m_v.clear();
  for(size_t i = 0; i < DOF(); ++i)
    m_v.push_back(2.0*DRand() - 1.0);

  //scale to appropriate length
  ManifoldCfg origin;
  _dm->ScaleCfg(_env, _incr, origin, *this);
  if(_norm)
    NormalizeOrientation();
}

#endif

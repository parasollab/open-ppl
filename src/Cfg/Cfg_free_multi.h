#ifndef Cfg_free_multi_h
#define Cfg_free_multi_h

#include "Cfg_free.h"

class Cfg_free_multi : public Cfg_free {
public:
  //class for multiple rigid body robots
  //v: translational dofs for all robots then rotational dofs for all robots

  //===================================================================
  // Constructors and Destructor
  //===================================================================
  Cfg_free_multi();
  Cfg_free_multi(double x, double y, double z, double roll, double pitch, double yaw);
  Cfg_free_multi(const Vector6D& _v);
  Cfg_free_multi(const vector<double>& _v);
  Cfg_free_multi(const Cfg& c);
  virtual ~Cfg_free_multi();
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg_free::define_type(t);
      
    }
  #endif

  static int getNumofRobots() { return NumofRobots; }
  static void setNumofRobots(int robots) { NumofRobots = robots; }

  virtual const char* GetName() const;

  virtual Vector3D GetRobotCenterPosition() const;

  virtual bool ConfigEnvironment(Environment*) const;

  virtual void GetRandomCfg(double _r, double _rStep);
  virtual void GetRandomCfg(Environment* _env);
  virtual void GetRandomCfg(Environment *_env, shared_ptr<Boundary> _bb);

  virtual void GetRandomCfg_CenterOfMass(Environment* _env);
  virtual void GetRandomCfg_CenterOfMass(Environment* _env, shared_ptr<Boundary> _bb);

 protected:
  static int NumofRobots;
  
 private:
};

#endif

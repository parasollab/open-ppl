// $Id$
/**@file Cfg_2D.h
  *A derived class from Cfg. It provides some specific
  *implementation for a 2-dof rigid-body moving in a 2-D work space.
  *
  *@date 6/6/11
  *@author Jory Denny
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_2D_h
#define Cfg_2D_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg.h"
#include "Point.h"
using namespace mathtool;

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from Cfg_free. It provides some specific
  *implementation for a 3-dof rigid-body moving in a 2-D work space.
  */
class Cfg_2D : public Cfg {
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
  Cfg_2D();
  Cfg_2D(const Cfg&c);
  Cfg_2D(const Vector2d& _v);
  Cfg_2D(double, double);
  Cfg_2D(const Point2d _p);

  ///Do nothing
  virtual ~Cfg_2D();
  //@}
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg_free::define_type(t);
      
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

  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment *env);
  
  virtual void GetRandomRay(double incr, Environment* env, shared_ptr< DistanceMetricMethod> dm);
  
  virtual void add(const Cfg&, const Cfg&);
  virtual void subtract(const Cfg&, const Cfg&);
  virtual void negative(const Cfg&);
  virtual void multiply(const Cfg&, double);
  virtual void divide(const Cfg&, double);
   
  virtual Cfg& operator=(const Cfg&);

  virtual void WeightedSum(const Cfg&, const Cfg&, double weight = 0.5);       
  
  /** Set a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to set.
   * @param value the value to set the parameter as
   */
  virtual int SetSingleParam(int param, double value);    
  
  /** Increment a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to set.
   * @param value the value to increment the parameter by.
   */
  virtual int IncSingleParam(int param, double value);  
 
  ///Increase every value in this instance in each dimention by the value in _increment
  virtual void Increment(const Cfg& _increment);
  virtual void IncrementTowardsGoal(const Cfg &goal, const Cfg &increment);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* n_ticks, 
           double positionRes, double orientationRes);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int n_ticks);

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition() const;

  virtual const char* GetName() const;

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(Environment*) const;

  //Get position in the form of Point2d
  Point2d getPos() const {return p;}
  void setPos(Point2d _p){ m_v[0]=_p[0]; m_v[1]=_p[1]; p=_p;}
 
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
 protected:
  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual void GetRandomCfg_CenterOfMass(Environment* env);
  
 private:
  Point2d p;

};

#endif

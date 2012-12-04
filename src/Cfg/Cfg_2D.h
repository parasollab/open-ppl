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
  Cfg_2D(const Point2d _p);

  ///Do nothing
  virtual ~Cfg_2D();
  //@}
  
  virtual vector<Robot> GetRobots(int _numJoints);
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &_t)  
    {
      Cfg::define_type(_t);
      _t.member(m_point);
      
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

  template<class DistanceMetricPointer>
  void GetRandomRay(double incr, Environment* env, DistanceMetricPointer dm, bool norm=true);

  virtual void add(const Cfg&, const Cfg&);
  virtual void subtract(const Cfg&, const Cfg&);
  virtual void negative(const Cfg&);
  virtual void multiply(const Cfg&, double, bool _norm=true);
  virtual void divide(const Cfg&, double);
   
  virtual Cfg& operator=(const Cfg&);

  virtual void WeightedSum(const Cfg&, const Cfg&, double weight = 0.5);       
  
  /** Set a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to set.
   * @param value the value to set the parameter as
   * Note: norm is a dummy value in 2d
   */
  virtual int SetSingleParam(size_t param, double value, bool norm=true);    
  
  /** Increment a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to set.
   * @param value the value to increment the parameter by.
   */
  virtual int IncSingleParam(size_t param, double value);  
 
  ///Increase every value in this instance in each dimention by the value in _increment
  virtual void Increment(const Cfg& _increment);
  virtual void IncrementTowardsGoal(const Cfg &goal, const Cfg &increment);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* n_ticks, 
           double positionRes, double orientationRes);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int n_ticks);

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition() const;
  virtual Vector3D GetRobotCenterofMass(Environment*) const;

  virtual const string GetName() const;

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(Environment*) const;

  //Get position in the form of Point2d
  Point2d getPos() const {return m_point;}
  void setPos(Point2d _p){ m_v[0]=_p[0]; m_v[1]=_p[1]; m_point=_p;}
 
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

 private:
  Point2d m_point;
};

template<class DistanceMetricPointer>
void 
Cfg_2D::GetRandomRay(double incr, Environment* env, DistanceMetricPointer dm, bool _norm) {
  //randomly sample params
  double dist=0.0;
  m_v.clear();
  for(size_t i=0; i<m_dof; ++i) {
    m_v.push_back( double(2.0)*DRand() - double(1.0) );
    dist += pow(m_v[i],2);
  }

  //scale to appropriate length
  Cfg_2D origin;
  dm->ScaleCfg(env, incr, origin, *this, _norm);
  setPos(Point2d(m_v[0], m_v[1]));
}

#endif

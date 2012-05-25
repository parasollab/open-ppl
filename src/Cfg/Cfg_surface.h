// $Id: Cfg_surface.h 

/**@file Cfg_surface.h
  *A derived class from Cfg
  *implementation for a 3-dof rigidbody moving in a 3-D work space 
  *but restricted to movement either on the default surface or or
  *valid surfaces
  *
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_surface_h
#define Cfg_surface_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg.h"
#include "Point.h"
using namespace mathtool;

////////////////////////////////////////////////////////////////////////////////////////////
class GMSPolyhedron;

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from Cfg. It provides some specific
  *implementation for a 3-dof rigid-body moving ON a 3-D work space.
  *positional compontents are xz and height component is y.
  *Will be used in conjunction with group behaviors code.
  */
class Cfg_surface : public Cfg {
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
  Cfg_surface();
  Cfg_surface(double _x, double _y, double _H, int _sid);
  Cfg_surface(const Vector3d& _v);
  Cfg_surface(const Cfg& _c);
  Cfg_surface(const Point2d _p, double _h, int _sid);
  
  ///Do nothing
  virtual ~Cfg_surface();
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
  virtual void Write(ostream& _os) const;
  ///Read configuration from input stream
  virtual void Read(istream& _is);
  
  /**Randomly generate a Cfg
    *@param R This new Cfg will have distance (position) R from origin
    *@param rStep *@todo what is rStep?
    */
  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment* _env,shared_ptr<Boundary> _bb);
  virtual void GetRandomCfg(Environment* _env);

  ///Get a random vector whose magnitude is incr (note. the orienatation of of this Cfg is 0)
  virtual void GetRandomRay(double _incr, Environment* _env, shared_ptr<DistanceMetricMethod> _dm, bool _norm=true);
  //void GetRandomRayPos(double _incr, Environment* _env);
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  // operations
  virtual void add(const Cfg&, const Cfg&);
  virtual void subtract(const Cfg&, const Cfg&);
  virtual void negative(const Cfg&);
  virtual void multiply(const Cfg&, double, bool _norm=true);
  virtual void divide(const Cfg&, double);

  Cfg_surface& operator=(const Cfg_surface& _c);
  virtual void WeightedSum(const Cfg&, const Cfg&, double _weight = 0.5);       
  ///////////////////////////////////////////////////////////////////////////////////////////
  
  /** Set a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to set.
   * @param value the value to set the parameter as
   * Note: norm is a dummy value in 2d
   */
  virtual int SetSingleParam(size_t _param, double _value, bool _norm=true);
  
  /** Increment a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to set.
   * @param value the value to increment the parameter by.
   */
  virtual int IncSingleParam(size_t _param, double _value);  
 
  ///Increase every value in this instance in each dimention by the value in _increment
  virtual void Increment(const Cfg& _increment);
  virtual void IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, 
           double _positionRes, double _orientationRes);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks);

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition() const;

  virtual const string GetName() const;

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(Environment*) const;


  //Get position in the form of Point2d
  Point2d getPos() const {return m_p;}
  void setPos(Point2d _p){ m_v[0]=_p[0]; m_v[2]=_p[1]; m_p=_p;}

  //Get position in the form of Point2d
  double getHeight() const {return m_H;}
  void setHeight(double _h){ m_v[1]=_h; m_H=_h;}
  int getSurfaceID() const { return m_SurfaceID; }
  void setSurfaceID(int sid) { m_SurfaceID = sid; }
  
  //get rotation - doesn't make sense here yet 
  double GetRot() const {return 0;} //there is no rot here
  void SetRot(double d) { cout <<"Setting rot in wrong cfg type!"<<endl; }
  
  //@}
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  
  virtual Cfg* CreateNewCfg() const;
  virtual Cfg* CreateNewCfg(vector<double>&) const;

 protected:
  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual void GetRandomCfgCenterOfMass(Environment* env);
  virtual void GetRandomCfgCenterOfMass(Environment* env,shared_ptr<Boundary> bb);
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    private Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
 private:

  Point2d m_p; //positional component
  double m_H; // height component to complement position
  int    m_SurfaceID; //surface id that this cfg is associated with 
};

#endif

// $Id: CfgSurface.h 

/**@file CfgSurface.h
  *A derived class from Cfg
  *implementation for a 3-dof rigidbody moving in a 3-D work space 
  *but restricted to movement either on the default surface or or
  *valid surfaces
  *
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CfgSurface_h
#define CfgSurface_h

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
class CfgSurface : public Cfg {
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
  CfgSurface();
  CfgSurface(double _x, double _y, double _h, int _sid);
  CfgSurface(const Vector3d& _v);
  CfgSurface(const Cfg& _c);
  CfgSurface(const Point2d _p, double _h, int _sid);
  
  ///Do nothing
  virtual ~CfgSurface();
  
  virtual vector<Robot> GetRobots(int){return vector<Robot>();}

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
  
  virtual void GetRandomCfg(Environment* _env,shared_ptr<Boundary> _bb);
  virtual void GetRandomCfg(Environment* _env);

  ///Get a random vector whose magnitude is incr (note. the orienatation of of this Cfg is 0)
  template<class DistanceMetricPointer>
  void GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm=true);
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  // operations
  virtual void add(const Cfg&, const Cfg&);
  virtual void subtract(const Cfg&, const Cfg&);
  virtual void negative(const Cfg&);
  virtual void multiply(const Cfg&, double, bool _norm=true);
  virtual void divide(const Cfg&, double);

  CfgSurface& operator=(const CfgSurface& _c);
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
  Point2d GetPos() const {return m_p;}
  void SetPos(Point2d _p){ m_v[0]=_p[0]; m_v[2]=_p[1]; m_p=_p;}

  //Get position in the form of Point2d
  double GetHeight() const {return m_h;}
  void SetHeight(double _h){ m_v[1]=_h; m_h=_h;}
  int GetSurfaceID() const { return m_surfaceID; }
  void SetSurfaceID(int sid) { m_surfaceID = sid; }
  
  //@}
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  
  virtual Cfg* CreateNewCfg() const;

 protected:
  ///Randomly generate a Cfg whose center positon is inside a given bounding box.
  virtual Vector3D GetRobotCenterofMass(Environment*) const;
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
  double m_h; // height component to complement position
  int    m_surfaceID; //surface id that this cfg is associated with 

  //helper function to ensure data integrity (dofs match m_p and m_h)
  void UpdatePrivateVariables();
};

template<class DistanceMetricPointer>
void
CfgSurface::GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm) {
  //randomly sample params
  m_v.clear();
  Vector2d v( DRand(), DRand() );
  v = v.normalize();
  m_v.push_back( v[0] ); //for now just create a ray in the plane (not so great for terrain)
  m_v.push_back( 0.0  );
  m_v.push_back( v[1] );

  SetPos(Point2d(m_v[0], m_v[2]));
  SetHeight( m_v[1] );
  //how to handle surface id?
}

#endif

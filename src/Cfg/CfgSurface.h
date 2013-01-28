// $Id: CfgSurface.h 

/**@file CfgSurface.h
 *A derived class from Cfg
 *implementation for a 3-dof rigidbody moving in a 3-D work space 
 *but restricted to movement either on the default surface or or
 *valid surfaces
 *
 */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CFGSURFACE_H_
#define CFGSURFACE_H_

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

    CfgSurface();
    CfgSurface(double _x, double _y, double _h, int _sid);
    CfgSurface(const Vector3d& _v);
    CfgSurface(const CfgSurface& _c);
    CfgSurface(const Point2d& _p, double _h, int _sid);
    ///Do nothing
    virtual ~CfgSurface();

    CfgSurface& operator=(const CfgSurface& _cfg);
    ///determines equality of this and other configuration
    bool operator== (const CfgSurface& _cfg) const;
    ///determines non-equality of this and other configuration
    bool operator!= (const CfgSurface& _cfg) const;
    //addition
    CfgSurface operator+(const CfgSurface& _cfg) const;
    CfgSurface& operator+=(const CfgSurface& _cfg);
    //subtraction
    CfgSurface operator-(const CfgSurface& _cfg) const;
    CfgSurface& operator-=(const CfgSurface& _cfg);
    //negate
    CfgSurface operator-() const;
    //scalar multiply
    CfgSurface operator*(double _d) const;
    CfgSurface& operator*=(double _d);
    //scalar divide
    CfgSurface operator/(double _d) const;
    CfgSurface& operator/=(double _d);
    //access dof values
    double& operator[](size_t _dof);
    const double& operator[](size_t _dof) const;
    //I/O
    friend ostream& operator<< (ostream&, const CfgSurface& _cfg);
    friend istream& operator>> (istream&, CfgSurface& _cfg);


    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Access Methods : Retrive and set related information of this class
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    virtual const string GetName() const {return "CfgSurface";}

    ///Get internal storage of configuration
    vector<double> GetData() const;
    void SetData(const vector<double>& _data);

    //Get position in the form of Point2d
    const Point2d& GetPos() const {return m_pt;}
    void SetPos(const Point2d& _p){m_pt = _p;}

    //Get position in the form of Point2d
    double GetHeight() const {return m_h;}
    void SetHeight(double _h) {m_h = _h;}
    int GetSurfaceID() const {return m_surfaceID;}
    void SetSurfaceID(int _sid) {m_surfaceID = _sid;}

    /// methods for Distance Metric.
    virtual vector<double> GetPosition() const;
    virtual double Magnitude() const;
    virtual double PositionMagnitude() const;

    ///The center position is get from param, c, configuration. (The position part of c)
    virtual Vector3D GetRobotCenterPosition() const;
    virtual Vector3D GetRobotCenterofMass(Environment*) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Generation Related Methods : These methods are related to create Cfgs randomly
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Get a random vector whose magnitude is incr (note. the orienatation of of this Cfg is 0)
    template<class DistanceMetricPointer>
      void GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm=true);

    ///Move the (the first link of)  robot in enviroment to the given configuration.
    virtual bool ConfigEnvironment(Environment*) const;

    void GetResolutionCfg(Environment*);

    ///Increase every value in this instance in each dimention by the value in _increment
    virtual void IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks);

    virtual void WeightedSum(const Cfg&, const Cfg&, double _weight = 0.5);       

    virtual void GetPositionOrientationFrom2Cfg(const Cfg&, const Cfg&);
    
  protected:
    ///Randomly generate a Cfg whose center positon is inside a given bounding box.
    virtual void GetRandomCfgImpl(Environment* env,shared_ptr<Boundary> bb);
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    private Data member and member methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
  private:
    Point2d m_pt;
    double m_h;
    int m_surfaceID; //surface id that this cfg is associated with 

  public:
#ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg::define_type(t);
    }
#endif
};

template<class DistanceMetricPointer>
void
CfgSurface::GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm) {
  //randomly sample params
  m_v.clear();
  Vector2d v(DRand(), DRand());
  v = v.normalize();
  m_pt[0] = v[0]; //for now just create a ray in the plane (not so great for terrain)
  m_pt[2] = v[1];
  m_h = 0.0; 
  //how to handle surface id?
  m_witnessCfg.reset();
}

#endif

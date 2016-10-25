#ifndef CFG_SURFACE_H_
#define CFG_SURFACE_H_

#include "Cfg.h"

#include <Vector.h>
using namespace mathtool;

#define INVALID_SURFACE -999
#define BASE_SURFACE -1

class GMSPolyhedron;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup SurfaceCfgs
/// @brief Configurations restricted to be on a surface
///
/// A derived class from Cfg. It provides some specific implementation for a
/// 3-@dof rigid-body moving ON a \f$3D\f$ work space restricted on either the
/// default surface or other valid surfaces. Positional compontents are \f$x\f$
/// and \f$z\f$ and the height component is \f$y\f$. Will be used in conjunction
/// with group behaviors code.
////////////////////////////////////////////////////////////////////////////////
class CfgSurface : public Cfg {
  public:

    CfgSurface();
    CfgSurface(double _x, double _y, double _h, int _sid);
    CfgSurface(const Vector3d& _v);
    CfgSurface(const CfgSurface& _c);
    CfgSurface(const Point2d& _p, double _h, int _sid);
    CfgSurface(const Cfg& _c);

    ///Do nothing destructor
    virtual ~CfgSurface();

    //Init CfgSurface
    void InitCfgSurface();

    //assignment operator
    CfgSurface& operator=(const CfgSurface& _cfg);

    ///determines equality of this and other configuration
    bool operator==(const CfgSurface& _cfg) const;

    ///determines non-equality of this and other configuration
    bool operator!=(const CfgSurface& _cfg) const;

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
    virtual double& operator[](size_t _dof);
    virtual const double& operator[](size_t _dof) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Access Methods : Retrive and set related information of this class
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    virtual const string GetName() const {return "CfgSurface";}

    ///Set internal storage of configuration
    virtual void SetData(const vector<double>& _data);

    void SetDataFromThis();

    //Get position in the form of Point2d
    const Point2d& GetPos() const {return m_pt;}
    void SetPos(const Point2d& _p){m_pt = _p; SetDataFromThis(); }

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
    virtual Vector3d GetRobotCenterPosition() const;
    virtual Vector3d GetRobotCenterofMass() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Generation Related Methods : These methods are related to create Cfgs randomly
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Get a random vector whose magnitude is incr (note. the orienatation of of this Cfg is 0)
    template<class DistanceMetricPointer>
      void GetRandomRay(double _incr, DistanceMetricPointer _dm, bool _norm=true);

    ///Move the (the first link of)  robot in enviroment to the given configuration.
    virtual void ConfigureRobot() const;

    void GetResolutionCfg(Environment*);

    ///Increase every value in this instance in each dimention by the value in _increment
    virtual void IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks);

    virtual void WeightedSum(const Cfg&, const Cfg&, double _weight = 0.5);

    virtual void GetPositionOrientationFrom2Cfg(const Cfg&, const Cfg&);

    //I/O
    virtual void Read(istream& _is);
    virtual void Write(ostream& _os) const;

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
  protected:
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

//IO operators for Cfg
ostream& operator<< (ostream& _os, const Cfg& _cfg);
istream& operator>> (istream& _is, Cfg& _cfg);

template<class DistanceMetricPointer>
void
CfgSurface::
GetRandomRay(double _incr, DistanceMetricPointer _dm, bool _norm) {
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

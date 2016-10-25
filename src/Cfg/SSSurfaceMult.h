#ifndef SSSURFACEMULT_H_
#define SSSURFACEMULT_H_

#ifdef PMPSSSurfaceMult
#include "Cfg/SSSurface.h"
#include "ValidityCheckers/SurfaceValidity.h"
#include "ValidityCheckers/CollisionDetectionValidity.h"
#include "DistanceMetrics/EuclideanDistance.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Weight.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
struct EmptyTraits{
  typedef SSSurface CfgType;
  typedef DefaultWeight<SSSurface> WeightType;
  typedef SSSurface& CfgRef;

  typedef MPProblem<EmptyTraits> MPProblemType;

  typedef boost::mpl::list<EuclideanDistance<EmptyTraits> > DistanceMetricMethodList;

  typedef boost::mpl::list<
  CollisionDetectionValidity<EmptyTraits>,
  SurfaceValidity<EmptyTraits>
  > ValidityCheckerMethodList;

  typedef boost::mpl::list<> NeighborhoodFinderMethodList;

  typedef boost::mpl::list<> SamplerMethodList;

  typedef boost::mpl::list<> LocalPlannerMethodList;

  typedef boost::mpl::list<> ExtenderMethodList;

  typedef boost::mpl::list<> PathModifierMethodList;

  typedef boost::mpl::list<> ConnectorMethodList;

  typedef boost::mpl::list<> MetricMethodList;

  typedef boost::mpl::list<> MapEvaluatorMethodList;

  typedef boost::mpl::list<> MPStrategyMethodList;
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup SurfaceCfgs
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class SSSurfaceMult : public Cfg{
  public:
  typedef EmptyTraits TraitsType;
  typedef SSSurface CompositeCfgType;
  typedef MPProblem<EmptyTraits> SurfaceMPProblemType;

    SSSurfaceMult();
    SSSurfaceMult(const SSSurfaceMult& _rhs);
    SSSurfaceMult(const Cfg& _rhs){}

    template<class DistanceMetricPointer>
      void GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm=true);

    //prepare multibodies for collision checking
    virtual bool ConfigureRobot(Environment* _env) const;

    void SetNumCfgs(size_t _num);
    size_t GetNumCfgs(){return m_cfgs.size();}

    vector<SSSurface>& GetCfgs(){return m_cfgs;}

    void SetCfgs(vector<SSSurface> _cfgs){m_cfgs = _cfgs;}

    SSSurfaceMult& operator=(const SSSurfaceMult& _rhs);

    ///determines equality of this and other configuration
    bool operator==(const SSSurfaceMult& _cfg) const;

    ///determines non-equality of this and other configuration
    bool operator!=(const SSSurfaceMult& _cfg) const;

    //addition
    SSSurfaceMult operator+(const SSSurfaceMult& _cfg) const;
    SSSurfaceMult& operator+=(const SSSurfaceMult& _cfg);

    //subtraction
    SSSurfaceMult operator-(const SSSurfaceMult& _cfg) const;
    SSSurfaceMult& operator-=(const SSSurfaceMult& _cfg);

    //negate
    SSSurfaceMult operator-() const;

    //scalar multiply
    SSSurfaceMult operator*(double _d) const;
    SSSurfaceMult& operator*=(double _d);

    //scalar divide
    SSSurfaceMult operator/(double _d) const;
    SSSurfaceMult& operator/=(double _d);

    virtual const string GetName() const{ return "SSSurfaceMult";}

    //I/O helper functions
    virtual void Read(istream& _is);
    virtual void Write(ostream& _os) const;

  protected:
    vector<SSSurface> m_cfgs;
};

//I/O operators for SSSurfaceMult
ostream& operator<< (ostream& _os, const SSSurfaceMult& _cfg);
istream& operator>> (istream& _is, SSSurfaceMult& _cfg);

template<class DistanceMetricPointer>
void
SSSurfaceMult::GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm) {
  //how to handle surface id?
  //how many Cfgs will we have?

}

#endif
#endif

#ifndef RVDISTANCE_H_
#define RVDISTANCE_H_

#include "MinkowskiDistance.h"
#include "ReachableVolumeUtil/ReachableVolume.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RVDistance : public MinkowskiDistance<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    RVDistance(bool _normalize = false);
    RVDistance(typename MPTraits::MPProblemType* _problem, XMLNode& _node);
    virtual double Distance(const CfgType& _c1, const CfgType& _c2);
    virtual ~RVDistance();
    double RotationalDistance(const CfgType& _c);
    static double RotationalDistance(const CfgType& _c, double _r2);
    static double InternalDistance(Environment* _env, CfgType _c1, CfgType _c2, bool _debug);

  private:
    double m_S;
    double m_S_rot;
    bool m_debug;
};

template<class MPTraits>
RVDistance<MPTraits>::
RVDistance(bool _normalize) :
  MinkowskiDistance<MPTraits>(2, 2, 1.0/2, _normalize) {
    this->m_name = "ReachableVolume";
  }

template<class MPTraits>
RVDistance<MPTraits>::
RVDistance(typename MPTraits::MPProblemType* _problem, XMLNode& _node) :
  MinkowskiDistance<MPTraits>(_problem, _node, false) {
    this->m_name = "ReachableVolume";
    /*
       this->m_r1 = 2;
       this->m_r2 = 2;
       this->m_r3 = 1.0/2;
       */
    this->m_r1 = 1;
    this->m_r2 = 1;
    this->m_r3 = 1;
    this->m_normalize = _node.Read("normalize", false, true,
        "flag if position dof should be normalized by environment diagonal");
    m_S = _node.Read("S", false, .5, 0.0, 1.0,
        "S, the scaling factor used by RV distance metric d=S*TranslationalDistance + (1-S)*RVDistance");
    m_S_rot = _node.Read("S_rot", false, .5, 0.0, 1.0,
        "S_rot, the rotational scaling factor");
  }

template<class MPTraits>
double
RVDistance<MPTraits>::RotationalDistance(const CfgType& _c) {
  return RotationalDistance(_c, this->m_r2);
}

template<class MPTraits>
double
RVDistance<MPTraits>::RotationalDistance(const CfgType& _c, double _r2) {
  vector<double> r = _c.GetRotation();
  double rot = 0;
  for(size_t i=0; i<r.size(); ++i)
    rot += pow(fabs(r[i]), _r2);
  return rot;
}

template<class MPTraits>
double
RVDistance<MPTraits>::InternalDistance(Environment* _env, CfgType _c1, CfgType _c2,bool _debug = false) {
  _c1.ResetRigidBodyCoordinates();
  _c2.ResetRigidBodyCoordinates();

  //get joint positions of _c1
  _c1.ConfigureRobot();
  shared_ptr<vector<Vector3d> > joints1 = shared_ptr<vector<Vector3d> >(new vector<Vector3d>);
  _env->GetRobot(_c1.GetRobotIndex())->PolygonalApproximation(*joints1);

  //get joint positions of _c2
  _c2.ConfigureRobot();
  shared_ptr<vector<Vector3d> > joints2 = shared_ptr<vector<Vector3d> >(new vector<Vector3d>);
  _env->GetRobot(_c2.GetRobotIndex())->PolygonalApproximation(*joints2);


  //compute sum of distances
  double d=0;
  for(size_t i=0; i<joints1->size(); ++i){
    if(_debug)
      cout<<(*joints1)[i]<<"\t\t"<<(*joints2)[i]<<"\t\td="<<ReachableVolume::distance((*joints1)[i],(*joints2)[i])<<endl;
    d+=ReachableVolume::distance((*joints1)[i],(*joints2)[i]);
  }
  if(_debug)
    cout<<"internal distance = "<<d<<endl;
  return d;
}


template<class MPTraits>
double
RVDistance<MPTraits>::Distance(const CfgType& _c1, const CfgType& _c2) {
  //my stuff goes here
  m_debug=false;
  if(m_debug){
    cout<<"c1= "<<_c1<<endl;
    cout<<"c2= "<<_c2<<endl;
  }
  CfgType diff = _c1 - _c2;//this->DifferenceCfg(_c1, _c2);
  if(m_debug){
    cout<<"diff= "<<diff<<endl;
  }
  //double pos = this->PositionDistance(_env, diff);
  double pos = this->PositionDistance(diff);
  double rot = this->RotationalDistance(diff);
  Environment* env = this->GetMPProblem()->GetEnvironment();
  double rv = InternalDistance(env,_c1,_c2,m_debug);
  if(m_debug){
    cout<<"pos="<<pos<<", rot="<<rot<<" total ="<<pow(m_S*(pos+rot)+(1-m_S)*rv, this->m_r3)<<endl;
  }
  return pow(m_S*((1-m_S_rot)*pos+m_S_rot*rot)+(1-m_S)*rv, this->m_r3);
}

template<class MPTraits>
RVDistance<MPTraits>::~RVDistance() {}



#endif

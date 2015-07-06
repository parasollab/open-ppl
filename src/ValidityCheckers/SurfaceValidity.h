#ifndef SURFACEVALIDITY_H_
#define SURFACEVALIDITY_H_

#if(defined(PMPCfgSurface) || defined(PMPSSSurfaceMult))

#include "Environment/Environment.h"
#include "Environment/FixedBody.h"
#include "Environment/SurfaceMultiBody.h"
#include "Utilities/MetricUtils.h"
#include "ValidityCheckers/ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class SurfaceValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    SurfaceValidity(string _vcLabel="");
    SurfaceValidity(typename MPTraits::MPProblemType* _problem, XMLNode& _node) ;

    virtual ~SurfaceValidity() {}

    virtual bool
      IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName);

  private:
    string m_vcLabel;
};//end SurfaceValidity class definition


template<class MPTraits>
SurfaceValidity<MPTraits>::SurfaceValidity(string _vcLabel) : ValidityCheckerMethod<MPTraits>(){
  this->m_name = "SurfaceValidity";
  this->m_vcLabel = _vcLabel;
}

template<class MPTraits>
SurfaceValidity<MPTraits>::SurfaceValidity(typename MPTraits::MPProblemType* _problem, XMLNode& _node)
  : ValidityCheckerMethod<MPTraits>(_problem, _node){
    this->m_name = "SurfaceValidity";
    this->m_vcLabel = _node.Read("vc_method", true, "", "Validity Checker Method");
  }

template<class MPTraits>
bool
SurfaceValidity<MPTraits>::IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName){
  Environment* env = this->GetMPProblem()->GetEnvironment();

  bool result = false;
  int sid = _cfg.GetSurfaceID();
  if( this->m_debug )
     cout << " active bodies: " << env->NumRobots() << endl;
  if( sid == -1 ) {
    //call default validity checker specified
    result = this->GetMPProblem()->GetValidityChecker(m_vcLabel)->IsValid(_cfg, _cdInfo, _callName);
  }
  else {
    //do surface validity based on sid
    //check if on surface
    Point2d pt = _cfg.GetPos();
    double  h  = _cfg.GetHeight();
    int numSurfaces = env->NumSurfaces();
    if( sid>=0 && sid < numSurfaces ) {
      shared_ptr<SurfaceMultiBody> surfaceBody = env->GetSurface(sid);
      shared_ptr<FixedBody> fb = surfaceBody->GetFixedBody(0);
      GMSPolyhedron& polyhedron = fb->GetWorldPolyhedron();
      result = polyhedron.IsOnSurface(pt, h);
    }
    //////////////////////////////////////////////////////////////////////////////
  }

  _cfg.SetLabel("VALID", result);
  return result;
}

#endif
#endif

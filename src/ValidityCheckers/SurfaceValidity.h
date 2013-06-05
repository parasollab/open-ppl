#ifndef SURFACEVALIDITY_H_
#define SURFACEVALIDITY_H_

#if(defined(PMPCfgSurface) || defined(PMPSSSurfaceMult))

#include "MPProblem/Environment.h"
#include "Utilities/MetricUtils.h"
#include "ValidityCheckerMethod.h"

template<class MPTraits>
class SurfaceValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    
    SurfaceValidity(string _vcLabel="");
    SurfaceValidity(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) ;

    virtual ~SurfaceValidity() {}

    virtual bool 
      IsValidImpl(CfgType& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, string *_callName);

  private:
    string m_vcLabel;
};//end SurfaceValidity class definition


template<class MPTraits>
SurfaceValidity<MPTraits>::SurfaceValidity(string _vcLabel) : ValidityCheckerMethod<MPTraits>(){
  this->m_name = "SurfaceValidity";
  this->m_vcLabel = _vcLabel;
}

template<class MPTraits>
SurfaceValidity<MPTraits>::SurfaceValidity(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) 
  : ValidityCheckerMethod<MPTraits>(_problem, _node){
    _node.verifyName("SurfaceValidity");
    this->m_name = "SurfaceValidity";
    this->m_vcLabel = _node.stringXMLParameter("vc_method", true, "", "Validity Checker Method");
  }

template<class MPTraits>
bool 
SurfaceValidity<MPTraits>::IsValidImpl(CfgType& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, string *_callName){

  bool result = false;
  int sid = _cfg.GetSurfaceID();
  if( this->m_debug ) {
     cout << " active bodies: " << _env->GetActiveBodyCount() << " usable bodies: " << _env->GetUsableMultiBodyCount() << endl;
  }
  if( sid == -1 ) { 
    //call default validity checker specified
    result = this->GetMPProblem()->GetValidityChecker(m_vcLabel)->IsValid(_cfg, _env, _stats, _cdInfo, _callName);
  }
  else {
    //do surface validity based on sid
    //check if on surface
    Point2d pt = _cfg.GetPos();
    double  h  = _cfg.GetHeight();
    int numSurfaces = _env->GetNavigableSurfacesCount();
    if( sid>=0 && sid < numSurfaces ) {
      shared_ptr<MultiBody> surface_body = _env->GetNavigableSurface(sid);
      shared_ptr<FixedBody> fb = surface_body->GetFixedBody(0);
      GMSPolyhedron & polyhedron = fb->GetWorldPolyhedron();
      result = polyhedron.IsOnSurface(pt, h);
    }
    //////////////////////////////////////////////////////////////////////////////
  }

  _cfg.SetLabel("VALID", result);
  return result;
}

#endif
#endif

#include "SurfaceValidity.h"

SurfaceValidity::
SurfaceValidity() : ValidityCheckerMethod() {
  m_name = "SurfaceValidity";
}

SurfaceValidity::
SurfaceValidity(string _vcLabel) : ValidityCheckerMethod(), m_vcLabel(_vcLabel) {
  m_name = "SurfaceValidity";
}

SurfaceValidity::
SurfaceValidity(XMLNodeReader& _node, MPProblem* _problem) : ValidityCheckerMethod(_node, _problem) {
  m_name = "SurfaceValidity";
  m_vcLabel = _node.stringXMLParameter("vc_method", true, "", "Validity Checker Method");
}

bool 
SurfaceValidity::IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, string * _callName) {
  bool result=false;

#ifndef PMPCfgSurface
  cerr << " Calling SurfaceValidity::IsValidImpl with wrong cfg type: " << _cfg.GetName() << endl;
  exit(0);
#else
  int sid = ((Cfg_surface&) _cfg).getSurfaceID();
  //bool result;
  if( sid == -1 ) { 
    //call default validity checker specified
    ValidityChecker* vc = this->GetMPProblem()->GetValidityChecker();
    result = vc->GetMethod(m_vcLabel)->IsValid(_cfg, _env, _stats, _cdInfo, _callName);

  }
  else {
    //do surface validity based on sid
    //check if on surface
    //////////////////////////////////////////////////////////////////////////////
    //Point2d pt = ((Cfg_surface&) _cfg).getPos();
    //double  h  = ((Cfg_surface&) _cfg).getHeight();
    Point2d pt = ((Cfg_surface&) _cfg).getPos();
    double  h  = ((Cfg_surface&) _cfg).getHeight();
    int numSurfaces = _env->GetNavigableSurfacesCount();
    if( sid>=0 && sid < numSurfaces ) {
      shared_ptr<MultiBody> surface_body = _env->GetNavigableSurface(sid);
      shared_ptr<FixedBody> fb = surface_body->GetFixedBody(0);
      GMSPolyhedron & polyhedron = fb->GetWorldPolyhedron();
      //Point3d surfPt3d = polyhedron.getRandPtOnSurface();
      bool onSurf = polyhedron.IsOnSurface(pt, h);
      result = onSurf;
    }
    //////////////////////////////////////////////////////////////////////////////
  }
  _cfg.SetLabel("VALID", result);
#endif

  return result;
}

bool SurfaceValidity::
isInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo) { 
#ifndef PMPCfgSurface
  cerr << " Calling SurfaceValidity::isInsideObstacle with wrong cfg type: " << _cfg.GetName() << endl;
  exit(0);
#endif

#ifdef PMPCfgSurface
  int sid = ((Cfg_surface&) _cfg).getSurfaceID();
  //bool result;
  bool result=false;
  //_stats
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(0);//using 0 by default
  StatClass* stats = region->GetStatClass();
  string callee="SurfaceValidity::isInsideObstacle";

  if( sid == -1 ) { 
    //call default validity checker specified
    ValidityChecker* vc = this->GetMPProblem()->GetValidityChecker();
    CfgType tmpCfg = (CfgType) _cfg;
    result = vc->GetMethod(m_vcLabel)->IsValid(tmpCfg, _env, *stats, _cdInfo, false, &callee);
    return !result;
  }
  else {
    //do surface validity based on sid
    //check if on surface
    //////////////////////////////////////////////////////////////////////////////
    Point2d pt = ((Cfg_surface&) _cfg).getPos();
    double  h  = ((Cfg_surface&) _cfg).getHeight();
    int numSurfaces = _env->GetNavigableSurfacesCount();
    if( sid>=0 && sid < numSurfaces ) {
      shared_ptr<MultiBody> surface_body = _env->GetNavigableSurface(sid);
      shared_ptr<FixedBody> fb = surface_body->GetFixedBody(0);
      GMSPolyhedron & polyhedron = fb->GetWorldPolyhedron();
      //Point3d surfPt3d = polyhedron.getRandPtOnSurface();
      bool onSurf = polyhedron.IsOnSurface(pt, h);
      result = onSurf;
    }
    else { 
      return false;
    }
    return !result;
    //////////////////////////////////////////////////////////////////////////////
  }
#endif
}

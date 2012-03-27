#ifndef SURFACEVALIDITY_H_
#define SURFACEVALIDITY_H_

#include "ValidityCheckerMethod.hpp"
#include <string>
#include <map>
using namespace std;

template<typename CFG>
class SurfaceValidity : public ValidityCheckerMethod {
public:
  SurfaceValidity() { }
  SurfaceValidity(string _vcLabel);
  SurfaceValidity(XMLNodeReader& _node, MPProblem* _problem);
  ~SurfaceValidity() { }

  virtual bool 
    IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
	CDInfo& _cdInfo, bool _enablePenetration, string *_callName);

  virtual bool isInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo); 


private:
  string m_vcLabel;
};

template<typename CFG>
SurfaceValidity<CFG>::
SurfaceValidity(string _vcLabel) : 
  m_vcLabel(_vcLabel){ }

template<typename CFG>
SurfaceValidity<CFG>::
SurfaceValidity(XMLNodeReader& _node, MPProblem* _problem) :
  ValidityCheckerMethod(_node, _problem) {

    m_vcLabel    = _node.stringXMLParameter("vc_method", true, "", "Validity Checker Method");

  }


template<typename CFG>
bool 
SurfaceValidity<CFG>::IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, bool _enablePenetration, string * _callName) {

  bool result=false;

#ifndef PMPCfgSurface
  cerr << " Calling SurfaceValidity::IsValid with wrong cfg type: " << _cfg.GetName() << endl;
  exit(0);
#else
  int sid = ((Cfg_surface&) _cfg).getSurfaceID();
  //bool result;
  if( sid == -1 ) { 
    //call default validity checker specified
    ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
    result = vc->GetVCMethod(m_vcLabel)->IsValid(_cfg, _env, _stats, _cdInfo, _enablePenetration, _callName);

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

template<typename CFG>
bool SurfaceValidity<CFG>::
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
  MPRegion<CFG,WeightType>* region = GetMPProblem()->GetMPRegion(0);//using 0 by default
  StatClass* stats = region->GetStatClass();
  string callee="SurfaceValidity::isInsideObstacle";

  if( sid == -1 ) { 
    //call default validity checker specified
    ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
    CFG tmpCfg = (CFG) _cfg;
    result = vc->GetVCMethod(m_vcLabel)->IsValid(tmpCfg, _env, *stats, _cdInfo, false, &callee);
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

#endif

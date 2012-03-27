/**
 * SurfaceLP.h
 * This class defines the surface local planner which performs 
 * connections between configurations and handles checking between
 * validity between CFGs on any kind of surface. 
 *
 * Last Updated : 02/03/12 
 * Update Author: Sam R.
 */

#ifndef SURFACELP_H_
#define SURFACELP_H_

#include "StraightLine.h"
typedef ElementSet<LocalPlannerMethod<CfgType,WeightType> >::MethodPointer LocalPlannerPointer;
template <class CFG, class WEIGHT> class LocalPlanners;

template <class CFG, class WEIGHT> 
class SurfaceLP: public StraightLine<CFG, WEIGHT> {
public:

  SurfaceLP();
  SurfaceLP(XMLNodeReader& _inNode, MPProblem* _inProblem);
  virtual ~SurfaceLP();

  virtual void PrintOptions(ostream& _outOs);
  virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

  virtual bool IsConnected(Environment *_env, StatClass& _stats,
      shared_ptr<DistanceMetricMethod > _dm, const CFG & _c1, const CFG & _c2, 
      CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput, double _positionRes, 
      double _orientationRes, bool _checkCollision=true, 
      bool _savePath=false, bool _saveFailedPath=false);


protected:
};

// Definitions for Constructors and Destructor
template <class CFG, class WEIGHT>
SurfaceLP<CFG, WEIGHT>::
SurfaceLP() : StraightLine<CFG, WEIGHT>() {
  this->SetName("SurfaceLP");
}

template <class CFG, class WEIGHT> 
SurfaceLP<CFG, WEIGHT>::
SurfaceLP(XMLNodeReader& _inNode, MPProblem* _inProblem) : 
  StraightLine<CFG, WEIGHT>(_inNode, _inProblem) {
    this->SetName("SurfaceLP");

  }

template <class CFG, class WEIGHT> 
SurfaceLP<CFG, WEIGHT>::~SurfaceLP() { }

// Definitions for I/O and Access
template <class CFG, class WEIGHT> 
void SurfaceLP<CFG, WEIGHT>::
PrintOptions(ostream& _outOs) {
  _outOs << "    " << this->GetName() << "::  ";
  _outOs << "vcMethod = " << " " << this->m_vcMethod << " ";
  _outOs << endl;
}

template <class CFG, class WEIGHT> 
LocalPlannerMethod<CFG, WEIGHT>* 
SurfaceLP<CFG, WEIGHT>::CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT>* _copy = new SurfaceLP<CFG, WEIGHT>(*this);
  return _copy;
}


// Main IsConnected Function
template <class CFG, class WEIGHT> bool SurfaceLP<CFG,WEIGHT>::
IsConnected(Environment* _env, StatClass& _stats,
    shared_ptr< DistanceMetricMethod> _dm,
    const CFG& _c1, const CFG& _c2, CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, 
    bool _savePath, bool _saveFailedPath) {  

#ifndef PMPCfgSurface
  cerr << " Calling SurfaceLP::IsConnected with wrong cfg type: " << _c1.GetName() << endl;
  exit(0);
#endif

  bool connected = false;
#ifdef PMPCfgSurface
  _savePath = true;
  _stats.IncLPAttempts(this->GetNameAndLabel());

  _lpOutput->path.clear();
  if( _c1.getSurfaceID() == _c2.getSurfaceID() ) {
    //basic straight line handles this case
    connected = StraightLine<CFG,WEIGHT>::IsConnected(_env,_stats,_dm,_c1,_c2,_col,_lpOutput,
	_positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);
  }
  else {
    //This is the complicated case
    //They are on different surfaces and we must check incrementally and validate
    //height component
    connected = true; //assume it starts connected (the check will negate this)

    //build or make an increment (as in SL LP)
    ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
    //typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_vcMethod);
    typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(StraightLine<CFG, WEIGHT>::m_vcMethod);
    //typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod( StraightLine<CFG, WEIGHT>::GetVCMethod() );
    double _positionRes = 0.05;
    double _orientationRes = 0.05;
    int nTicks;
    CFG tick;
    tick = _c1; 
    CFG incr;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    incr.FindIncrement(_c1,_c2,&nTicks,_positionRes,_orientationRes, _env->GetRdRes());
#else
    incr.FindIncrement(_c1,_c2,&nTicks,_positionRes,_orientationRes);
#endif
    string callee = this->GetName();
    string method = "-SurfaceLP::IsConnected";
    CDInfo cdInfo;
    callee=callee+method;
    string _callee = callee;

    double delta = 0.2;
    double maxDiff = 1.5;
    bool allOnValidSurface = true;
    vector<int> surfaceIDs;
    for(int i = 1; i < nTicks && allOnValidSurface; i++){ //don't need to check the ends, _c1 and _c2
      tick.Increment(incr);
      bool foundValidSurfForTick=false;
      CDInfo tmp_cdInfo;
      for(int sid=-1; sid<_env->GetNavigableSurfacesCount()&&!foundValidSurfForTick; sid++) {
	CFG tmpTick = tick;
	((Cfg_surface&) tmpTick).setSurfaceID(sid);//set SurfaceID to test if 2D collision is okay
	if( vcm->IsValid(tmpTick, _env, _stats, tmp_cdInfo, true, &_callee) ) {
	  if( sid == -1 ) {
	    if( fabs(tick.GetSingleParam(1))<delta ) {
	      //this is valid, -1 should have y-value 0
	      foundValidSurfForTick = true;
	      surfaceIDs.push_back(sid);
	    }
	  }
	  else {
	    ///////////////////////////////////////////////////////////////////////////////
	    //from 0--numSurfaces-1, check height
	    shared_ptr<MultiBody> surface_body = _env->GetNavigableSurface(sid);
	    shared_ptr<FixedBody> fb = surface_body->GetFixedBody(0);
	    GMSPolyhedron & polyhedron = fb->GetWorldPolyhedron();
	    bool isValid=false;
	    Point2d pt = tick.getPos();
	    double tH = polyhedron.HeightAtPt(pt, isValid); 
	    if( isValid ) {
	      double hDiff = fabs(tH-tick.GetSingleParam(1));
	      if( hDiff < delta ) {
		foundValidSurfForTick = true;
		surfaceIDs.push_back(sid);
	      }
	    }

	    ///////////////////////////////////////////////////////////////////////////////
	  }
	}//endif vcm->IsValid

      }//endfor j

      if( !foundValidSurfForTick ) {
	//this edge shouldn't be considered valid
	//set appropriate values and break
	allOnValidSurface = false;
	connected = false;
      }

    }//endfor i<nTicks
  }


#endif
  if ( connected )
    _stats.IncLPConnections(this->GetNameAndLabel() );  
  return connected;
}


#endif

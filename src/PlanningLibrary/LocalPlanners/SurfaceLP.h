#ifndef SURFACE_LP_H_
#define SURFACE_LP_H_

#ifdef PMPCfgSurface
#include "Cfg/CfgSurface.h"

#include "StraightLine.h"

#include "Environment/FixedBody.h"
#include "Environment/SurfaceMultiBody.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief Validate paths between configurations on different surfaces
/// @tparam MPTraits Motion planning universe
///
/// The surface local planner performs connections between configurations and
/// handles checking between validity between CFGs on any kind of surface.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class SurfaceLP : public StraightLine<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    SurfaceLP(double _acceptableHeightDiff=1, const string& _vcLabel = "", bool _evalation = false,
        bool _saveIntermediates = false);
    SurfaceLP(MPProblemType* _problem, XMLNode& _node);
    virtual ~SurfaceLP();

    virtual void Print(ostream& _os) const;

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = true, bool _saveFailedPath = false);

    bool IsOnSpecifiedSurface(Environment* _env, CfgType& _c1, int _sid);

    double m_acceptableHeightDiff;
};

// Definitions for Constructors and Destructor
template<class MPTraits>
SurfaceLP<MPTraits>::SurfaceLP(double _acceptableHeightDiff, const string& _vcLabel, bool _evalation,
        bool _saveIntermediates) : StraightLine<MPTraits>(_vcLabel, _evalation,
          _saveIntermediates) {
  this->SetName("SurfaceLP");
  m_acceptableHeightDiff = _acceptableHeightDiff;
}

template<class MPTraits>
SurfaceLP<MPTraits>::SurfaceLP(MPProblemType* _problem, XMLNode& _node) :
    StraightLine<MPTraits>(_problem, _node) {
  this->SetName("SurfaceLP");
  m_acceptableHeightDiff = 0.75;
}

template<class MPTraits>
SurfaceLP<MPTraits>::~SurfaceLP() { }

// Definitions for I/O and Access
template<class MPTraits>
void
SurfaceLP<MPTraits>::Print(ostream& _os) const {
  StraightLine<MPTraits>::Print(_os);
  _os << "\tvc label : " << this->m_vcLabel
      << "\n\tacceptable height diff = " << m_acceptableHeightDiff
      << endl;
}

template<class MPTraits>
bool
SurfaceLP<MPTraits>::IsOnSpecifiedSurface(Environment* _env, CfgType& _c1, int _sid) {
  string callee = this->GetNameAndLabel() + "-SurfaceLP::IsConnected";
  ValidityCheckerPointer vcm =
      this->GetMPProblem()->GetValidityChecker(StraightLine<MPTraits>::m_vcLabel);
  bool foundValidSurf = false;
  if(_sid == BASE_SURFACE) {
    if(vcm->IsValid(_c1, callee)) {
      if(fabs(_c1.GetHeight()) < m_acceptableHeightDiff) {
	//B
	//this is valid, BASESURF should have y-value 0
	foundValidSurf = true;
      }
    }//endif vcm->IsValid
  }
  else {
    ////////////////////////////////////////////////////////////////////////
    //from 0--numSurfaces-1, check height
    shared_ptr<SurfaceMultiBody> surfaceBody = _env->GetSurface(_sid);
    shared_ptr<FixedBody> fixedBody = surfaceBody->GetFixedBody(0);
    Vector3d bodyCenter = fixedBody->GetCenterOfMass();
    GMSPolyhedron& polyhedron = fixedBody->GetWorldPolyhedron();
    double bodyRadius = polyhedron.m_maxRadius;

    bool isValid = false;
    Point2d pt = _c1.GetPos();
    Vector3d v(pt[0],_c1.GetHeight(),pt[1]);
    if( this->m_debug )
       cout << " pos: " << v << " sid: " << _sid << " body center: " << bodyCenter << " radius: " << bodyRadius << endl;
    if( (v-bodyCenter).norm() < 3*bodyRadius ) { //quick check on radius
      double tH = polyhedron.HeightAtPt(pt, isValid);
      if(isValid) {
        double hDiff = fabs(tH - _c1.GetHeight());
        if(this->m_debug)
	  cout << " sid: " << _sid
	     << " isValid: " << isValid
	     << " hDiff: " << hDiff
	     << " of acceptableHDiff: " << m_acceptableHeightDiff
	     << " tH: " << tH
	     << " tickHeight: " << _c1.GetHeight() << endl;
        if(hDiff < m_acceptableHeightDiff) {
	  foundValidSurf = true;
        }
      }
    }
    else {
       if( this->m_debug )
	  cout  << " out of range completely? " << endl;
    }

    ////////////////////////////////////////////////////////////////////////
  }
  return foundValidSurf;
}

// Main IsConnected Function
template<class MPTraits>
bool
SurfaceLP<MPTraits>::IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath){
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  bool connected = false;
  stats->IncLPAttempts(this->GetNameAndLabel());

  _lpOutput->m_path.clear();
  //This is where we check if each intermediate configuration is on specified
  //surface or a neighboring surface.
  //If they are on different surfaces and we must check incrementally and
  //validate height component
  connected = true; //assume it starts connected (the check will negate this)

  //build or make an increment (as in SL LP)
  ValidityCheckerPointer vc =
      this->GetMPProblem()->GetValidityChecker(this->m_vcLabel);
  int nTicks;
  CfgType tick = _c1;
  CfgType incr;
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);
  string callee = this->GetNameAndLabel() + "::IsConnected";
  CDInfo cdInfo;

  bool allOnValidSurface = true;
  //vector<int> surfaceIDs;
  if(this->m_debug)
     cout <<endl<< "c1: " << _c1 << " c2: " << _c2 << endl;

  for(int i = 1; i < nTicks && allOnValidSurface; i++){
    //don't need to check the ends, _c1 and _c2
    tick += incr;
    if(this->m_debug)
      cout << " SurfaceLP - ticki: " << i << " of nTicks: " << nTicks << " tick: " << tick <<endl;
    bool foundValidSurfForTick = false;

    if( IsOnSpecifiedSurface(env, tick, tick.GetSurfaceID()) ) {
       foundValidSurfForTick=true;
       //cout << " Quick exit...IsOnSpecifiedSurface tick: " << tick << endl;
       if( this->m_debug ) cout << "\t tick: " << i << " on surface: " << tick.GetSurfaceID() << endl;
    }
    else {
      for(int sid = BASE_SURFACE; sid < (int)env->NumSurfaces() &&
          !foundValidSurfForTick; sid++) {
	CfgType tmpTick = tick;
	tmpTick.SetSurfaceID(sid);//set SurfaceID to test if 2D collision is okay
	if( IsOnSpecifiedSurface(env, tmpTick,sid) ) {
	   foundValidSurfForTick=true;
	   if( this->m_debug ) cout << "\t tick: " << i << " on surface: " << sid << endl;
	}
      }
    }

    if(!foundValidSurfForTick) {
      //this edge shouldn't be considered valid
      //set appropriate values and break
      allOnValidSurface = false;
      connected = false;
    }
  }//endfor i<nTicks

  if(connected)
    stats->IncLPConnections(this->GetNameAndLabel());
  return connected;
}

#endif
#endif

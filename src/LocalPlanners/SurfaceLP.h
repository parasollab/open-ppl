/**
 * SurfaceLP.h
 * This class defines the surface local planner which performs 
 * connections between configurations and handles checking between
 * validity between CFGs on any kind of surface. 
 */

#ifndef SURFACELP_H_
#define SURFACELP_H_

#ifdef PMPCfgSurface

#include "StraightLine.h"

template <class MPTraits>
class SurfaceLP: public StraightLine<MPTraits>{
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    SurfaceLP();
    SurfaceLP(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~SurfaceLP();

    virtual void PrintOptions(ostream& _os);

    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col, 
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, 
        bool _savePath = true, bool _saveFailedPath = false);
};

// Definitions for Constructors and Destructor
template<class MPTraits>
SurfaceLP<MPTraits>::
SurfaceLP() : StraightLine<MPTraits>() {
  this->SetName("SurfaceLP");
}

template<class MPTraits>
SurfaceLP<MPTraits>::
SurfaceLP(MPProblemType* _problem, XMLNodeReader& _node) : 
  StraightLine<MPTraits>(_problem, _node) {
    _node.verifyName("SurfaceLP");
    this->SetName("SurfaceLP");
  }

template<class MPTraits>
SurfaceLP<MPTraits>::~SurfaceLP() { }

// Definitions for I/O and Access
template<class MPTraits>
void SurfaceLP<MPTraits>::
PrintOptions(ostream& _os) {
  _os << "    " << this->GetName() << "::  ";
  _os << "vcMethod = " << " " << this->m_vcMethod << " ";
  _os << endl;
}

// Main IsConnected Function
template<class MPTraits>
bool SurfaceLP<MPTraits>::
IsConnected(Environment* _env, StatClass& _stats,
        DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col, 
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision, 
        bool _savePath, bool _saveFailedPath){


  bool connected = false;
  _stats.IncLPAttempts(this->GetNameAndLabel());

  _lpOutput->path.clear();
  if( _c1.GetSurfaceID() == _c2.GetSurfaceID() ) {
    //basic straight line handles this case
    connected = StraightLine<MPTraits>::IsConnected(_env,_stats,_dm,_c1,_c2,_col,_lpOutput,
        _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);
  }
  else {
    //This is the complicated case
    //They are on different surfaces and we must check incrementally and validate
    //height component
    connected = true; //assume it starts connected (the check will negate this)

    //build or make an increment (as in SL LP)
    ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(StraightLine<MPTraits>::m_vcMethod);
    double _positionRes = 0.05;
    double _orientationRes = 0.05;
    int nTicks;
    CfgType tick;
    tick = _c1; 
    CfgType incr;
    incr.FindIncrement(_c1,_c2,&nTicks,_positionRes,_orientationRes);
    string callee = this->GetName();
    string method = "-SurfaceLP::IsConnected";
    CDInfo cdInfo;
    callee=callee+method;
    string _callee = callee;

    double delta = 0.2;
    bool allOnValidSurface = true;
    vector<int> surfaceIDs;
    for(int i = 1; i < nTicks && allOnValidSurface; i++){ //don't need to check the ends, _c1 and _c2
      tick.Increment(incr);
      bool foundValidSurfForTick=false;
      CDInfo tmp_cdInfo;
      for(int sid=-1; sid<_env->GetNavigableSurfacesCount()&&!foundValidSurfForTick; sid++) {
        CfgType tmpTick = tick;
        ((CfgSurface&) tmpTick).SetSurfaceID(sid);//set SurfaceID to test if 2D collision is okay
        if( vcm->IsValid(tmpTick, _env, _stats, tmp_cdInfo, &_callee) ) {
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
            Point2d pt = tick.GetPos();
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

  if ( connected )
    _stats.IncLPConnections(this->GetNameAndLabel() );  
  return connected;
}
#endif
#endif

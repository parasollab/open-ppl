/**
 * SurfaceLP.h
 * This class defines the surface local planner which performs
 * connections between configurations and handles checking between
 * validity between CFGs on any kind of surface.
 */

#ifndef SURFACELP_H_
#define SURFACELP_H_

#ifdef PMPCfgSurface
#include "Cfg/CfgSurface.h"

#include "StraightLine.h"

template <class MPTraits>
class SurfaceLP : public StraightLine<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    SurfaceLP();
    SurfaceLP(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~SurfaceLP();

    virtual void PrintOptions(ostream& _os) const;

    virtual bool IsConnected(
        Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = true, bool _saveFailedPath = false);

    double m_acceptableHeightDiff;
};

// Definitions for Constructors and Destructor
template<class MPTraits>
SurfaceLP<MPTraits>::SurfaceLP() : StraightLine<MPTraits>() {
  this->SetName("SurfaceLP");
  m_acceptableHeightDiff = 0.75;
}

template<class MPTraits>
SurfaceLP<MPTraits>::SurfaceLP(MPProblemType* _problem, XMLNodeReader& _node) :
    StraightLine<MPTraits>(_problem, _node) {
  _node.verifyName("SurfaceLP");
  this->SetName("SurfaceLP");
  m_acceptableHeightDiff = 0.75;
}

template<class MPTraits>
SurfaceLP<MPTraits>::~SurfaceLP() { }

// Definitions for I/O and Access
template<class MPTraits>
void
SurfaceLP<MPTraits>::PrintOptions(ostream& _os) const {
  _os << "    " << this->GetNameAndLabel() << "::  "
      << "vcLabel = " << " " << this->m_vcLabel << " "
      << "acceptableHeightDiff = " << m_acceptableHeightDiff << " "
      << endl;
}

// Main IsConnected Function
template<class MPTraits>
bool
SurfaceLP<MPTraits>::IsConnected(
    Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath){

  bool connected = false;
  _stats.IncLPAttempts(this->GetNameAndLabel());

  _lpOutput->m_path.clear();
  //This is where we check if each intermediate configuration is on specified
  //surface or a neighboring surface.
  //If they are on different surfaces and we must check incrementally and
  //validate height component
  connected = true; //assume it starts connected (the check will negate this)

  //build or make an increment (as in SL LP)
  ValidityCheckerPointer vcm =
      this->GetMPProblem()->GetValidityChecker(StraightLine<MPTraits>::m_vcLabel);
  int nTicks;
  CfgType tick = _c1;
  CfgType incr;
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);
  string callee = this->GetName() + "-SurfaceLP::IsConnected";
  CDInfo cdInfo;

  bool allOnValidSurface = true;
  vector<int> surfaceIDs;

  for(int i = 1; i < nTicks && allOnValidSurface; i++){
    //don't need to check the ends, _c1 and _c2
    tick += incr;
    if(this->m_debug)
      cout << " SurfaceLP - ticki: " << i << " of nTicks: " << nTicks << " tick: " << tick <<endl;
    bool foundValidSurfForTick = false;
    CDInfo tmpCDInfo;
    for(int sid = BASE_SURFACE; sid < (int)_env->GetNavigableSurfacesCount() &&
        !foundValidSurfForTick; sid++) {
      CfgType tmpTick = tick;
      tmpTick.SetSurfaceID(sid);//set SurfaceID to test if 2D collision is okay

      if(sid == BASE_SURFACE) {
        if(vcm->IsValid(tmpTick, _env, _stats, tmpCDInfo, callee)) {
          if(fabs(tick.GetHeight()) < m_acceptableHeightDiff) {
            //B
            //this is valid, -1 should have y-value 0
            foundValidSurfForTick = true;
            surfaceIDs.push_back(sid);
          }
        }//endif vcm->IsValid
      }
      else {
        ////////////////////////////////////////////////////////////////////////
        //from 0--numSurfaces-1, check height
        shared_ptr<MultiBody> surfaceBody = _env->GetNavigableSurface(sid);
        shared_ptr<FixedBody> fixedBody = surfaceBody->GetFixedBody(0);
        GMSPolyhedron& polyhedron = fixedBody->GetWorldPolyhedron();
        bool isValid = false;
        Point2d pt = tick.GetPos();
        double tH = polyhedron.HeightAtPt(pt, isValid);
        if(isValid) {
          double hDiff = fabs(tH - tick.GetHeight());
          if(this->m_debug)
            cout << " sid: " << sid
                 << " isValid: " << isValid
                 << " hDiff: " << hDiff
                 << " of acceptableHDiff: " << m_acceptableHeightDiff
                 << " tH: " << tH
                 << " tickHeight: " << tick.GetHeight() << endl;
          if(hDiff < m_acceptableHeightDiff) {
            foundValidSurfForTick = true;
            surfaceIDs.push_back(sid);
          }
        }

        ////////////////////////////////////////////////////////////////////////
      }
    }//endfor j

    if(!foundValidSurfForTick) {
      //this edge shouldn't be considered valid
      //set appropriate values and break
      allOnValidSurface = false;
      connected = false;
    }
  }//endfor i<nTicks

  if(connected)
    _stats.IncLPConnections(this->GetNameAndLabel());
  return connected;
}

#endif
#endif

#ifndef TRACEOBSTACLE_H_
#define TRACEOBSTACLE_H_

#include "BasicExtender.h"

#include "Environment/FixedBody.h"
#include "Environment/StaticMultiBody.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Extend tangent to a workspace obstacle.
/// @tparam MPTraits Motion planning universe
///
/// Trace workspace obstacle. In this way of extending, the first
/// colliding triangle is found after extending by @c BasicExtender. An obstacle
/// vector is obtained from the first detected obstacle triangle that caused the
/// collision. The source configuration is extended in that direction with a
/// randomly generated orientation. This is useful when growing in areas where
/// difficult movements are needed.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class TraceObstacle : public BasicExtender<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    TraceObstacle(const string& _dmLabel = "", const string& _vcLabel = "",
        double _delta = 1.0, bool _randomOrientation = true);
    TraceObstacle(MPProblemType* _problem, XMLNode& _node);

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, LPOutput<MPTraits>& _lpOutput);
};

template<class MPTraits>
TraceObstacle<MPTraits>::TraceObstacle(const string& _dmLabel,
    const string& _vcLabel, double _delta, bool _randomOrientation) :
  BasicExtender<MPTraits>(_dmLabel, _vcLabel, _delta, _randomOrientation) {
    this->SetName("TraceObstacle");
  }

template<class MPTraits>
TraceObstacle<MPTraits>::TraceObstacle(MPProblemType* _problem,
    XMLNode& _node) :
  BasicExtender<MPTraits>(_problem, _node) {
    this->SetName("TraceObstacle");
  }

template<class MPTraits>
bool
TraceObstacle<MPTraits>::Extend(const CfgType& _near, const CfgType& _dir,
    CfgType& _new, LPOutput<MPTraits>& _lpOutput) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  CfgType newDir;
  CDInfo cdInfo;
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 10.0;

  // Expand to find a colliding triangle
  this->Expand(_near, _dir, _new, this->m_delta, _lpOutput, cdInfo,
    env->GetPositionRes(), env->GetOrientationRes());

  // Get an obstacle vector from the colliding triangle
  int cIndex = cdInfo.m_collidingObstIndex;
  int obsContactIndex = cdInfo.m_rapidContactID2;
  if( cIndex == -1 ) {
    cIndex = LRand() % env->NumObstacles();
    obsContactIndex = -1;
  }
  GMSPolyhedron& poly =
      env->GetObstacle(cIndex)->GetFixedBody(0)->GetWorldPolyhedron();
  vector<Vector3d>& vertexList    = poly.m_vertexList;
  vector<GMSPolygon>& polygonList = poly.m_polygonList;

  int polyIndex;
  if( obsContactIndex != -1 )
    polyIndex = obsContactIndex;
  else
    polyIndex = LRand() % polygonList.size();
  int randEdgeInd = LRand() % polygonList[polyIndex].m_vertexList.size();
  int v1Ind= polygonList[polyIndex].m_vertexList[randEdgeInd];
  randEdgeInd = (randEdgeInd+1) % polygonList[polyIndex].m_vertexList.size();
  int v2Ind= polygonList[polyIndex].m_vertexList[randEdgeInd];
  Vector3d obsVec = vecScale * (vertexList[v1Ind] - vertexList[v2Ind]);
  if( DRand() < 0.5 )
    obsVec *= -1.0;

  // Apply this obstacle vector
  newDir = _dir;
  for(size_t i = 0; i < newDir.PosDOF(); i++)
    newDir[i] = _near[i] + obsVec[i];

  // Expand with Random or Same Orientation
  return BasicExtender<MPTraits>::Extend(_near, newDir, _new, _lpOutput);
}

#endif

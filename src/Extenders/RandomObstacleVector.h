#ifndef RANDOMOBSTACLEVECTOR_H_
#define RANDOMOBSTACLEVECTOR_H_

#include "BasicExtender.h"

#include "Environment/FixedBody.h"
#include "Environment/StaticMultiBody.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Extend in a direction based upon a random obstacle vector.
/// @tparam MPTraits Motion planning universe
///
/// In this extend method, \f$q_{dir} = q_{near} + O\f$ where \f$O\f$ is a
/// random obstacle vector from all of the triangles in the environment. The
/// oriantation DOFs are set randomly.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RandomObstacleVector : public BasicExtender<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    RandomObstacleVector(const string& _dmLabel = "",
        const string& _vcLabel = "", double _delta = 1.0,
        bool _randomOrientation = true);
    RandomObstacleVector(MPProblemType* _problem, XMLNode& _node);

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, LPOutput<MPTraits>& _lpOutput);
};

template<class MPTraits>
RandomObstacleVector<MPTraits>::RandomObstacleVector(const string& _dmLabel,
    const string& _vcLabel, double _delta, bool _randomOrientation) :
  BasicExtender<MPTraits>(_dmLabel, _vcLabel, _delta, _randomOrientation) {
    this->SetName("RandomObstacleVector");
  }

template<class MPTraits>
RandomObstacleVector<MPTraits>::RandomObstacleVector(MPProblemType* _problem,
    XMLNode& _node) :
  BasicExtender<MPTraits>(_problem, _node) {
    this->SetName("RandomObstacleVector");
  }

template<class MPTraits>
bool
RandomObstacleVector<MPTraits>::Extend(const CfgType& _near,
    const CfgType& _dir, CfgType& _new, LPOutput<MPTraits>& _lpOutput) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 10.0;

  // Get an obstacle vector from env
  int numBodies = env->NumObstacles();
  if( numBodies > 1 ) {//this growth method only works with obstacles (need 2 multibodies)
    int randIndex = (LRand() % (numBodies-1)) + 1;
    GMSPolyhedron& poly = env->GetObstacle(randIndex)->GetFixedBody(0)->GetWorldPolyhedron();
    vector<Vector3d>& vertexList    = poly.m_vertexList;
    vector<GMSPolygon>& polygonList = poly.m_polygonList;

    // Random polygon
    int randPolyInd = LRand() % polygonList.size();
    int randEdgeInd = LRand() % polygonList[randPolyInd].m_vertexList.size();
    int v1Ind= polygonList[randPolyInd].m_vertexList[randEdgeInd];
    randEdgeInd = (randEdgeInd+1) % polygonList[randPolyInd].m_vertexList.size();
    int v2Ind= polygonList[randPolyInd].m_vertexList[randEdgeInd];
    Vector3d obsVec = vecScale * (vertexList[v1Ind] - vertexList[v2Ind]);
    if( DRand() < 0.5 )
      obsVec *= -1.0;

    // Apply this obstacle vector
    CfgType newDir = _dir;
    for(size_t i = 0; i < newDir.PosDOF(); i++)
      newDir[i] = _near[i] + obsVec[i];

    // Expand with the BasicExtender
    return BasicExtender<MPTraits>::Extend(_near, newDir, _new, _lpOutput);
  }

  return false;
}

#endif

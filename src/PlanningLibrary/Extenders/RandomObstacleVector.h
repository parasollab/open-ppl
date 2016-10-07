#ifndef RANDOM_OBSTACLE_VECTOR_H_
#define RANDOM_OBSTACLE_VECTOR_H_

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
template <typename MPTraits>
class RandomObstacleVector : public BasicExtender<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ///@}
    ///\name Construction
    ///@{

    RandomObstacleVector(const string& _dmLabel = "", const string& _vcLabel = "",
        double _min = .001, double _max = 1, bool _randomOrientation = true);

    RandomObstacleVector(MPProblemType* _problem, XMLNode& _node);

    virtual ~RandomObstacleVector() = default;

    ///@}
    ///\name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
RandomObstacleVector<MPTraits>::
RandomObstacleVector(const string& _dmLabel, const string& _vcLabel,
    double _min, double _max, bool _randomOrientation) :
    BasicExtender<MPTraits>(_dmLabel, _vcLabel, _min, _max, _randomOrientation) {
  this->SetName("RandomObstacleVector");
}


template <typename MPTraits>
RandomObstacleVector<MPTraits>::
RandomObstacleVector(MPProblemType* _problem, XMLNode& _node) :
    BasicExtender<MPTraits>(_problem, _node) {
  this->SetName("RandomObstacleVector");
}


template <typename MPTraits>
bool
RandomObstacleVector<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 10.0;

  // Get an obstacle vector from env
  int numBodies = env->NumObstacles();
  if( numBodies > 1 ) {
    //this growth method only works with obstacles (need 2 multibodies)
    int randIndex = (LRand() % (numBodies-1)) + 1;
    GMSPolyhedron& poly = env->GetObstacle(randIndex)->GetFixedBody(0)->
        GetWorldPolyhedron();
    vector<Vector3d>& vertexList    = poly.m_vertexList;
    vector<GMSPolygon>& polygonList = poly.m_polygonList;

    // Random polygon
    int randPolyInd = LRand() % polygonList.size();
    int randEdgeInd = LRand() % polygonList[randPolyInd].GetNumVertices();
    int v1Ind= polygonList[randPolyInd][randEdgeInd];
    randEdgeInd = (randEdgeInd+1) % polygonList[randPolyInd].GetNumVertices();
    int v2Ind= polygonList[randPolyInd][randEdgeInd];
    Vector3d obsVec = vecScale * (vertexList[v1Ind] - vertexList[v2Ind]);
    if( DRand() < 0.5 )
      obsVec *= -1.0;

    // Apply this obstacle vector
    CfgType newDir = _end;
    for(size_t i = 0; i < newDir.PosDOF(); i++)
      newDir[i] = _start[i] + obsVec[i];

    // Expand with the BasicExtender
    return BasicExtender<MPTraits>::Extend(_start, newDir, _new, _lp);
  }

  return false;
}

#endif

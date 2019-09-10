#ifndef PMPL_RANDOM_OBSTACLE_VECTOR_H_
#define PMPL_RANDOM_OBSTACLE_VECTOR_H_

#include "BasicExtender.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"

////////////////////////////////////////////////////////////////////////////////
/// Extend in a direction based upon a random obstacle vector.
///
/// In this extend method, \f$q_{dir} = q_{near} + O\f$ where \f$O\f$ is a
/// random obstacle vector from all of the triangles in the environment. The
/// oriantation DOFs are set randomly.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RandomObstacleVector : public BasicExtender<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    RandomObstacleVector();

    RandomObstacleVector(XMLNode& _node);

    virtual ~RandomObstacleVector() = default;

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
RandomObstacleVector<MPTraits>::
RandomObstacleVector() {
  this->SetName("RandomObstacleVector");
}


template <typename MPTraits>
RandomObstacleVector<MPTraits>::
RandomObstacleVector(XMLNode& _node) : BasicExtender<MPTraits>(_node) {
  this->SetName("RandomObstacleVector");
}

/*------------------------ ExtenderMethod Overrides --------------------------*/

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
  const size_t numObstacles = env->NumObstacles();
  if(numObstacles > 1) {
    //this growth method only works with obstacles (need 2 multibodies)
    //Need a random body index greater than 0:
    const size_t obstIndex = (LRand() % (numObstacles - 1)) + 1;
    MultiBody* const obst = env->GetObstacle(obstIndex);
    const unsigned int body = LRand() % obst->GetNumBodies(); // Any random body
    const GMSPolyhedron& poly = obst->GetBody(body)->GetWorldPolyhedron();
    const vector<Vector3d>& vertexList    = poly.GetVertexList();
    const vector<GMSPolygon>& polygonList = poly.GetPolygonList();

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

/*----------------------------------------------------------------------------*/

#endif

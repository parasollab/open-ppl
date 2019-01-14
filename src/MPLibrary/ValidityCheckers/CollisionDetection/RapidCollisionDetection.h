#ifndef PMPL_RAPID_COLLISION_DETECTION_H_
#define PMPL_RAPID_COLLISION_DETECTION_H_

#include "CollisionDetectionMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// RAPID checks for polygon collisions between two meshes.
///
/// RAPID is fast, but it cannot determine any auxiliary information or detect
/// when one mesh is completely inside the other. The contacts (colliding
/// triangle IDs) are stored in the CDInfo object.
///
/// Reference:
///   Stefan Gottschalk and Ming C. Lin and Dinesh Manocha. "OBBTree: A
///   Hierarchical Structure for Rapid Interference Detection". SIGGRAPH 1996.
///
/// @warning RAPID is not a thread-safe library; it uses static data as part of
///          its internal implementation. We cannot use RAPID in any
///          multi-threaded context, including the simulator - race conditions
///          will result! Use PQP instead, which uses separate structures for
///          each collision check.
///
/// @ingroup CollisionDetection
////////////////////////////////////////////////////////////////////////////////
class Rapid: public CollisionDetectionMethod {

  public:

    ///@name Construction
    ///@{

    Rapid();

    virtual ~Rapid();

    ///@}
    ///@name CollisionDetectionMethod Overrides
    ///@{

    static void Build(Body* const _body);

    virtual bool IsInCollision(const Body* const _body1,
        const Body* const _body2, CDInfo& _cdInfo) override;

    ///@}

};

#endif

#ifndef FREE_BODY_H
#define FREE_BODY_H

#include "Body.h"
#include <set>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief Movable Body in workspace
///
/// Movable Body (i.e., one piece of geometry) in the workspace. Provides for
/// computing transformations so that it can be validated.
////////////////////////////////////////////////////////////////////////////////
class FreeBody : public Body {
  public:

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner Owner of this body
    FreeBody(MultiBody* _owner);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner Owner of this body
    /// @param _polyhedron Geometry of body
    FreeBody(MultiBody* _owner, GMSPolyhedron& _polyhedron);

    FreeBody(const FreeBody&) = delete;
    FreeBody& operator=(const FreeBody&) = delete;
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    virtual bool IsFixedBody() const { return false; }

    virtual Transformation& GetWorldTransformation();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Set transformation of body
    /// @param _transformation Transformation
    void Configure(Transformation& _transformation);

    using Body::Read;
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Parse
    /// @param _is Stream
    /// @param _cbs Counting stream buffer
    void Read(istream& _is, CountingStreamBuffer& _cbs);

    friend ostream& operator<<(ostream& _os, FreeBody& _fb);

  private:
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute transformation of this body wrt the world frame
    /// @param visited Stores which bodies have been visited
    /// @return Transformation
    ///
    /// Compute transformation "this" body w.r.t the world frame in a recursive
    /// manner; multiply the world transformation of the previous body with the
    /// transformation from the proximal joint to the center of gravity of
    /// "this" body (Need a generalization for the connectionship, since
    /// currently it handles only one backward connection).
    Transformation& ComputeWorldTransformation(std::set<int>& visited);
};

#endif

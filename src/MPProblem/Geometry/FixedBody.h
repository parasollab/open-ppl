#ifndef FIXED_BODY_H_
#define FIXED_BODY_H_

#include "Body.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief Stationary Body in workspace
////////////////////////////////////////////////////////////////////////////////
class FixedBody : public Body {
  public:

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner Owner of this body
    /// @param _filename Filename
    FixedBody(MultiBody* _owner, const string& _filename = "");

    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner Owner of this body
    /// @param _polyhedron Geometry of body
    FixedBody(MultiBody* _owner, GMSPolyhedron& _polyhedron) __attribute__ ((deprecated)) ;

    FixedBody(const FixedBody&) = delete;
    FixedBody& operator=(const FixedBody&) = delete;
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    virtual bool IsFixedBody() const { return true; }

    virtual Transformation& GetWorldTransformation();

    using Body::Read;
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Parse
    /// @param _is Stream
    /// @param _cbs Counting stream buffer
    void Read(istream& _is, CountingStreamBuffer& _cbs);

    friend ostream& operator<<(ostream& _os, const FixedBody& _fb);
};

#endif

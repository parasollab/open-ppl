#ifndef FIXED_BODY_H_
#define FIXED_BODY_H_

#include "Body.h"

////////////////////////////////////////////////////////////////////////////////
/// A stationary object in workspace.
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class FixedBody : public Body {

  public:

    ///@name Construction
    ///@{

    /// @param _owner The owning multibody.
    /// @param _filename Filename
    FixedBody(MultiBody* _owner, const string& _filename = "");

    FixedBody(const FixedBody&) = delete;            ///< No copy
    FixedBody& operator=(const FixedBody&) = delete; ///< No assign

    virtual ~FixedBody() = default;

    ///@}
    ///@name Transformation
    ///@{

    virtual Transformation& GetWorldTransformation();

    /// Set the transform for this object.
    /// @param _worldTransformation Transformation w.r.t. world frame
    void PutWorldTransformation(Transformation& _worldTransformation);

    ///@}
    ///@name I/O
    ///@{

    using Body::Read;

    /// @brief Parse
    /// @param _is Stream
    /// @param _cbs Counting stream buffer
    void Read(istream& _is, CountingStreamBuffer& _cbs);

    friend ostream& operator<<(ostream& _os, const FixedBody& _fb);

    /// @}
};

#endif

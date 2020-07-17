#ifndef FREE_BODY_H_
#define FREE_BODY_H_

#include <set>

#include "Body.h"
#include "Connection.h"

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
    /// @brief Body type
    ////////////////////////////////////////////////////////////////////////////
    enum class BodyType {
      Planar,     ///< 2D
      Volumetric, ///< 3D
      Fixed,      ///< Fixed base
      Joint       ///< Joint
    };

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Body movement type
    ////////////////////////////////////////////////////////////////////////////
    enum class MovementType {
      Rotational,   ///< Rotation + translation
      Translational ///< Just translation
    };

    ///@name Construction
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner Owner of this body
    /// @param _index Index in MultiBody
    FreeBody(MultiBody* _owner, size_t _index);

    FreeBody(const FreeBody&) = delete;            ///< No copy
    FreeBody& operator=(const FreeBody&) = delete; ///< No assign

    ///@}
    ///@name IO Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _tag Tag
    /// @param _where Error information
    /// @return BodyType of _tag
    static BodyType GetBodyTypeFromTag(const string& _tag,
        const string& _where);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _tag Tag
    /// @param _where Error information
    /// @return MovementType of _tag
    static MovementType GetMovementTypeFromTag(const string& _tag,
        const string& _where);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _b BodyType
    /// @return Tag
    static string GetTagFromBodyType(BodyType _b);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _bm MovementType
    /// @return Tag
    static string GetTagFromMovementType(MovementType _bm);

    ///@}
    ///@name Body Information
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Is this body a base?
    bool IsBase() const {return m_bodyType != BodyType::Joint;}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _bt BodyType
    void SetBodyType(BodyType _bt) {m_bodyType = _bt;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Body type
    BodyType GetBodyType() const {return m_bodyType;}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _mt Base movement type of body
    void SetMovementType(MovementType _mt) {m_movementType = _mt;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Base movement type of body
    MovementType GetMovementType() const {return m_movementType;}

    ///@}
    ///@name Connection Information
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of forward Connection
    size_t ForwardConnectionCount() const {return m_forwardConnections.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of backward Connection
    size_t BackwardConnectionCount() const {return m_backwardConnections.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Index of desired forward Connection
    /// @return Requested forward Connection
    Connection& GetForwardConnection(size_t _index);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Index of desired backward Connection
    /// @return Requested backward Connection
    Connection& GetBackwardConnection(size_t _index);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determines if two bodies share the same joint
    /// @param _otherBody Second body
    /// @return True if adjacent
    bool IsAdjacent(shared_ptr<FreeBody> _otherBody) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determines if two bodies are within \p _i joints of each other
    /// @param _otherBody Second body
    /// @param _i Number of joints
    /// @return True if within \p _i joints
    bool IsWithinI(shared_ptr<FreeBody> _otherBody, size_t _i) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Link two Body, i.e., add a Connection between them
    /// @param _c Connection description
    void Link(Connection* _c);

    ///@}
    ///@name Transformation
    ///@{

    virtual Transformation& GetWorldTransformation();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Set transformation of body
    /// @param _transformation Transformation
    void Configure(Transformation& _transformation);

    Transformation& GetRenderTransformation();

    ////////////////////////////////////////////////////////////////////////////
    /// @return Rendering transformation without recomputing
    Transformation& RenderTransformation() {return m_renderTransformation;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Set transformation of body
    /// @param _transformation Transformation
    void ConfigureRender(Transformation& _transformation);

    /////////////////////////////////////////////////////////////////////////////
    /// @bried define the necessary connection(s) to close a loop
    void SetClosureIndices(pair<size_t, size_t> _connection);

    ///@}
    ///@name I/O
    ///@{

    using Body::Read;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Parse
    /// @param _is Stream
    /// @param _cbs Counting stream buffer
    void Read(istream& _is, CountingStreamBuffer& _cbs);

    friend ostream& operator<<(ostream& _os, FreeBody& _fb);

    ///@}

  private:
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determines if two bodies are within \p _i joints of each other
    /// @param _body1 First Body
    /// @param _body2 Second Body
    /// @param _i Number of joints
    /// @param _prevBody Previous Body
    /// @return True if within \p _i joints
    bool IsWithinI(const FreeBody* const _body1,
        const FreeBody* const _body2, size_t _i,
        const FreeBody* const _prevBody) const;

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
    Transformation& ComputeWorldTransformation(std::set<size_t>& visited);

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
    Transformation& ComputeRenderTransformation(std::set<size_t>& visited);

    size_t m_index;                            ///< Index in ActiveMultiBody
    BodyType m_bodyType;                       ///< Body type
    MovementType m_movementType;               ///< Movement type
    vector<Connection*> m_forwardConnections;  ///< Forward Connections
    vector<Connection*> m_backwardConnections; ///< Backward Connections

    Transformation m_renderTransformation;     ///< Rendering Transform
    vector<pair<size_t, size_t> > m_closureIndices;
};

#endif

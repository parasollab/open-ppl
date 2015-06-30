#ifndef FREE_BODY_H_
#define FREE_BODY_H_

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

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner Owner of this body
    FreeBody(MultiBody* _owner);

    FreeBody(const FreeBody&) = delete;            ///< No copy
    FreeBody& operator=(const FreeBody&) = delete; ///< No assign

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    static BodyType GetBodyTypeFromTag(const string& _tag,
        const string& _where);
    static MovementType GetMovementTypeFromTag(const string& _tag,
        const string& _where);

    static string GetTagFromBodyType(BodyType _b);
    static string GetTagFromMovementType(MovementType _bm);

    ////////////////////////////////////////////////////////////////////////////
    /// @return Is this body a base?
    bool IsBase() { return m_isBase; };
    ////////////////////////////////////////////////////////////////////////////
    /// @return Body type
    BodyType GetBodyType() { return m_bodyType; };
    ////////////////////////////////////////////////////////////////////////////
    /// @return Base movement type of body
    MovementType GetMovementType() { return m_movementType; };
    ////////////////////////////////////////////////////////////////////////////
    /// @param _baseType Type of base of this body
    //void SetBase(Base _baseType) { m_baseType = _baseType; };
    ////////////////////////////////////////////////////////////////////////////
    /// @param _baseMovementType Type of movement of the base of this body
    //void SetBaseMovement(BaseMovement _baseMovementType) { m_baseMovementType = _baseMovementType; };

    ////////////////////////////////////////////////////////////////////////////
    /// @param _connection Connection to add as forward Connection
    void AddForwardConnection(const Connection& _connection) {m_forwardConnection.push_back(_connection);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _connection Connection to add as backward Connection
    void AddBackwardConnection(const Connection& _connection) {m_backwardConnection.push_back(_connection);}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of forward Connection
    size_t ForwardConnectionCount() const {return m_forwardConnection.size();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of backward Connection
    size_t BackwardConnectionCount() const {return m_backwardConnection.size();}
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
    bool IsAdjacent(shared_ptr<FreeBody> _otherBody);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determines if two bodies are within \p _i joints of each other
    /// @param _otherBody Second body
    /// @param _i Number of joints
    /// @return True if within \p _i joints
    bool IsWithinI(shared_ptr<FreeBody> _otherBody, int _i);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determines if two bodies are within \p _i joints of each other
    /// @param _body1 First Body
    /// @param _body2 Second Body
    /// @param _i Number of joints
    /// @param _prevBody Previous Body
    /// @return True if within \p _i joints
    bool IsWithinIHelper(FreeBody* _body1, FreeBody* _body2, int _i,FreeBody* _prevBody);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Link two Body, i.e., add a Connection between them
    /// @param _otherBody Second body
    /// @param _transformationToBody2 Transformation to second body
    /// @param _dhparameters DH frame description
    /// @param _transformationToDHFrame Transformation to DH frame
    void Link(const shared_ptr<FreeBody>& _otherBody,
        const Transformation& _transformationToBody2,
        const DHparameters& _dhparameters,
        const Transformation& _transformationToDHFrame);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Link two Body, i.e., add a Connection between them
    /// @param _c Connection description
    void Link(const Connection& _c);

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

    bool m_isBase;                           ///< Base or Joint
    BodyType m_bodyType;                     ///< Body type
    MovementType m_movementType;             ///< Movement type
    vector<Connection> m_forwardConnection;  ///< Forward Connection s
    vector<Connection> m_backwardConnection; ///< Backward Connection s

};

#endif

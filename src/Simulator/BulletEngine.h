#ifndef BULLET_ENGINE_H_
#define BULLET_ENGINE_H_

#include "btBulletDynamicsCommon.h"

#include "glutils/gltraits.h"

// Bullet forward-declarations.
class btMultiBodyDynamicsWorld;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btMultiBodyConstraintSolver;
class btMultiBody;

// PMPL forward-declarations.
class MultiBody;


////////////////////////////////////////////////////////////////////////////////
/// Encapsulates the details of creating a bullet simulation.
////////////////////////////////////////////////////////////////////////////////
class BulletEngine final {

  ///@name Bullet Components
  ///@{

  /// The bullet world.
  btMultiBodyDynamicsWorld*        m_dynamicsWorld{nullptr};

  // The collision detection components.
  btDefaultCollisionConfiguration* m_collisionConfiguration{nullptr};
  btCollisionDispatcher*           m_dispatcher{nullptr};
  btBroadphaseInterface*           m_broadphase{nullptr};
  btMultiBodyConstraintSolver*     m_solver{nullptr};

  /// The collision shapes in our bullet world. These are stored separately from
  /// rigid bodies to allow bodies with the same shape to share collision
  /// structures.
  btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

  ///@}

  public:

    ///@name Construction
    ///@{

    BulletEngine();
    ~BulletEngine();

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Step the simulation forward.
    /// @param[in] _timestep The total length of time to advance the simulation.
    /// @param[in] _maxSubSteps The maximum number of sub-intervals to use.
    /// @param[in] _resolution The length of a sub interval.
    void Step(const btScalar _timestep, const int _maxSubSteps,
        const btScalar _resolution);

    ///@}
    ///@name Transform Access
    ///@{

    /// Get the current transform for a given object.
    /// @param[in] _i The object's index.
    /// @return An OpenGL transform matrix describing object _i's current
    ///         position and orientation.
    glutils::transform GetObjectTransform(const size_t _i) const;

    ///@}
    ///@name Modifiers
    ///@{

    /// Add a PMPL multibody to the simulation.
    /// @param[in] _m The multibody to add.
    btMultiBody* AddObject(MultiBody* _m);

    /// Add an object to the world.
    /// @param _shape The bullet collision shape that reprsents the object.
    /// @param _trans The world tranform of the object.
    /// @param _mass The mass of the _shape in the world.
    btMultiBody* AddObject(btCollisionShape* _shape, const btTransform& _trans,
        double _mass);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Build a bullet collision shape from a pmpl MultiBody.
    /// @param[in] _body The pmpl MultiBody.
    /// @return A bullet collision shape.
    btCollisionShape* BuildCollisionShape(MultiBody* _body);

    /// Build a bullet collision shape from an obj file.
    /// @param[in] _filename The obj file to use.
    /// @return A bullet triangle mesh collision shape.
    btTriangleMesh* BuildCollisionShape(const std::string& _filename);

    ///@}

};

#endif

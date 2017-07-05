#ifndef BULLET_ENGINE_H_
#define BULLET_ENGINE_H_

#include <functional>
#include <map>
#include <vector>

#include "Geometry/Bodies/Connection.h"

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
class Body;
class MultiBody;
class MPProblem;
class Robot;


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
  ///@name Call-back Functions
  ///@}

  /// A call-back function can be used to modify the physics engine behavior for
  /// a dynamic object. It will be executed by the bullet engine after each
  /// internal timestep.
  typedef std::function<void(void)> CallbackFunction;

  /// A set of call-back functions that are used for this engine object.
  typedef std::vector<CallbackFunction> CallbackSet;

  /// The set of all call-back functions for this simulation.
  CallbackSet m_callbacks;

  /// A map from all dynamics worlds (one per engine object) to their call-back
  /// sets. This is needed to get bullet to call the appropriate call-back set
  /// from a single, universal function.
  static std::map<btDynamicsWorld*, CallbackSet&> s_callbackMap;

  ///@}
  ///@name Other Internal State
  ///@{

  /// A pointer to the MPProblem being simulated.
  MPProblem* const m_problem;

  bool m_debug{false};  ///< Show debug messages?

  ///@}

  public:

    ///@name Construction
    ///@{

    BulletEngine(MPProblem* const _problem);

    ~BulletEngine();

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Step the simulation forward.
    /// @param _timestep    The total length of time to advance the simulation.
    void Step(const btScalar _timestep);

    ///@}
    ///@name Transform Access
    ///@{

    /// Get the current transform for a given object.
    /// @param _i The object's index.
    /// @param _j The component index (0 for base by default).
    /// @return An OpenGL transform matrix describing object _i's current
    ///         position and orientation.
    glutils::transform GetObjectTransform(const size_t _i, const size_t _j = 0)
        const;

    ///@}
    ///@name Modifiers
    ///@{

    /// Add a PMPL robot to the simulation.
    /// @param _robot The robot to add.
    btMultiBody* AddRobot(Robot* const _robot);

    /// Add a PMPL multibody to the simulation.
    /// @param _m The multibody to add.
    btMultiBody* AddObject(MultiBody* _m);

    /// Add an object to the world,
    /// @param _shape The bullet collision shape that reprsents the object.
    /// @param _transform The world tranform of object.
    /// @param _mass The mass of _shape in the world.
    btMultiBody* AddObject(btCollisionShape* _shape, btTransform _transform,
        const double _mass);

    /// Set the gravity in the world (this will also set it for all bodies)
    /// @param _gravityVec Is simply the 3-vector representing (x,y,z) gravity
    void SetGravity(const btVector3& _gravityVec);

    ///@}
    ///@name Call-back Function Interface
    ///@{

    /// Create a call-back to make a multibody behave like a car with perfect
    /// friction.
    /// @param _model The multibody to affect.
    void CreateCarlikeCallback(btMultiBody* const _model);

    /// Execute all call-backs for a given dynamics world.
    static void ExecuteCallbacks(btDynamicsWorld* _world, btScalar _timeStep);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Add an object to the world.
    /// @param _shapes The bullet collision shapes that reprsents the object
    ///                with or without multiple links
    /// @param _baseTransform The world transform for the base.
    /// @param _baseMass The mass for the base.
    /// @param _joints The list of connections between links.
    btMultiBody* AddObject(std::vector<btCollisionShape*>&& _shapes,
        btTransform&& _baseTransform, const double _baseMass,
        std::vector<std::shared_ptr<Connection>>&& _joints);

    /// Build a bullet collision shape from a pmpl MultiBody.
    /// @param _body The pmpl MultiBody.
    /// @return A bullet collision shape.
    std::vector<btCollisionShape*> BuildCollisionShape(MultiBody* _body);

    /// Build a bullet collision shape from a pmpl Body.
    /// @param _body The pmpl Body to use.
    /// @return A bullet triangle mesh collision shape.
    btTriangleMesh* BuildCollisionShape(const Body* _body);

    ///@}

};

#endif

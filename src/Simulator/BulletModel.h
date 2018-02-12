#ifndef PMPL_BULLET_MODEL_H_
#define PMPL_BULLET_MODEL_H_

#include <vector>

class btCollisionShape;
class btMultiBody;
class btMultiBodyConstraint;
class btMultiBodyDynamicsWorld;
class btMultiBodyLinkCollider;
class btTriangleMesh;

class Body;
class MultiBody;


////////////////////////////////////////////////////////////////////////////////
/// A structure for encompasing all the components which make up a bullet
/// multibody model. This is required because bullet's encapsulation and OOP
/// practices are worse than an average third-grader's.
////////////////////////////////////////////////////////////////////////////////
class BulletModel final {

  private:

    ///@name Internal State
    ///@{

    MultiBody* const m_pmplModel;      ///< The PMPL multibody.
    btMultiBody* const m_bulletModel;  ///< The bullet multibody.

    /// The dynamics world to which this is attached.
    btMultiBodyDynamicsWorld* m_world{nullptr};

    // The extra junk which should have been created and managed within the
    // bullet multibody.
    std::vector<btMultiBodyLinkCollider*> m_colliders;
    std::vector<btMultiBodyConstraint*>   m_constraints;
    std::vector<btCollisionShape*>        m_collisionShapes;

    bool m_debug{false};  ///< Show debug messages?

    ///@}
    ///@name Class Constants
    ///@{

    /// We will disable adjacent-link collision tests in the simulation.
    /// This is because we cannot perfectly convert angles between PMPL and
    /// Bullet, so it's possible that joints on tight-fitting parts would
    /// incur suprious collisions as small errors add up.
    static constexpr bool s_disableParentCollision{true};

    ///@}

  public:

    ///@name Construction
    ///@{

    BulletModel(MultiBody* const _mb);

    ~BulletModel();

    ///@}
    ///@name Initialization
    ///@{
    /// These functions create and destroy bullet components. There are no
    /// guarantees on what will happen if you call them on a model attached to a
    /// dynamics world. If the simulation is running, race conditions will
    /// ensue. If it is paused, there may be strange effects or seg faults upon
    /// resume. The best practice is to only call these when the model is not
    /// attached to a simulation.

    /// Create the bullet structures from the PMPL multibody.
    void Initialize();

    /// Release all bullet structures.
    /// @param _delete Delete the bullet model? Should only be true when called
    ///                by the destructor.
    void Uninitialize(const bool _delete = false);

    /// Destroy and rebuild all structures. This is intended for edit tools
    /// which need to update bullet models to match changes to the PMPL
    /// structures. Requires the model to already be part of a dynamics world.
    void Rebuild();

    ///@}
    ///@name Accessors
    ///@{

    MultiBody* GetPMPLMultiBody() noexcept;

    btMultiBody* GetBulletMultiBody() noexcept;

    ///@}
    ///@name Dynamics World Helpers
    ///@{

    void AddToDynamicsWorld(btMultiBodyDynamicsWorld* const _world);

    void RemoveFromDynamicsWorld();

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Build the bullet structures from the PMPL multibody.
    /// @TODO This is a very large function. Break it up into smaller pieces for
    ///       adding the base and links.
    void Build();

    /// Build a set of bullet collision shapes for a pmpl MultiBody.
    /// @param _body The pmpl MultiBody.
    /// @return A set of bullet collision shapes.
    std::vector<btCollisionShape*> BuildCollisionShapes(
        const MultiBody* const _body);

    /// Build a bullet collision shape for a pmpl Body.
    /// @param _body The pmpl Body to use.
    /// @return A bullet collision shape.
    btCollisionShape* BuildCollisionShape(const Body* const _body);

    ///@}
};

#endif

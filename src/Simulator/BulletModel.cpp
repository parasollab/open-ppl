#include "BulletModel.h"

#include "Conversions.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/GMSPolyhedron.h"
#include "MPProblem/Robot/DynamicsModel.h"

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "ConvexDecomposition/cd_wavefront.h"

#include "nonstd/io.h"

#include <algorithm>

#define USE_BULLET_COLLIDERS // Comment this out to disable bullet collision.


/*------------------------------- Construction -------------------------------*/

BulletModel::
BulletModel(MultiBody* const _mb) : m_pmplModel(_mb),
    m_bulletModel(new btMultiBody(0, 0, btVector3(), false, false)) {
  // Uninitialize first to destruct and clear the dummy m_bulletModel.
#ifdef DEBUG_BULLET_PROBLEMS
  _mb->m_bullet = m_bulletModel;
#endif
  Uninitialize();
  Initialize();
}


BulletModel::
~BulletModel() {
  Uninitialize(true);
}

/*------------------------------ Initialization ------------------------------*/

void
BulletModel::
Initialize() {
  // Store active model's current configuration and zero before rebuilding.
  std::vector<double> zeros(m_pmplModel->DOF(), 0),
                            currentCfg;
  if(m_pmplModel->IsActive()) {
    currentCfg = m_pmplModel->GetCurrentCfg();
    m_pmplModel->Configure(zeros);
    std::cout << "Initializing from " << currentCfg << std::endl;
  }

  Build();

  if(m_pmplModel->IsActive()) {
    m_pmplModel->Configure(currentCfg);
    ConfigureSimulatedPosition(m_pmplModel, m_bulletModel);
  }

  // Zero the model's velocities.
  m_bulletModel->setBaseVel({0,0,0});
  m_bulletModel->setBaseOmega({0,0,0});
}


void
BulletModel::
Uninitialize(const bool _delete) {
  if(m_world)
    RemoveFromDynamicsWorld();

  for(auto collider : m_colliders)
    delete collider;
  for(auto constraint : m_constraints)
    delete constraint;
  for(auto collision : m_collisionShapes)
    delete collision;

  m_colliders.clear();
  m_constraints.clear();
  m_collisionShapes.clear();

  // We must manually destruct the model rather than deleting it to avoid having
  // it move around in memory, which will disrupt the Robot objects. Overwrite
  // the space with zeros to ensure no stale data is left over.
  if(!_delete) {
    m_bulletModel->~btMultiBody();
    std::fill_n((char*)m_bulletModel, sizeof(btMultiBody), (char)0);
  }
  else
    delete m_bulletModel;
}


void
BulletModel::
Rebuild() {
  auto world = m_world;
  Uninitialize();
  Initialize();
  if(world)
    AddToDynamicsWorld(world);
}

/*-------------------------------- Accessors ---------------------------------*/

MultiBody*
BulletModel::
GetPMPLMultiBody() noexcept {
  return m_pmplModel;
}


btMultiBody*
BulletModel::
GetBulletMultiBody() noexcept {
  return m_bulletModel;
}

/*------------------------------ Dynamics World ------------------------------*/

void
BulletModel::
AddToDynamicsWorld(btMultiBodyDynamicsWorld* const _world) {
  if(m_world)
    throw RunTimeException(WHERE, "Cannot attach a model to more than one "
        "dynamics world.");
  m_world = _world;

  // Determine the collision filter group and mask.
  // See BulletCollision/BroadphaseCollision/btBroadphaseProxy.h
  // and BulletCollision/CollisionDispatch/btCollisionWorld.h
  // for more info on these.
  auto group = m_pmplModel->IsActive()
      ? btBroadphaseProxy::DefaultFilter
      : btBroadphaseProxy::StaticFilter;
  auto mask = m_pmplModel->IsActive()
      ? btBroadphaseProxy::AllFilter
      : btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter;
        // This is logically equivalent to "All BUT StaticFilter":

  // Add the multibody.
  m_world->addMultiBody(m_bulletModel);

  // Add the colliders.
  for(auto* collider : m_colliders)
    m_world->addCollisionObject(collider, group, mask);

  // Add the constraints.
  for(auto* constraint : m_constraints)
    m_world->addMultiBodyConstraint(constraint);

  /// @TODO Need to set friction properly.
  //col->setFriction(m_problem->GetEnvironment()->GetFrictionCoefficient());
}


void
BulletModel::
RemoveFromDynamicsWorld() {
  if(!m_world)
    throw RunTimeException(WHERE, "Cannot remove a model which is not in a "
        "dynamics world.");

  // Remove the multibody.
  m_world->removeMultiBody(m_bulletModel);

  // Remove the colliders.
  for(auto* collider : m_colliders)
    m_world->removeCollisionObject(collider);

  // Remove the constraints.
  for(auto* constraint : m_constraints)
    m_world->removeMultiBodyConstraint(constraint);

  m_world = nullptr;
}

/*------------------------------ Helpers -------------------------------------*/

void
BulletModel::
Build() {
  // Create collision shapes for each body in this multibody.
  m_collisionShapes = BuildCollisionShapes(m_pmplModel);

  const bool  isDynamic = m_pmplModel->IsActive();
  const auto& joints    = m_pmplModel->GetJoints();

  if(m_debug)
    std::cout << "BulletModel::AddObject: Creating multibody"
              << "\n\t" << m_collisionShapes.size() << " bodies"
              << "\n\t" << joints.size() << " joints"
              << "\n\t" << (isDynamic ? "" : "not ") << "dynamic"
              << std::endl;

  // First check that number of elements of each vector match. We need one less
  // joint as the base is not considered a joint.
  if(m_collisionShapes.size() - 1 != joints.size())
    throw RunTimeException(WHERE, "Expected the number of shapes (" +
        std::to_string(m_collisionShapes.size()) + ") to be equal to the " +
        "number of joints plus 1 (" + std::to_string(joints.size() + 1) + ").");

  // Create the base and btMultiBody object.
  {
    // Compute inertia for base.
    const double baseMass = m_pmplModel->IsActive()
                          ? m_pmplModel->GetBase()->GetMass() : 0;
    btVector3 baseInertia(0, 0, 0);
    if(isDynamic)
      m_collisionShapes[0]->calculateLocalInertia(baseMass, baseInertia);

    // Compute other properties that apply to the whole multibody object.
    const int  numLinks  = m_collisionShapes.size() - 1;
    const bool fixedBase = !isDynamic;
    const bool canSleep  = false; /// @TODO Not sure what this means.

    // Make the btMultiBody in the exact same address with placement new.
    new(m_bulletModel) btMultiBody(numLinks, baseMass, baseInertia, fixedBase,
        canSleep);
    btTransform baseTransform = ToBullet(
        m_pmplModel->GetBase()->GetWorldTransformation());
    m_bulletModel->setBaseWorldTransform(baseTransform);

#ifdef USE_BULLET_COLLIDERS
    // The multibody on its own doesn't process collisions. Create a collider
    // object for each component to handle this. The base has id -1.
    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(m_bulletModel, -1);
    col->setCollisionShape(m_collisionShapes[0]);
    col->setWorldTransform(baseTransform);
    m_bulletModel->setBaseCollider(col);

    // Attach the collider to this model.
    m_colliders.push_back(col);
#endif

    if(m_debug)
      std::cout << "\tAdded base with mass " << baseMass << "."
                << std::endl;
  }

  // Create the links.
  for(size_t i = 0; i < joints.size(); ++i) {
    // Get the indices for this link and its parent in both PMPL and bullet
    // representation (bullet uses [0:n-1] where PMPL uses [1:n]).
    const int parentPmplIndex = joints[i]->GetPreviousBodyIndex(),
              linkPmplIndex   = joints[i]->GetNextBodyIndex(),
              parentIndex     = parentPmplIndex - 1,
              linkIndex       = linkPmplIndex - 1;

    if(m_debug)
      std::cout << "\tAdding joint " << i << " connecting bodies "
                << parentPmplIndex << " and " << linkPmplIndex << "."
                << std::endl;

    // Get the mass and moment of inertia for the link:
    /// @TODO Respect PMPL's moment instead of recomputing it here.
    /// @TODO Specify center of mass?
    const btScalar linkMass = joints[i]->GetNextBody()->GetMass();
    btVector3 linkInertia(0, 0, 0);
    if(isDynamic)
      m_collisionShapes[linkPmplIndex]->calculateLocalInertia(linkMass,
                                                              linkInertia);

    // Set up the connection between this link and its parent.

    // Get the joint transformations.
    Transformation parentToActuation = joints[i]->GetTransformationToDHFrame(),
                   actuation = joints[i]->GetDHParameters().GetTransformation(),
                   actuationToLink = joints[i]->GetTransformationToBody2(),
                   parentToLink = parentToActuation * actuation * actuationToLink;

    // Get the parent to link rotation for the 0 configuration, in the PARENT
    // frame.
    btQuaternion parentToLinkRotationInParentFrame;
    {
      mathtool::Quaternion temp;
      mathtool::convertFromMatrix(temp, parentToLink.rotation().matrix());
      parentToLinkRotationInParentFrame = ToBullet(temp);
    }

    // Get the parent to actuation frame translation, in the PARENT frame.
    btVector3 parentToActuationTranslationInParentFrame = ToBullet(
        parentToActuation.translation());

    // Get the actuation to link frame translation, in the LINK frame.
    btVector3 actuationToLinkTranslationInLinkFrame = ToBullet(
        -actuationToLink.rotation() * actuationToLink.translation());

    switch(joints[i]->GetConnectionType())
    {
      case Connection::JointType::Revolute:
      {
        // For a revolute link, the last piece now is to set the jointAxis in
        // the link's frame. For revolute joints this is the Z-direction in the
        // actuation frame, so we just need to rotate it to the link frame.
        const btVector3 jointAxisInLinkFrame = ToBullet(
            actuationToLink.rotation() * Vector3d(0, 0, 1));

        m_bulletModel->setupRevolute(linkIndex, linkMass, linkInertia,
            parentIndex,
            parentToLinkRotationInParentFrame,
            jointAxisInLinkFrame,
            parentToActuationTranslationInParentFrame,
            actuationToLinkTranslationInLinkFrame,
            s_disableParentCollision);

        // Add joint limits as a bullet constraint.
        const Range<double> range = joints[i]->GetJointRange(0);
        btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(
            m_bulletModel, linkIndex, range.min * PI, range.max * PI);
        m_constraints.push_back(con);

        if(m_debug)
          std::cout << "\t\tAdded revolute joint " << linkIndex
                    << "with joint range ["
                    << std::setprecision(3) << range.min * PI << " : "
                    << std::setprecision(3) << range.max * PI << "]."
                    << std::endl;
        break;
      }
      case Connection::JointType::Spherical:
      {
        /// @TODO support spherical constraints. As per answers here:
        /// http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=10780
        /// Until then, warn the user that their constraints won't be respected.
        std::cerr << "Bullet supports spherical joints, but not constraints for "
                  << "them. Any constraints on this joint will be ignored by the "
                  << "physics engine!"
                  << std::endl;

        m_bulletModel->setupSpherical(linkIndex, linkMass, linkInertia,
            parentIndex,
            parentToLinkRotationInParentFrame,
            parentToActuationTranslationInParentFrame,
            actuationToLinkTranslationInLinkFrame,
            s_disableParentCollision);

        if(m_debug)
          std::cout << "\t\tAdded spherical joint " << linkIndex << "."
                    << std::endl;
        break;
      }
      case Connection::JointType::NonActuated:
      {
        // Since this is a fixed joint, there is no need for joint constraints.
        m_bulletModel->setupFixed(linkIndex, linkMass, linkInertia, parentIndex,
            parentToLinkRotationInParentFrame,
            parentToActuationTranslationInParentFrame,
            actuationToLinkTranslationInLinkFrame,
            s_disableParentCollision);

        if(m_debug)
          std::cout << "\t\tAdded fixed joint " << linkIndex << "."
                    << std::endl;
        break;
      }
      default:
        throw RunTimeException(WHERE, "Unsupported joint type.");
    }

#ifdef USE_BULLET_COLLIDERS
    // Create a collider for the link.
    // See examples/MultiBody/Pendulum.cpp for related example code.
    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(m_bulletModel,
        linkIndex);
    col->setCollisionShape(m_collisionShapes[linkIndex + 1]);
    m_bulletModel->getLink(linkIndex).m_collider = col;

    m_colliders.push_back(col);
#endif
  }

  // Finalize the multibody.
  m_bulletModel->finalizeMultiDof();

  // Also from Pendulum.cpp:
  // Initialize the internal link transforms.
  btAlignedObjectArray<btQuaternion> scratch_q;
  btAlignedObjectArray<btVector3> scratch_m;
  m_bulletModel->forwardKinematics(scratch_q, scratch_m);

  // Initialize the internal collider transforms.
  btAlignedObjectArray<btQuaternion> world_to_local;
  btAlignedObjectArray<btVector3> local_origin;
  m_bulletModel->updateCollisionObjectWorldTransforms(world_to_local,
      local_origin);
}


std::vector<btCollisionShape*>
BulletModel::
BuildCollisionShapes(const MultiBody* const _multibody) {
  std::vector<btCollisionShape*> shapes;

  // Make collision shapes for each body.
  for(size_t i = 0; i < _multibody->GetNumBodies(); i++)
    shapes.push_back(BuildCollisionShape(_multibody->GetBody(i)));

  return shapes;
}


btCollisionShape*
BulletModel::
BuildCollisionShape(const Body* const _body) {
  // Get the body's polyhedron, vertices, and facets.
  const GMSPolyhedron& poly = _body->GetPolyhedron();
  const auto& vertices = poly.GetVertexList();
  const auto& facets = poly.GetPolygonList();

  // Initialize a btTriangleMesh with enough space for our model.
  btTriangleMesh* mesh = new btTriangleMesh();
  mesh->preallocateVertices(vertices.size());
  mesh->preallocateIndices(facets.size());

  // Add vertices, don't remove duplicates (because PMPL doesn't, and this will
  // mess up the facet indexes).
  for(const auto& v : vertices)
    mesh->findOrAddVertex(btVector3(v[0], v[1], v[2]), false);

  // Add facets.
  for(const auto& f : facets)
    mesh->addTriangleIndices(f[0], f[1], f[2]);

  // Make the collision shape from the mesh.
  auto shape = new btGImpactMeshShape(mesh);
  shape->updateBound();
  shape->setMargin(0); // Do not use any collision 'margin' around this obstacle.

  return shape;
}

/*----------------------------------------------------------------------------*/

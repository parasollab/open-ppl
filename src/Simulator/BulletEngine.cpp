#include "BulletEngine.h"

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"

#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "ConvexDecomposition/cd_wavefront.h"

#include "nonstd/runtime.h"

#include "Conversions.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/FixedBody.h"
#include "Geometry/Bodies/FreeBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"
#include "Geometry/GMSPolyhedron.h"

#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"

/*--------------------------- Static Initialization --------------------------*/

std::map<btDynamicsWorld*, BulletEngine::CallbackSet&>
BulletEngine::s_callbackMap;

/*------------------------------ Construction --------------------------------*/

BulletEngine::
BulletEngine(MPProblem* const _problem) : m_problem(_problem) {
  // Create the bullet objects needed for a dynamic rigid body simulation.
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_broadphase = new btDbvtBroadphase();
  m_solver = new btMultiBodyConstraintSolver();

  m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,
      m_broadphase, m_solver, m_collisionConfiguration);

  // This is needed to get gimpact shapes to respond to collisions.
  btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

  // Set the gravity in our world, based off of MPProblem:
  m_dynamicsWorld->setGravity(ToBullet(
                              m_problem->GetEnvironment()->GetGravity()));

  // Add this engine's call-back set to the call-back map.
  s_callbackMap.emplace(m_dynamicsWorld, m_callbacks);
  m_dynamicsWorld->setInternalTickCallback(ExecuteCallbacks);
}


BulletEngine::
~BulletEngine() {
  // Delete the rigid bodies in the bullet dynamics world.
  for(int i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
    btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);

    // If the body has a motion state, delete that too.
    if(body && body->getMotionState())
      delete body->getMotionState();

    m_dynamicsWorld->removeCollisionObject(obj);
    delete obj;
  }

  // Collision shapes aren't deleted with their associated rigid bodies because
  // multiple rigid bodies might share a collision shape. Delete them explicitly.
  for(int i = 0; i < m_collisionShapes.size(); ++i)
    delete m_collisionShapes[i];
  m_collisionShapes.clear();

  // Remove this engine's dynamics world and call-back set from the call-back
  // map.
  s_callbackMap.erase(m_dynamicsWorld);

  // Delete the remaining bullet objects.
  delete m_dynamicsWorld;
  delete m_solver;
  delete m_broadphase;
  delete m_dispatcher;
  delete m_collisionConfiguration;
}

/*---------------------------- Simulation Interface --------------------------*/

void
BulletEngine::
Step(const btScalar _timestep, const int _maxSubSteps,
    const btScalar _resolution) {
  // This doesn't seem to be required in the examples, testing without it for
  // now.
  //m_dynamicsWorld->updateAabbs();
  //m_dynamicsWorld->computeOverlappingPairs();
  m_dynamicsWorld->stepSimulation(_timestep, _maxSubSteps, _resolution);
}

/*----------------------------- Transform Access -----------------------------*/

glutils::transform
BulletEngine::
GetObjectTransform(const size_t _i, const size_t _j) const {
  // Check for out-of-range access.
  nonstd::assert_msg(_i < size_t(m_dynamicsWorld->getNumMultibodies()),
      "BulletEngine error: requested transform for object " + std::to_string(_i)
      + ", but there are only " +
      std::to_string(m_dynamicsWorld->getNumMultibodies()) +
      " objects in the simulation.");

  glutils::transform t;
  btMultiBody* mb = m_dynamicsWorld->getMultiBody(_i);
  if(_j == 0)
    mb->getBaseWorldTransform().getOpenGLMatrix(t.data());
  else
    mb->getLink(int(_j-1)).m_cachedWorldTransform.getOpenGLMatrix(t.data());

  return t;
}

/*----------------------------- Modifiers ------------------------------------*/

btMultiBody*
BulletEngine::
AddObject(MultiBody* _m) {
  // Static and active bodies must be handled slightly differently. Try first
  // for downcast to static body.
  StaticMultiBody* sbody = dynamic_cast<StaticMultiBody*>(_m);
  if(sbody) {
    // This is a static body.
    // For now, assume that static multibodies have no links. We should expand
    // this in the future though to support fixed joints.
    auto base = sbody->GetFixedBody(0);

    return AddObject(BuildCollisionShape(_m),
        ToBullet(base->GetWorldTransformation()), 0, {});
  }
  else {
    // This is an active body.
    ActiveMultiBody* abody = dynamic_cast<ActiveMultiBody*>(_m);
    auto base = abody->GetFreeBody(0);

    // Collect joint connection objects.
    std::vector<std::shared_ptr<Connection>> joints;
    std::copy(abody->joints_begin(), abody->joints_end(),
        std::back_inserter(joints));

    return AddObject(BuildCollisionShape(_m),
        ToBullet(base->GetWorldTransformation()), base->GetMass(),
        std::move(joints));
  }
}


btMultiBody*
BulletEngine::
AddObject(btCollisionShape* _shape, btTransform _transform, const double _mass) {
  return AddObject({_shape}, std::move(_transform), _mass, {});
}


btMultiBody*
BulletEngine::
AddObject(std::vector<btCollisionShape*>&& _shapes, btTransform&& _baseTransform,
    const double _baseMass, std::vector<std::shared_ptr<Connection>>&& _joints) {
  if(m_debug)
    std::cout << "BulletEngine.cpp: adding object with "
              << _shapes.size() << " components and "
              << _joints.size() << " joints." << std::endl;

  // First check that number of elements of each vector match. We need one less
  // joint as the base is not considered a joint.
  if(_shapes.size() - 1 != _joints.size())
    throw RunTimeException(WHERE, "Expected the number of shapes (" +
        std::to_string(_shapes.size()) + ") to be equal to the number of "
        "joints (" + std::to_string(_joints.size()) + ") plus 1.");

  // Add the shapes to a list of shapes to be deleted later.
  for(auto shape : _shapes)
    m_collisionShapes.push_back(shape);

  // Compute the dynamics properties of this object. It is dynamic if the base
  // has non-zero mass.
  const bool isDynamic = _baseMass != 0.;

  if(m_debug)
    std::cout << "\tObject is " << (isDynamic ? "" : "not ") << "dynamic."
              << std::endl;

  // See BulletCollision/BroadphaseCollision/btBroadphaseProxy.h
  // and BulletCollision/CollisionDispatch/btCollisionWorld.h
  // for more info on these.
  auto collisionFilterGroup = isDynamic ? btBroadphaseProxy::DefaultFilter
                                        : btBroadphaseProxy::StaticFilter;
  auto collisionFilterMask = isDynamic ? btBroadphaseProxy::AllFilter
      : btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter;
        // This is logically equivalent to "All BUT StaticFilter":

  // Create the base and overall btMultiBody object.
  btMultiBody* mb;
  {
    // Compute inertia for base.
    btVector3 inertia(0, 0, 0);
    if(isDynamic)
      _shapes[0]->calculateLocalInertia(_baseMass, inertia);

    // Compute other properties that apply to the whole multibody object.
    const int links = _shapes.size() - 1;
    const bool fixedBase = !isDynamic;
    const bool canSleep = false; // Can this object sleep? Not sure what it means.

    // Make multibody.
    mb = new btMultiBody(links, _baseMass, inertia, fixedBase, canSleep);
    mb->setBaseWorldTransform(_baseTransform);

    // The multibody on its own doesn't process collisions. Create a collider
    // object for each component to handle this. The base has id -1.
    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(mb, -1);
    col->setCollisionShape(_shapes[0]);
    col->setWorldTransform(_baseTransform);
    col->setFriction(m_problem->GetEnvironment()->GetFrictionCoefficient());

    // Attach the collider to the dynamics world and multibody.
    m_dynamicsWorld->addCollisionObject(col, collisionFilterGroup,
        collisionFilterMask);
    mb->setBaseCollider(col);

    if(m_debug)
      std::cout << "\tAdded base with mass " << _baseMass << "."
                << std::endl;
  }

  // Create the links.
  for(size_t i = 0; i < _joints.size(); i++) {
    if(m_debug)
      std::cout << "\tAdding joint " << i << "..." << std::endl;

    // Get the PMPL link.
    /// @TODO This assumes that the PMPL body is an ActiveMultiBody. Generalize
    ///       our bodies so that there is no such artificial distinction.
    FreeBody* const link = _joints[i]->GetNextBody();

    // Get the PMPL indexes for this link.
    const int parentPmplIndex = _joints[i]->GetPreviousBodyIndex();
    const int linkPmplIndex = _joints[i]->GetNextBodyIndex();

    // The indices according to how Bullet stores the links (from 0 to
    // num_links - 1).
    const int parentIndex = parentPmplIndex - 1;
    const int linkIndex = linkPmplIndex - 1;

    // Get the mass for the link:
    const btScalar mass = link->GetMass();

    btVector3 inertia(0, 0, 0);
    if(isDynamic)
      _shapes[linkPmplIndex]->calculateLocalInertia(mass, inertia);

    // Get the transformations between the parent frame, the joint frame, and the
    // link frame.
    Transformation parentToJoint = _joints[i]->GetTransformationToDHFrame();
    Transformation jointActuation = _joints[i]->GetDHParameters().
        GetTransformation();
    Transformation jointToLink = _joints[i]->GetTransformationToBody2();

    // Compute the transformations from (parent to joint actuation) and
    // (parent to link) frames.
    Transformation parentToJointActuation = parentToJoint * jointActuation;
    Transformation parentToLink = parentToJointActuation * jointToLink;

    // Compute the rotation from parent frame to link frame in bullet
    // quaternion representation.
    btQuaternion parentToLinkRotation;
    {
      Quaternion temp;
      mathtool::convertFromMatrix(temp, parentToLink.rotation().matrix());
      parentToLinkRotation = ToBullet(temp);
    }

    // Compute the translations from (parent to joint) and (joint to link).
    btVector3 parentToJointTranslation = ToBullet(parentToJoint.translation());
    btVector3 jointToLinkTranslation   = ToBullet(jointToLink.translation());

    // It looks like this should be true (disabled) if "the self-collision
    // has conflicting/non-resolvable contact normals".
    const bool disableParentCollision = true;

    // Set up the connection between this link and its parent based on the joint
    // type.
    switch(_joints[i]->GetConnectionType()) {
      case Connection::JointType::Revolute:
      {
        // Compute the joint's axis of rotation.
        const Transformation& parent = link->GetWorldTransformation();
        btVector3 jointAxis = ToBullet(
            (parent * parentToJointActuation) * Vector3d(0, 0, 1));

        mb->setupRevolute(linkIndex, mass, inertia, parentIndex,
               parentToLinkRotation, jointAxis, parentToJointTranslation,
               jointToLinkTranslation, disableParentCollision);

        // Add joint limits as a bullet constraint.
        const pair<double, double> limits = _joints[i]->GetJointLimits(0);
        btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(mb,
            linkIndex, limits.first * PI, limits.second * PI);
        m_dynamicsWorld->addMultiBodyConstraint(con);

        if(m_debug)
          std::cout << "\t\tAdded revolute joint " << linkIndex
                    << "with range of motion ["
                    << std::setprecision(3) << limits.first * PI << " : "
                    << std::setprecision(3) << limits.second * PI << "]."
                    << std::endl;
        break;
      }
      case Connection::JointType::Spherical:
      {
        /// @TODO support spherical constraints. As per answers here:
        /// http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=10780
        /// Until then, warn the user that their constraints won't be respected.
        std::cerr << "Bullet supports spherical joints, but not constraints for "
                  << "them. Any constraints on this joint will be ignored!"
                  << std::endl;

        mb->setupSpherical(linkIndex, mass, inertia, parentIndex,
              parentToLinkRotation, parentToJointTranslation,
              jointToLinkTranslation, disableParentCollision);

        if(m_debug)
          std::cout << "\t\tAdded spherical joint " << linkIndex << "."
                    << std::endl;
        break;
      }
      case Connection::JointType::NonActuated:
      {
        // Since this is a fixed joint, there is no need for joint constraints.
        mb->setupFixed(linkIndex, mass, inertia, parentIndex,
            parentToLinkRotation, parentToJointTranslation,
            jointToLinkTranslation, disableParentCollision);

        if(m_debug)
          std::cout << "\t\tAdded fixed joint " << linkIndex << "."
                    << std::endl;
        break;
      }
      default:
        throw RunTimeException(WHERE, "Unsupported joint type.");
    }
  }

  // Finalize the multibody.
  mb->finalizeMultiDof();
  mb->setBaseVel({0, 0, 0}); // Must happen after finalizeMultiDof().
  m_dynamicsWorld->addMultiBody(mb);

  // Create a collider for each joint.
  // See examples/MultiBody/Pendulum.cpp for related example code.
  for(int i = 0; i < mb->getNumLinks(); i++) {
    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(mb, i);
    col->setCollisionShape(_shapes[i + 1]);
    col->setFriction(m_problem->GetEnvironment()->GetFrictionCoefficient());

    m_dynamicsWorld->addCollisionObject(col, collisionFilterGroup,
        collisionFilterMask);

    mb->getLink(i).m_collider = col;
  }

  // Also from Pendulum.cpp:
  btAlignedObjectArray<btQuaternion> scratch_q;
  btAlignedObjectArray<btVector3> scratch_m;
  mb->forwardKinematics(scratch_q, scratch_m);
  btAlignedObjectArray<btQuaternion> world_to_local;
  btAlignedObjectArray<btVector3> local_origin;
  mb->updateCollisionObjectWorldTransforms(world_to_local, local_origin);

  return mb;
}


void
BulletEngine::
SetGravity(const btVector3& _gravityVec) {
  m_dynamicsWorld->setGravity(_gravityVec);
}

/*--------------------------- Callback Functions -----------------------------*/

void
BulletEngine::
CreateCarlikeCallback(btMultiBody* const _model) {
  // Create a call-back function to make _model appear car-like. For now we will
  // always assume that _model's forward direction is (1, 0, 0) in its local
  // frame.
  CallbackFunction f = [_model]() {
    const btVector3 velocity = _model->getBaseVel();
    const btVector3 heading = _model->localDirToWorld(-1, {1, 0, 0});
    const btScalar sign = velocity * heading < 0 ? -1 : 1;
    _model->setBaseVel(heading * sign * velocity.length());
  };

  // Add this to the set of callbacks for this simulation engine.
  m_callbacks.push_back(f);
}


void
BulletEngine::
ExecuteCallbacks(btDynamicsWorld* _world, btScalar _timeStep) {
  // Get the call-back set associated with this dynamics world.
  CallbackSet& callbacks = s_callbackMap.at(_world);

  // Execute each call-back in the set.
  for(auto& callback : callbacks)
    callback();
}

/*------------------------------ Helpers -------------------------------------*/

vector<btCollisionShape*>
BulletEngine::
BuildCollisionShape(MultiBody* _multibody) {
  std::vector<btCollisionShape*> shapes;

  // Make simulator collision shape(s).
  for(size_t i = 0; i < _multibody->GetNumBodies(); i++) {
    const Body* body = _multibody->GetBody(i);
    btTriangleMesh* shapeMesh = BuildCollisionShape(body);

    //Make the bullet impact shape:
    auto ptr = new btGImpactMeshShape(shapeMesh);
    ptr->updateBound();
    shapes.push_back(ptr);
  }

  return shapes;
}


btTriangleMesh*
BulletEngine::
BuildCollisionShape(const Body* _body) {
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

  return mesh;
}

/*----------------------------------------------------------------------------*/

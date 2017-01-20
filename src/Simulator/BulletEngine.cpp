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
#include "Geometry/Bodies/FixedBody.h"
#include "Geometry/Bodies/FreeBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"


/*------------------------------ Construction --------------------------------*/

BulletEngine::
BulletEngine() {
  // Create the bullet objects needed for a dynamic rigid body simulation.
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_broadphase = new btDbvtBroadphase();
  m_solver = new btMultiBodyConstraintSolver();

  m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,
      m_broadphase, m_solver, m_collisionConfiguration);

  // This is needed to get gimpact shapes to respond to collisions.
  btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

  // Set the gravity in our world.
  m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
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
  // Get base transform and mass.
  std::vector< double > masses;

  //The btTransform holds data for the basis and origin of the body, so it
  //defines where the object C.O.M. is and what its rotation is at that point.
  std::vector< btTransform > transformations;
  std::vector< std::shared_ptr< Connection > > joints;
  //std::vector< btVector3 > coms; //Center of mass vectors

  //Note that shape definition stuff is handled in the BuildCollisionShape()
  //function, using btMesh type stuff

  //Tim's Temp Note: I probably have to first loop through all bodies in here
  //and get the Bullet versions of world transforms for each body.
  //Then I'll also be handling all of them inside of BuildCollisionShape,
  //and everything should correlate in the end (just as it does now for the
  //first element)

  StaticMultiBody* sbody = dynamic_cast<StaticMultiBody*>(_m);
  if(sbody) {//So it's a static body:
    if(m_debug)
      std::cout << "Working on a static body" << std::endl;
    auto body = sbody->GetFixedBody(0);
    transformations.push_back(ToBullet(body->GetWorldTransformation()));
    masses.push_back(0);
  }
  else {
    ActiveMultiBody* abody = dynamic_cast<ActiveMultiBody*>(_m);
    if(m_debug)
      std::cout << "Working on an active body with number of free bodies = "
                << abody->NumFreeBody() << std::endl;

    // Collect the transformations for each body in the multibody.
    for(size_t i = 0; i < abody->NumFreeBody(); i++) {
      FreeBody* body = abody->GetFreeBody(i);
      transformations.push_back(ToBullet(body->GetWorldTransformation()));
      //coms.push_back(ToBullet(body->GetCenterOfMass()));
      masses.push_back(1); ///< @TODO change the mass to appropriate amount

      if(m_debug)
        std::cout << "\tParsed body " << i << " with mass " << 1
                  << "and world transform " << body->GetWorldTransformation()
                  << std::endl;
    }

    //Next, go through the joints in this active body:
    if(m_debug)
      std::cout << "BulletEngine: about to loop through " << abody->NumJoints()
                << " joints" << std::endl;
    for(auto jointIt = abody->joints_begin();
          jointIt < abody->joints_end(); jointIt++){
      //the auto in this context is a
      //vector< std::shared_ptr<Connection> >::const_iterator
      //Or in other words, an iterator for a vector of pointers to Connections
      joints.push_back(*jointIt);//Dereference iterator and push back in joints
      if(m_debug)
        std::cout << "Pushed back joint: " << **jointIt << std::endl;
    }
  }

  //form each base/link's collision shape for Bullet:
  if(m_debug)
    std::cout << "About to enter BuildCollisionShape" << std::endl;
  std::vector<btCollisionShape*> shapes = BuildCollisionShape(_m);

  return AddObject(shapes, transformations, masses, joints);
}

btMultiBody*
BulletEngine::
AddObject(btCollisionShape* _shape, btTransform _transform, double _mass) {
  if(m_debug)
    std::cout << "BulletEngine: Entering using the old version of AddObject"
              << std::endl;

  //this is basically just a wrapper overload to allow for
  //a more simplified AddObject call, so make objects into vectors and such:
  std::vector< btCollisionShape* > shapes = {_shape};
  std::vector< btTransform > transforms = {_transform};
  std::vector< double > masses = {_mass};

  return AddObject(shapes, transforms, masses);
}


btMultiBody*
BulletEngine::
AddObject(std::vector<btCollisionShape*> _shapes,
          std::vector<btTransform> _transforms,
          std::vector<double> _masses,
          std::vector<std::shared_ptr<Connection>> _joints) {
  if(m_debug)
    std::cout << "BulletEngine.cpp: entering final AddObject overload with "
              << _joints.size() << " joints" << std::endl;

  // This function is assuming the base is in the first element of the 3 args
  // First check that number of elements of each vector match:
  if(_shapes.size() != _transforms.size()
      || _shapes.size() != _masses.size()
      || _shapes.size() - 1 != _joints.size()) // 1 more shape than joint
    throw RunTimeException(WHERE, "_shapes size must equal size of _transforms,"
        " _masses, and _joints + 1");

  // Add the shapes to a list of shapes to be deleted later.
  for(size_t i = 0; i < _shapes.size(); i++)
    m_collisionShapes.push_back(_shapes.at(i));

  //First Step: Handle the base and overall btMultiBody object.
  // Compute inertia for base.
  const double baseMass = _masses.at(0);
  btVector3 inertia(0, 0, 0);
  const bool isDynamic(baseMass != 0.);
  if(isDynamic) // Then calculate the moment of inertia if dynamic
    _shapes.at(0)->calculateLocalInertia(baseMass, inertia);

  // Make multi body.
  const int links = _shapes.size() - 1;
  const bool isFixedBase = !isDynamic;
  const bool canSleep = false; // Can this object sleep? Not sure what it means.
  btMultiBody* mb = new btMultiBody(links, baseMass, inertia,
                                    isFixedBase, canSleep);

  mb->setBaseWorldTransform(_transforms.at(0));

  if(m_debug)
    std::cout << "BulletEngine.cpp: Made a btMultiBody with number of joints/"
              << "links = " << mb->getNumLinks() << " and number of shapes "
              << "(links + base) = " << _shapes.size() << std::endl;

  //Adapted from Bullet example code TestJointTorqueSetup.cpp lines 297-298:
  short collisionFilterGroup, collisionFilterMask;
  // Noting from BulletCollision/BroadphaseCollision/btBroadphaseProxy.h,
  // the second arg is selecting the collisionFilterGroup ("StaticFilter" = 2)
  // the third arg is selecting the collisionFilterMask ("DefaultFilter" = 1)
  // Also see BulletCollision/CollisionDispatch/btCollisionWorld.h for info
  //  collisionFilterGroup = 2; // static filter
  //  collisionFilterMask = 1 + 2; // default and static filter
  //Doing some light verification, these worked as well as the hardcoded values:
  if(isDynamic) {
    collisionFilterGroup = short(btBroadphaseProxy::DefaultFilter);
    collisionFilterMask = short(btBroadphaseProxy::AllFilter);
  } else {
    collisionFilterGroup = short(btBroadphaseProxy::StaticFilter);
    //This is logically equivalent to "All BUT StaticFilter":
    collisionFilterMask = short(btBroadphaseProxy::AllFilter
                                ^ btBroadphaseProxy::StaticFilter);
  }

  // The multibody on its own doesn't process collisions. Add a collider for
  // each link. The base collider has link id -1.
  btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(mb, -1);
  col->setCollisionShape(_shapes.at(0));
  col->setWorldTransform(_transforms.at(0));

  m_dynamicsWorld->addCollisionObject(col,
                                collisionFilterGroup, collisionFilterMask);
  mb->setBaseCollider(col);

  //NOTE: from btMultiBody.cpp, m_links is described as:
  // array of m_links, excluding the base. index from 0 to num_links-1.
  //This means that the base in Bullet is stored separately from any links,
  // and so all of the indexing will reflect that.
  //This really means that PMPL link index is one off of the Bullet link index.

  //Note a pretty big assumption, but one that is safe for now: in PMPL, joint
  // number is always the same as the parent's link number.
  // in Bullet, the joint number is always the CHILD's link number.

  for(size_t i = 0; i < _joints.size(); i++) {
    if(m_debug)
      std::cout << "BulletEngine.cpp: in AddObject working on joint " << i
                << std::endl;

    //The data that applies to all links:
    int parentPmplIndex = _joints.at(i)->GetPreviousBodyIndex();
    int linkPmplIndex = _joints.at(i)->GetNextBodyIndex();
    FreeBody* linkBody = _joints.at(i)->GetNextBody();

    //The indices according to how Bullet stores the links:
    int linkIndex = linkPmplIndex - 1;// should be >= 0 (should be equal to i)
    int parentIndex = parentPmplIndex - 1;// >= -1 index is perfectly valid here

    //Get the mass for the link:
    btScalar mass = linkBody->GetMass();

    btVector3 inertia(0, 0, 0);
    if(isDynamic) // Calculate the moment of inertia if base is dynamic
      _shapes.at(linkPmplIndex)->calculateLocalInertia(mass, inertia);


    //NOTE: I'm ASSUMING that the data in transToDHFrame is the body1->Joint
    //transform and then the transToBody2 is Joint->Body2 (not body1->body2)

    //These are the 3 members that may be needed for the next data:
    DHParameters& dhParams = _joints.at(i)->GetDHParameters();
    //Transformation objects have a translation vec and a Orientation (3x3 mat)
    Transformation transToBody2 = _joints.at(i)->GetTransformationToBody2();
    Transformation transToDHFrame = _joints.at(i)->GetTransformationToDHFrame();

    //Due to my above assumption, I'm also using the product to two matrices
    //to represent the full rotation from parent body -> new link
    //TODO should dhFrameTransform be just the first matrix? Look at future use for more info
    mathtool::Matrix3x3 dhFrameTransform = transToDHFrame.rotation().matrix()
                            * dhParams.GetTransformation().rotation().matrix();
    //Combine the matrices so that it's body1 -> body2 transform:
    mathtool::Matrix3x3 fullRotation = dhFrameTransform
                                      * transToBody2.rotation().matrix();
    //Convert into quaternion from matrix:
    Quaternion pmplRotParentToThis = Quaternion();
    mathtool::convertFromMatrix(pmplRotParentToThis, fullRotation);
    //Convert to btQuaternion from PMPL quaternion:
    btQuaternion rotParentToThis = ToBullet(pmplRotParentToThis);

    //vector between parent COM and joint:
    btVector3 parentComToThisPivotOffset = ToBullet(transToDHFrame.translation());
    //vector between link COM and joint:
    btVector3 thisPivotToThisComOffset = ToBullet(transToBody2.translation());

//    std::cout << std::endl<< std::endl<< std::endl<<"BULLETENGINE.CPP: ABOUT TO PRINT A BUNCH OF STUFF!!!" << std::endl;
//    std::cout << "dhFrameTransform = " <<dhFrameTransform << std::endl << std::endl;
//    std::cout << "fullRotation =  "<< fullRotation << std::endl << std::endl;
//    std::cout << "pmplRotParentToThis = " << pmplRotParentToThis << std::endl << std::endl;
//    std::cout << "rotParentToThis = ";
//    PrintbtQuaternion(rotParentToThis);

    //These show that we have the correct offset from the joint info in PMPL,
    // so it appears Bullet is doing exactly as PMPL is specifying.
    if(m_debug) {
      std::cout << std::endl << "parentComToThisPivotOffset = ";
      PrintbtVector3(parentComToThisPivotOffset);
      std::cout << std::endl << "thisPivotToThisComOffset = ";
      PrintbtVector3(thisPivotToThisComOffset);
    }

    //It looks like this should be true (disabled) if "the self-collision
    //has conflicting/non-resolvable contact normals"
    const bool disableParentCollision = true;

    //NOTE! Link means the body! Not the joint.

    // Determine joint type (Revolute, Prismatic, Spherical, Planar, or Fixed)
    //Note, PMPL only does Revolute, Spherical, or NonActuated/Fixed right now
    switch(_joints.at(i)->GetConnectionType()) {
      case Connection::JointType::Revolute :
      {
        //jointAxis should be what axis the rotation is about
        //jointAxis should simply be the z-axis from the DH frame (from PMPL)

        //I'm not sure if the joint axis would need to include that 'internal' transform
        //for DH or not... I'm assuming it does for now. If not, change
        //dhFrameTransform as mentioned near definition
        //Note: The above comment shouldn't matter actually, since that DH frame
        //transform should be limited to rotations ABOUT the z direction.

        //Justification for jointAxis calculation:
        //world transform of parent * transform to joint * vector that grabs the z-axis of that frame.
        //Note that this is STILL assuming that jointAxis is not relative to anything but WORLD frame,
        //instead of the link/joint frame... Which is probably going to be incorrect.
        //HOWEVER, if it's actually wrong in the end, the correct one should just be (0,0,1).
        btVector3 jointAxis =
            (_transforms.at(parentPmplIndex).getBasis() // parent's world trans.
                * ToBullet(dhFrameTransform)) // move into the DH frame
                * btVector3(0.,0.,1.); // grab the z-axis (DH axis of rotation)

        if(m_debug) {
          std::cout << "jointAxis = ";
          PrintbtVector3(jointAxis);
        }

        mb->setupRevolute(linkIndex, mass, inertia, parentIndex,
               rotParentToThis, jointAxis, parentComToThisPivotOffset,
               thisPivotToThisComOffset, disableParentCollision);

        //Handle joint constraints/limits:
        //First get PMPL's copy of the limits.
        const pair<double, double> limits = _joints.at(i)->GetJointLimits(0);
        //Bullet uses [-PI, PI], PMPL uses [-1, 1].

        //So from how btMultiBodyJointLimitConstraint uses btMultiBodyConstraint
        // in its constructor, it's clear you pass the child's link number
        // (recall: link = body, not joint) and it grabs the parent automatically
        // Note that linkIndex should equal i
        btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(mb,
                                      linkIndex, limits.first*PI, limits.second*PI);
                                      //linkIndex, 1., 0.);
                                      //linkIndex, -PI / 2., PI / 2.);

        m_dynamicsWorld->addMultiBodyConstraint(con);

        if(m_debug)
          std::cout << "Set constraints on joint " << i << " to: "
                    << limits.first * PI << " and " << limits.second * PI
                    << std::endl;
        break;
      }
      case Connection::JointType::Spherical :
      {
        //This case can be used, but please note that NO constraints will
        // be respected, and you must manually comment out this exception:
        RunTimeException(WHERE, "Spherical joints are supported, but spherical "
            "constraints are NOT! You can remove this exception if you need a "
            "spherical joint with no constraints.");

        mb->setupSpherical(linkIndex, mass, inertia, parentIndex,
              rotParentToThis, parentComToThisPivotOffset,
              thisPivotToThisComOffset, disableParentCollision);
        /// @TODO support spherical constraints. As per answers here:
        /// http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=10780
        /// Bullet doesn't support >1DOF joints on btMultiBodies.
        break;
      }
      case Connection::JointType::NonActuated :
      {
        mb->setupFixed(linkIndex, mass, inertia, parentIndex,
            rotParentToThis, parentComToThisPivotOffset,
            thisPivotToThisComOffset, disableParentCollision);
        //Since it's fixed, there is no need for joint constraints.
        break;
      }
      default:
        throw RunTimeException(WHERE, "Unsupported joint type.");
    }
  }

  //Do some of the final things:
  mb->finalizeMultiDof();
  mb->setBaseVel(btVector3(0, 0, 0));//important to have this after finalizeMultiDof()
  m_dynamicsWorld->addMultiBody(mb);

  //Using the Bullet example file in examples/MultiBody/Pendulum.cpp
  //as the basis for this (that's why this is very last):
  for (int i = 0; i < mb->getNumLinks(); i++) {
    //This loop, like in the example, is assuming that i represents the child
    //link of each joint (make sense since base, the -1 index, is already set up)
    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(mb, i);
    col->setCollisionShape(_shapes.at(i+1));
    bool isDynamic = 1;
    short collisionFilterGroup = isDynamic ?
        short(btBroadphaseProxy::DefaultFilter)
        : short(btBroadphaseProxy::StaticFilter);
    short collisionFilterMask = isDynamic ?
        short(btBroadphaseProxy::AllFilter)
        : short(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);
    m_dynamicsWorld->addCollisionObject(col,
                                    collisionFilterGroup, collisionFilterMask);
    //Finally, set the link's actual collider:
    mb->getLink(i).m_collider = col;
  }

  //Also from Pendulum.cpp:
  btAlignedObjectArray<btQuaternion> scratch_q;
  btAlignedObjectArray<btVector3> scratch_m;
  mb->forwardKinematics(scratch_q,scratch_m);
  btAlignedObjectArray<btQuaternion> world_to_local;
  btAlignedObjectArray<btVector3> local_origin;
  mb->updateCollisionObjectWorldTransforms(world_to_local,local_origin);

  //After everything is set up, return the multibody pointer:
  return mb;
}

/*------------------------------ Helpers -------------------------------------*/

vector<btCollisionShape*>
BulletEngine::
BuildCollisionShape(MultiBody* _body) {
  // Get the multibody's obj file and use it to create a bullet body.
  /// @TODO Parse all components of the multibodies instead of just the first.
  ///       Look at bullet/examples/MultiBody/MultiDofDemo.cpp
  ///           and bullet/src/BulletDynamics/FeatherStone/btMultiBody.h
  std::vector< std::string > filenames;
  std::vector< btCollisionShape* > shapes;

  //Determine whether it's a static or active MultiBody based on casting result:
  StaticMultiBody* sbody = dynamic_cast<StaticMultiBody*>(_body);
  if(sbody)
    filenames.push_back(sbody->GetFixedBody(0)->GetFilePath());
  else {
    ActiveMultiBody* abody = dynamic_cast<ActiveMultiBody*>(_body);
    for(size_t i = 0; i < abody->NumFreeBody(); i++)
      filenames.push_back(abody->GetFreeBody(i)->GetFilePath());
  }

  // Make simulator collision shape(s).
  for(size_t i = 0; i < filenames.size(); i++){
    btTriangleMesh* shapeMesh = BuildCollisionShape(filenames.at(i));

    //Make the bullet impact shape:
    auto ptr = new btGImpactMeshShape(shapeMesh);
    ptr->updateBound();
    shapes.push_back(ptr);
  }

  return shapes;
}


btTriangleMesh*
BulletEngine::
BuildCollisionShape(const std::string& _filename) {
  ConvexDecomposition::WavefrontObj obj;
  obj.loadObj(_filename.c_str());

  // Build a btTriangleMesh from an obj file.
  btTriangleMesh* mesh = new btTriangleMesh();
  mesh->preallocateVertices(obj.mVertexCount);
  mesh->preallocateIndices(obj.mTriCount);

  // Add vertices.
  for(int i = 0; i < obj.mVertexCount; ++i) {
    int start = i * 3;
    mesh->findOrAddVertex(btVector3(obj.mVertices[start],
                                    obj.mVertices[start + 1],
                                    obj.mVertices[start + 2]), false);
  }

  // Add facets.
  for(int i = 0; i < obj.mTriCount; ++i) {
    int start = i * 3;
    mesh->addTriangleIndices(obj.mIndices[start],
                             obj.mIndices[start + 1],
                             obj.mIndices[start + 2]);
  }

  return mesh;
}

/*----------------------------------------------------------------------------*/

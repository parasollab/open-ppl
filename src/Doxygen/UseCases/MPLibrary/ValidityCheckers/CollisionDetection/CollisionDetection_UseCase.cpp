void CollisionDetectionUseCase() {

  // The GMSPolyhedron objects represent d-dimensional polyhedrons. Workspace
  // obstacles and robots are represented as 3d polyhedrons.
  GMSPolyhedron polyhedronA, polyhedronB;

  // The Transformation objects represent movements of 3d objects in space.
  mathtool::Transformation transformA, transformB;

  // The collision detection info object will be used by the CD object to output
  // additional information such as the clearance (distance from obstacles) of
  // the configuration.
  CDInfo cdInfo;

  // The collision detection method object
  CollisionDetectionMethod* cd = new CollisionDetectionMethod("PQP");

  // The IsInCollision function will return true if the polyhedrons are
  // considered to be in collision after being subjected to the respective
  // transformations. Less formally, the function checks if two objects
  // will be in collision with each other after certain movements.
  // Additional information will be stored in the CDInfo object cdInfo.
  bool valid = cd->IsInCollision(polyhedronA, transformA,
                                 polyhedronB, transformB, cdInfo);


  // A point that we would like to check the validity of.
  Point3D point;

  // The IsInsideObstacle function will return true if the point is inside of
  // the polyhedron.
  bool isInside = cd->IsInsideObstacle(point, polyhedronA, transformA);
}

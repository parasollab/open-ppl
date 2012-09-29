#include "SpheresCollisionDetection.h"
#include "MetricUtils.h"
#include "MultiBody.h"

BoundingSpheres::
BoundingSpheres() : CollisionDetectionMethod() {
  m_name = "boundingSpheres";
  m_type = Out;
  m_cdtype = BOUNDING_SPHERES;
}


BoundingSpheres::
~BoundingSpheres() {
}

bool
BoundingSpheres::
IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
	      StatClass& _stats, CDInfo& _cdInfo, std::string *_callName, int _ignoreIAdjacentMultibodies) {
  //cout << endl << "boundingSpheres Collision Check invocation" << flush;
  _stats.IncNumCollDetCalls(GetName(), _callName );
  
  Vector3D robot_com = _robot->GetCenterOfMass();
  Vector3D obst_com  = _obstacle->GetCenterOfMass();
  
  if(_robot->GetFreeBodyCount())
    robot_com = _robot->GetFreeBody(0)->GetWorldTransformation() * robot_com;
  if(_obstacle->GetFreeBodyCount())
    obst_com  = _obstacle->GetFreeBody(0)->GetWorldTransformation() * obst_com;
  
  double robot_radius = _robot->GetBoundingSphereRadius();
  double obst_radius  = _obstacle->GetBoundingSphereRadius();
  
  double dist = sqrt(sqr(robot_com.getX() - obst_com.getX()) +
		     sqr(robot_com.getY() - obst_com.getY()) +
		     sqr(robot_com.getZ() - obst_com.getZ()));
  
  if (dist > robot_radius+obst_radius)
    return false;
  else
    return true;
}


//////////


InsideSpheres::
InsideSpheres() : CollisionDetectionMethod() {
  m_name = "insideSpheres";
  m_type = In;
  m_cdtype = INSIDE_SPHERES;
}


InsideSpheres::
~InsideSpheres() {
}

bool
InsideSpheres::
IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
	      StatClass& _stats, CDInfo& _cdInfo, std::string *_callName,  int _ignoreIAdjacentMultibodies) {
  //cout << endl << "insideSpheres Collision Check invocation";
  _stats.IncNumCollDetCalls(GetName(),_callName );
  
  Vector3D robot_com = _robot->GetCenterOfMass();
  Vector3D obst_com  = _obstacle->GetCenterOfMass();

  if(_robot->GetFreeBodyCount())
    robot_com = _robot->GetFreeBody(0)->GetWorldTransformation() * robot_com;
  if(_obstacle->GetFreeBodyCount())
    obst_com  = _obstacle->GetFreeBody(0)->GetWorldTransformation() * obst_com;

  double robot_radius = _robot->GetInsideSphereRadius();
  double obst_radius  = _obstacle->GetInsideSphereRadius();

  double dist = sqrt(sqr(robot_com.getX() - obst_com.getX()) +
		     sqr(robot_com.getY() - obst_com.getY()) +
		     sqr(robot_com.getZ() - obst_com.getZ()));
  
  if (dist > robot_radius+obst_radius)
    return false;
  else
    return true;
}


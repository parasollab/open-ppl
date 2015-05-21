#include "SolidCollisionDetection.h"

#ifdef USE_SOLID

Solid::Solid() : CollisionDetectionMethod() {
  m_name = "SOLID";
  m_type = Exact;
}

void
Solid::
Build(Body* _body) {
  GMSPolyhedron& poly = _body->GetWorldPolyhedron();
  vertex = new MT_Point3[3*poly.m_polygonList.size()];
  for(size_t q=0; q < poly.m_polygonList.size(); q++) {
    int vertexNum[3];
    float point[3][3];
    for(int i=0; i<3; i++) {
      vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
      Vector3d tmp = poly.m_vertexList[vertexNum[i]];
      for(int j=0; j<3; j++)
        vertex[3*q+i][j] = tmp[j];
    }
  }
  base = DT_NewVertexBase(vertex[0],sizeof(vertex[0]));
  DT_ShapeHandle shape = DT_NewComplexShape(base);
  for(size_t q=0; q < poly.m_polygonList.size(); q++) {
    int vertexNum[3];
    float point[3][3];
    for(int i=0; i<3; i++) {
      vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
      Vector3d tmp = poly.m_vertexList[vertexNum[i]];
      for(int j=0; j<3; j++)
        point[i][j] = tmp[j];
    }
    DT_Begin();
    DT_VertexIndex(3*q+0);
    DT_VertexIndex(3*q+1);
    DT_VertexIndex(3*q+2);
    DT_End();
  }
  DT_EndComplexShape();
  DT_ObjectHandle object = DT_CreateObject(NULL,shape);
  _body->SetSolidBody(shared_ptr<DT_ObjectHandle>(new DT_ObjectHandle(object));
}

bool
Solid::IsInCollision(shared_ptr<ActiveMultiBody> _robot,
    shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
    size_t _ignoreIAdjacentMultibodies) {

  _robot->UpdateVertexBase();

  if(_cdInfo.ret_all_info == false){

    for(size_t i = 0; i < _robot->GetFreeBodyCount(); ++i) {

      shared_ptr<DT_ObjectHandle> rob = _robot->GetFreeBody(i)->GetSolidBody();

      for(size_t j = 0; j < _obstacle->GetBodyCount(); ++j) {

        // if robot check self collision, skip adjacent links.
        if(_robot == _obstacle &&
            _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) )
          continue;


        shared_ptr<DT_ObjectHandle> obst = _obstacle->GetBody(j)->GetSolidBody();

        MT_Vector3 cp1,cp2;
        MT_Vector3 separation;
        float tempDist;

        if (DT_GetPenDepth(*rob, *obst, cp1, cp2)){// intersection
          separation = cp1 - cp2;
          tempDist = cp1.distance(cp2);
          return true;
        }

      } // end for j
    } // end for i

    return false;
  }// _cdInfo.ret_all_info = false
  else{

    bool retVal = false;
    float dist = maxDist;  // =  1e10 by CollisionDetection.h
    float tempDist;


    // default _cdInfo contents
    _cdInfo.ResetVars();
    _cdInfo.ret_all_info = true;

    for(size_t i = 0 ; i < _robot->GetFreeBodyCount(); ++i) {

      shared_ptr<DT_ObjectHandle> rob = _robot->GetFreeBody(i)->GetSolidBody();

      for(size_t j = 0; j < _obstacle->GetBodyCount(); ++j) {

        // if robot check self collision, skip adjacent links.
        if(_robot == _obstacle &&
            _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) )
          continue;


        shared_ptr<DT_ObjectHandle> obst = _obstacle->GetBody(j)->GetSolidBody();

        MT_Vector3 cp1,cp2;
        MT_Vector3 separation;

        if (DT_GetPenDepth(*rob, *obst, cp1, cp2)){// intersection
          separation = cp1 - cp2;
          tempDist = -cp1.distance(cp2);

          DT_GetPenDepth(*obst, *rob, cp1, cp2);
          separation = cp1 - cp2;
          tempDist = -cp1.distance(cp2);

          retVal = true;
          if(tempDist < dist){
            dist = tempDist;
            _cdInfo.colliding_obst_index = j;
            _cdInfo.nearest_obst_index = j;
            _cdInfo.min_dist = dist;


            _cdInfo.robot_point = _robot->GetFreeBody(i)->WorldTransformation() * Vector3d(cp1[0],cp1[1],cp1[2]);
            _cdInfo.object_point = _obstacle->GetBody(j)->WorldTransformation() * Vector3d(cp2[0],cp2[1],cp2[2]);
          }

        }else{// no intersection
          DT_GetClosestPair(*rob, *obst, cp1, cp2);
          separation = cp2 - cp1;
          tempDist = cp1.distance(cp2);
          if(tempDist < dist){
            dist = tempDist;
            _cdInfo.nearest_obst_index = j;
            if(dist == 0){
              retVal = true;
              _cdInfo.colliding_obst_index = j;
            }
            _cdInfo.min_dist = dist;
            _cdInfo.robot_point = _robot->GetFreeBody(i)->WorldTransformation() * Vector3d(cp1[0],cp1[1],cp1[2]);
            _cdInfo.object_point = _obstacle->GetBody(j)->WorldTransformation() * Vector3d(cp2[0],cp2[1],cp2[2]);
          }
        }

      } // end for j
    } // end for i

    return retVal;
  }// _cdInfo.ret_all_info = true

} // end IsInCollision_solid()

#endif

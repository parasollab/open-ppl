#include "PQPCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Cfg/Cfg.h"
#include "MPProblem/Environment.h"

#ifdef USE_PQP
PQP::
PQP() : CollisionDetectionMethod() {
  m_name = "PQP";
  m_type = Exact;
  m_cdtype = PROXIMITYQUERYPACKAGE;
}


PQP::
~PQP() {
}


bool
PQP::
IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, StatClass& _stats, CDInfo& _cdInfo, std::string *_callName, int _ignoreIAdjacentMultibodies) {
  _stats.IncNumCollDetCalls(GetName(), _callName);

  if (_cdInfo.m_retAllInfo == true) {
    PQP_DistanceResult res;
    double m_minDist_so_far = MaxDist;
    _cdInfo.ResetVars();
    _cdInfo.m_retAllInfo = true;
    Vector3D robot_pt, obs_pt;
    bool ret_val=false;

    //for each part of robot
    for(int i=0 ; i<_robot->GetFreeBodyCount(); i++) {
      shared_ptr<PQP_Model> rob = _robot->GetFreeBody(i)->GetPQPBody();
      Transformation &t1 = _robot->GetFreeBody(i)->WorldTransformation();
      t1.m_orientation.ConvertType(Orientation::Matrix);
      double p1[3]; for(int ip1=0; ip1<3; ip1++) p1[ip1] = t1.m_position[ip1];

      //for each part of obstacle
      for(int j=0; j<_obstacle->GetBodyCount(); j++) {
        // if robot check self collision, skip adjacent links.
        if(_robot == _obstacle &&
           _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) )
          continue;

        shared_ptr<PQP_Model> obst = _obstacle->GetBody(j)->GetPQPBody();

        Transformation &t2 = _obstacle->GetBody(j)->WorldTransformation();
        t2.m_orientation.ConvertType(Orientation::Matrix);
        double p2[3]; for(int ip2=0; ip2<3; ip2++) p2[ip2] = t2.m_position[ip2];

        if(PQP_Distance(&res,t1.m_orientation.matrix,p1,rob.get(),
                        t2.m_orientation.matrix,p2,obst.get(),0.0,0.0)) {
          cout << "Error in CollisionDetection::PQP_Collide, PQP_ERR_COLLIDE_OUT_OF_MEMORY"<<endl;
          exit(1);
        }

        if ( res.Distance() <= 0.0 ){
	  if ( res.Distance() < m_minDist_so_far)
	    _cdInfo.m_collidingObstIndex = j;
          ret_val = true;
	}

        if( res.Distance()<m_minDist_so_far ) {
          _cdInfo.m_nearestObstIndex = j; 
          // which called this function - look there for more info
          m_minDist_so_far=res.Distance();
          _cdInfo.m_minDist = m_minDist_so_far;

          // change a 3 elmt array to Vector3D class
          for( int k=0;k<3;k++ ) {
            robot_pt[k] = res.P1()[k];
            obs_pt[k] = res.P2()[k];
          }

          // transform points to world coords
          // using *_pt vars in case overloaded * was not done well.
          _cdInfo.m_robotPoint = _robot->GetFreeBody(i)->WorldTransformation() * robot_pt;
          _cdInfo.m_objectPoint = _obstacle->GetBody(j)->WorldTransformation() * obs_pt;
        }
      }//end of each part of obs
    }//end of each part of robot

    return ret_val;
  } else {
    for(int i=0 ; i<_robot->GetFreeBodyCount(); i++) {
      shared_ptr<PQP_Model> rob = _robot->GetFreeBody(i)->GetPQPBody();

      for(int j=0; j<_obstacle->GetBodyCount(); j++) {
        // if robot check self collision, skip adjacent links.
        if(_robot == _obstacle &&
           _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) )
          continue;
  
        shared_ptr<PQP_Model> obst = _obstacle->GetBody(j)->GetPQPBody();
        Transformation &t1 = _robot->GetFreeBody(i)->WorldTransformation();
        Transformation &t2 = _obstacle->GetBody(j)->WorldTransformation();
        t1.m_orientation.ConvertType(Orientation::Matrix);
        t2.m_orientation.ConvertType(Orientation::Matrix);
        double p1[3], p2[3];
        for(int p=0; p<3; p++) 
        {
          p1[p] = t1.m_position[p];
          p2[p] = t2.m_position[p];
        }
  
        PQP_CollideResult result;
        if(PQP_Collide(&result, t1.m_orientation.matrix, p1, rob.get(),
                       t2.m_orientation.matrix, p2, obst.get(), PQP_FIRST_CONTACT)) {
          cout << "Error in CollisionDetection::PQP_Collide, PQP_ERR_COLLIDE_OUT_OF_MEMORY"
               << PQP_Collide(&result, t1.m_orientation.matrix, p1, rob.get(), t2.m_orientation.matrix, p2, obst.get(), PQP_FIRST_CONTACT) << endl;
          exit(1);
        }
        if(result.Colliding()) 
          return true;
      }
    }
    return false;
  }
}


//////////


bool 
PQP_Solid::
IsInsideObstacle(const Cfg& _cfg, Environment* _env) 
{
  int nmulti = _env->GetMultiBodyCount();
  int robot = _env->GetRobotIndex();

  Vector3D robot_pt(_cfg.GetData()[0], _cfg.GetData()[1], _cfg.GetData()[2]);
  
  for( int i=0; i < nmulti; i++ )
    if(i != robot && IsInsideObstacle(robot_pt, _env->GetMultiBody(i)))
      return true;
  return false;
}


PQP_Model* 
PQP_Solid::
BuildPQPSegment(PQP_REAL _dX, PQP_REAL _dY, PQP_REAL _dZ) const {
  //build a narrow triangle.
  PQP_Model* pRay = new PQP_Model();
  if( pRay==NULL ) 
    return NULL;
  
  if( _dY==0 && _dZ==0 && _dX==0 ) 
    cout<<"! CollisionDetection::BuildPQPRay Warning : All are [0]"<<endl;
  
  static PQP_REAL tiny_v = ((double)1e-20)/LONG_MAX;
  static PQP_REAL pico_v = tiny_v/2;
  PQP_REAL p1[3] = { tiny_v, tiny_v, tiny_v };
  PQP_REAL p2[3] = { pico_v, pico_v, pico_v };
  PQP_REAL p3[3] = { _dX, _dY, _dZ};
  
  pRay->BeginModel();
  pRay->AddTri(p1, p2, p3, 0);
  pRay->EndModel();
  
  return pRay;
}


bool 
PQP_Solid::
IsInsideObstacle(Vector3D _robot_pt, shared_ptr<MultiBody> _obstacle)
{
  static PQP_Model* m_pRay = BuildPQPSegment(1e10,0,0);
  assert(m_pRay != NULL);

  PQP_REAL t[3]={_robot_pt[0], _robot_pt[1], _robot_pt[2]};
  static PQP_REAL r[3][3]={{1,0,0}, {0,1,0}, {0,0,1}};
  
  for(int j=0; j<_obstacle->GetBodyCount(); j++) {
    shared_ptr<PQP_Model> obst = _obstacle->GetBody(j)->GetPQPBody();
    //GMSPolyhedron& poly=_obstacle->GetBody(j)->GetPolyhedron();
    Transformation& t2 = _obstacle->GetBody(j)->WorldTransformation();
    t2.m_orientation.ConvertType(Orientation::Matrix);
    double p2[3];
    for(int p=0; p<3; p++) 
      p2[p] = t2.m_position[p];
    
    PQP_CollideResult result;
    PQP_Collide(&result,r,t,m_pRay,t2.m_orientation.matrix,p2,obst.get());
   
    /*
    //
    // checking for adjacent triangles produces incorrect results, removed
    //
    //anaylize result (check if there are adjacent triangle)
    vector<int> tri;
    //for each tri
    for( int iT=0; iT < result.NumPairs(); iT++ ) {
      bool add = true;
      int* tri1 = poly.polygonList[result.Id2(iT)].m_vertexList;
      //for each checked triangle
      for( int i=0; i < tri.size(); i++) {
        int* tri2=poly.polygonList[tri[i]].m_vertexList;
        //check if they share same vertices
        for(int itri2=0; itri2 < 3; itri2++)
          for(int itri1=0; itri1 < 3; itri1++)
            if( tri2[itri2] == tri1[itri1] ) {
              add = false;
              break;
            }
        if( add==false ) 
          break;
      }
      //Ok no one shares vertex with you...
      if( add==true ) 
        tri.push_back(result.Id2(iT));
    }
    if( (tri.size()%2)==1 ) 
      return true;
    */

    if((result.NumPairs() % 2) == 1)
      return true;
  }//end of each part of obs

  return false;
} 


bool
PQP_Solid::
IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, StatClass& _stats, CDInfo& _cdInfo, std::string *_callName, int _ignoreIAdjacentMultibodies) {
  _stats.IncNumCollDetCalls(GetName(), _callName);

  PQP_CollideResult result;

  if (_cdInfo.m_retAllInfo == true) {
    PQP_DistanceResult res;
    double m_minDist_so_far = MaxDist;
    Vector3D robot_pt, obs_pt;
    bool ret_val=false;

    //for each part of robot
    for(int i=0 ; i<_robot->GetFreeBodyCount(); i++) {
      shared_ptr<PQP_Model> rob = _robot->GetFreeBody(i)->GetPQPBody();
      Transformation &t1 = _robot->GetFreeBody(i)->WorldTransformation();
      t1.m_orientation.ConvertType(Orientation::Matrix);
      double p1[3]; for(int ip1=0; ip1<3; ip1++) p1[ip1] = t1.m_position[ip1];

      //for each part of obstacle
      for(int j=0; j<_obstacle->GetBodyCount(); j++) {
        // if robot check self collision, skip adjacent links.
	//replace with finction that checks is is in i of link
        if(_robot == _obstacle &&
           _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) )
          continue;

        shared_ptr<PQP_Model> obst = _obstacle->GetBody(j)->GetPQPBody();

        Transformation &t2 = _obstacle->GetBody(j)->WorldTransformation();
        t2.m_orientation.ConvertType(Orientation::Matrix);
        double p2[3]; for(int ip2=0; ip2<3; ip2++) p2[ip2] = t2.m_position[ip2];

        if(PQP_Distance(&res,t1.m_orientation.matrix,p1,rob.get(),
                        t2.m_orientation.matrix,p2,obst.get(),0.0,0.0)) {
          cerr << "Error in CollisionDetection::PQP_Collide, PQP_ERR_COLLIDE_OUT_OF_MEMORY"<<endl;
          exit(1);
        }

        if ( res.Distance() <= 0.0 )  
          ret_val = true;

        if( res.Distance()<m_minDist_so_far ) {
          // _cdInfo.m_nearestObstIndex =  is set by IsInCollision()
          // which called this function - look there for more info
          m_minDist_so_far=res.Distance();
          _cdInfo.m_minDist = m_minDist_so_far;

          // change a 3 elmt array to Vector3D class
          for( int k=0;k<3;k++ ) {
            robot_pt[k] = res.P1()[k];
            obs_pt[k] = res.P2()[k];
          }

          // transform points to world coords
          // using *_pt vars in case overloaded * was not done well.
          _cdInfo.m_robotPoint = _robot->GetFreeBody(i)->WorldTransformation() * robot_pt;
          _cdInfo.m_objectPoint = _obstacle->GetBody(j)->WorldTransformation() * obs_pt;
        }
      }//end of each part of obs

      if(ret_val == false && _robot != _obstacle && IsInsideObstacle(_robot->GetFreeBody(i)->GetWorldPolyhedron().m_vertexList[0], _obstacle))
        ret_val = true;
    }//end of each part of robot
    return ret_val;
  } else {
    for(int i=0 ; i<_robot->GetFreeBodyCount(); i++) {
      shared_ptr<PQP_Model> rob = _robot->GetFreeBody(i)->GetPQPBody();

      for(int j=0; j<_obstacle->GetBodyCount(); j++) {
        // if robot check self collision, skip adjacent links.
        if(_robot == _obstacle &&
           _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) )
          continue;
  
        shared_ptr<PQP_Model> obst = _obstacle->GetBody(j)->GetPQPBody();
        Transformation &t1 = _robot->GetFreeBody(i)->WorldTransformation();
        Transformation &t2 = _obstacle->GetBody(j)->WorldTransformation();
        t1.m_orientation.ConvertType(Orientation::Matrix);
        t2.m_orientation.ConvertType(Orientation::Matrix);
        double p1[3], p2[3];
        for(int p=0; p<3; p++) {
          p1[p] = t1.m_position[p];
          p2[p] = t2.m_position[p];
        }
  
        if(PQP_Collide(&result, t1.m_orientation.matrix, p1, rob.get(),
                       t2.m_orientation.matrix, p2, obst.get(), PQP_FIRST_CONTACT)) {
          cerr << "Error in CollisionDetection::PQP_Collide, PQP_ERR_COLLIDE_OUT_OF_MEMORY"
               << PQP_Collide(&result, t1.m_orientation.matrix, p1, rob.get(), t2.m_orientation.matrix, p2, obst.get(), PQP_FIRST_CONTACT) << endl;
          exit(1);
        }
        if(result.Colliding()) 
          return true;
      }

      if(_robot != _obstacle && IsInsideObstacle(_robot->GetFreeBody(i)->GetWorldPolyhedron().m_vertexList[0], _obstacle))
        return true;
    }
    return false;
  }
}

#endif


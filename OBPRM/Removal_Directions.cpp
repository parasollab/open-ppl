///////////////////////////////////////////////////////////////////////////////
//  Removal_Directions.cpp
// 
//  Class containing heuristics for removal directions for the parts
//  of an assembly
//
//  10/26/00  Sujay
//
///////////////////////////////////////////////////////////////////////////////

#include "Removal_Directions.h"


vector<Cfg> Removal_Directions::expand(Cfg * _cfg, Roadmap * _rm, 
              CollisionDetection cd, ConnectMapNodes cn, int HEURISTIC) {

  _cfg->ConfigEnvironment(_rm->environment);
  vector<Cfg> validcfg;
  if (HEURISTIC == 1) {
    validcfg = Normals(_cfg, _rm, cd, cn);
  }
  return validcfg;
}


vector<Cfg> Removal_Directions::Normals(Cfg * _cfg, Roadmap * _rm, 
              CollisionDetection cd, ConnectMapNodes cn) {
  vector<Cfg> validcfgs;
  MultiBody * robot = 
   _rm->environment->GetMultiBody(_rm->environment->GetRobotIndex());
  for (int i = 0; i < robot->GetFreeBodyCount(); i++) {
    vector < Vector3D > norm_used;
    if (!IsBeyond(_rm->environment, i, CLEARANCE)) {
      robot->GetFreeBody(i)->GetPolyhedron().ComputeNormals();
      int numofpolygons = robot->GetFreeBody(i)->GetPolyhedron().numPolygons;
      for (int j = 0; j < numofpolygons; j++) {
        Vector3D bodynormal = 
           robot->GetBody(i)->GetPolyhedron().polygonList[j].normal;
// cout << "Polygon " << j << ":::" << bodynormal <<endl;
        bool flag = false;
        for (int x = 0; x < norm_used.size(); x++) {
          if (bodynormal[0]==norm_used[x][0] && bodynormal[1]==norm_used[x][1] 
               && bodynormal[2]==norm_used[x][2]) {
            flag = true;
            break;
          }
        }
        if (flag) continue;
        norm_used.push_back(bodynormal);
        Cfg newcfg;
        bool collision = false;
        vector<double> position;
        for (double k = REMOVAL_INCREMENT; k < REMOVAL_DISTANCE; k+= 
              REMOVAL_INCREMENT) {
          position = _cfg->GetData();
          for (int l = 0; l < 3; l++)
            position[i * 6 + l] = position[i * 6 + l] + bodynormal[l] * k;
          newcfg.SetData(position);
          collision = newcfg.isCollision(_rm->environment, &cd, 
                         cn.cnInfo.cdsetid, cd.cdInfo);
          if (collision && k < MINIMUM_CLEARANCE)
            break;
          else if (collision && k >= MINIMUM_CLEARANCE) {
          // get back to previous non-colliding position
            position = _cfg->GetData();
            for (int l = 0; l < 3; l++)
              position[i * 6 + l] = position[i * 6 + l] + bodynormal[l] * 
                                   (k - REMOVAL_INCREMENT);
            newcfg.SetData(position);
            validcfgs.push_back(newcfg);
            break;
          }
        }
        if (!collision) {
          newcfg.SetData(position);
          validcfgs.push_back(newcfg);
        }
      }
    } // endif isBeyond ....
  }
  return validcfgs;
}


bool Removal_Directions::IsBeyond(Environment* env, int currentbody, 
                                  double distance) {
  void *checkbody, *obst;
  double tmp;
  MultiBody * robot = env->GetMultiBody(env->GetRobotIndex());
  checkbody = robot->GetFreeBody(currentbody)->GetCstkBody();
  for (int i = 0; i < robot->GetFreeBodyCount(); i++) {
    if (i != currentbody) {
      obst = robot->GetFreeBody(i)->GetCstkBody();
      tmp = cstkBodyBodyDist(checkbody, obst, 500, 0, NULL, 0, NULL);
      if (tmp < distance)
        return false;
    }
  }
    // NOTE: This is assuming that robot index is always 1
  for (int m = 1; m < env->GetMultiBodyCount(); m++) {
    for (int k = 0; k < env->GetMultiBody(m)->GetFixedBodyCount(); k++) {		      obst = env->GetMultiBody(m)->GetFixedBody(k)->GetCstkBody();
      tmp = cstkBodyBodyDist(checkbody, obst, 500, 0, NULL, 0, NULL);
      if (tmp < distance)
        return false;
    }
  }	
  return true;
}

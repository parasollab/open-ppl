#ifndef WorkspaceObstacleBasedSamplers_h
#define WorkspaceObstacleBasedSamplers_h

#include "ObstacleBasedSamplers.h"


template <typename CFG>
class WorkspaceObstacleBasedSampler : public ObstacleBasedSampler<CFG>
{
  string pointSelection;  

 public:
  WorkspaceObstacleBasedSampler() {
    this->SetName("WorkspaceObstacleBasedSampler");
  }
  
  WorkspaceObstacleBasedSampler(Environment* _env, 
                                shared_ptr<DistanceMetricMethod> _dm, int _free = 1, int _coll = 0, 
                                double _step = 0, string _pointSelection="rW")
    : ObstacleBasedSampler<CFG>(_env, _dm,  _free ,  _coll , _step) 
  {
    this->SetName("WorkspaceObstacleBasedSampler");
     pointSelection=_pointSelection;
  }
 
  WorkspaceObstacleBasedSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
    : ObstacleBasedSampler<CFG>(in_Node,  in_pProblem)
  {
    this->SetName("WorkspaceObstacleBasedSampler");
    pointSelection = in_Node.stringXMLParameter("point_selection", true, "", "point selection strategy");
    if(!( pointSelection.compare("cM")==0 || pointSelection.compare("rV")==0 || pointSelection.compare("rT")==0 || 
      pointSelection.compare("rW")==0 || pointSelection.compare("eV")==0 || pointSelection.compare("rV_rT")==0 || 
      pointSelection.compare("rV_rW")==0 || pointSelection.compare("all")==0 )) 
    {
      cerr << "Select a valid point selection type first. cM, rV ,rT, rW, eV, rV_rT, rV_rW, all are valid selection types. exiting.\n";
      exit(-1);
    }
  }

  ~WorkspaceObstacleBasedSampler() {}

  virtual void Print(ostream& os) const
  {
    ObstacleBasedSampler<CFG>::Print(os); 
    os << "  pointSelectionStrategy = " << pointSelection<< ")";
  }

  CFG ChooseCenterOfMass(shared_ptr<MultiBody> mBody)
  {
    Vector3D x = mBody->GetCenterOfMass();
    CFG tmp;
    for(int i=0;i<3;i++)
      tmp.SetSingleParam(i, x[i]);
    for(int i=3;i<6;i++)
      tmp.SetSingleParam(i, 0.0);
    return tmp;
  }
   
  CFG ChooseRandomVertex(shared_ptr<MultiBody> mBody,bool isFreeBody )
  {
    GMSPolyhedron polyhedron;
    if(isFreeBody) 
      polyhedron = mBody->GetBody(0)->GetPolyhedron();
    else           
      polyhedron = mBody->GetBody(0)->GetWorldPolyhedron();
    Vector3D x =polyhedron.vertexList[(int)(DRand()*polyhedron.vertexList.size())];
 
    CFG tmp;
    for(int i=0;i<3;i++)
      tmp.SetSingleParam(i, x[i]);
    for(int i=3;i<6;i++)
      tmp.SetSingleParam(i, 0.0);
    
    return tmp;
  }

  Vector3D ChoosePointOnTriangle(Vector3D p, Vector3D q, Vector3D r) 
  {
    Vector3D u, v;
    u = q - p;
    v = r - p;
 
    double s = DRand(); 
    double t = DRand();
    while(s + t > 1)
    {
      t = DRand();
    }
    return (p + u*s + v*t);
  }
   
  CFG ChooseRandomWeightedTriangle(shared_ptr<MultiBody> mBody, bool isFreeBody )
  {
    GMSPolyhedron polyhedron;
    if(isFreeBody)
      polyhedron = mBody->GetBody(0)->GetPolyhedron();
    else 
      polyhedron = mBody->GetBody(0)->GetWorldPolyhedron();
    double area;
    area = mBody->GetBody(0)->GetPolyhedron().area;
 
    double targetArea = area * DRand();
 
    int index, i;
    double sum;
    index = 0; 
    i = 1;
    sum = mBody->GetBody(0)->GetPolyhedron().polygonList[0].area;
    while(targetArea > sum)
    {
      sum += mBody->GetBody(0)->GetPolyhedron().polygonList[i].area;
      index++;
      i++;
    }
 
    // We choose the triangle of the mBody with that index
    GMSPolygon *poly = &polyhedron.polygonList[index];
 
    // We choose a random point in that triangle
    Vector3D p, q, r;
    p = polyhedron.vertexList[poly->vertexList[0]];
    q = polyhedron.vertexList[poly->vertexList[1]];
    r = polyhedron.vertexList[poly->vertexList[2]];
 
    Vector3D x;
    x = ChoosePointOnTriangle(p, q, r);
    CFG tmp;
    for(int i=0;i<3;i++)
      tmp.SetSingleParam(i, x[i]);
    for(int i=3;i<6;i++)
      tmp.SetSingleParam(i, 0.0);
    return tmp;
  }

  CFG ChooseRandomTriangle(shared_ptr<MultiBody> mBody, bool isFreeBody)
  {
    GMSPolyhedron polyhedron;
    if(isFreeBody)
      polyhedron = mBody->GetBody(0)->GetPolyhedron();
    else 
      polyhedron = mBody->GetBody(0)->GetWorldPolyhedron();
    GMSPolygon *poly = &polyhedron.polygonList[(int)(DRand()*polyhedron.polygonList.size())];
    Vector3D p, q, r;
    p = polyhedron.vertexList[poly->vertexList[0]];
    q = polyhedron.vertexList[poly->vertexList[1]];
    r = polyhedron.vertexList[poly->vertexList[2]];
    Vector3D x;
    x = ChoosePointOnTriangle(p, q, r);
    CFG tmp;
    for(int i=0;i<3;i++)
      tmp.SetSingleParam(i, x[i]);
    for(int i=3;i<6;i++)
      tmp.SetSingleParam(i, 0.0);
    return tmp;
  }
  
  CFG ChooseExtremeVertex(shared_ptr<MultiBody> mBody,bool isFreeBody)
  {
    GMSPolyhedron polyhedron;
    // for robot, choose mBody frame; for obstacle, choose world frame
    if(isFreeBody)
      polyhedron = mBody->GetBody(0)->GetPolyhedron();
    else 
      polyhedron = mBody->GetBody(0)->GetWorldPolyhedron();
  
    int indexVert[6];
    for(int j = 0 ; j < 6 ; j++)
      indexVert[j] = 0;
  
    for(size_t i = 1 ; i < polyhedron.vertexList.size() ; i++)
    {
      //MAX X
      if(polyhedron.vertexList[i][0] < polyhedron.vertexList[indexVert[0]][0])
        indexVert[0] = i;
    
      //MIN X
      if(polyhedron.vertexList[i][0] > polyhedron.vertexList[indexVert[1]][0])
        indexVert[1] = i;
    
      //MAX Y
      if(polyhedron.vertexList[i][1] < polyhedron.vertexList[indexVert[2]][1])
        indexVert[2] = i;
    
      //MIN Y
      if(polyhedron.vertexList[i][1] > polyhedron.vertexList[indexVert[3]][1])
        indexVert[3] = i;
    
      //MAX Z
      if(polyhedron.vertexList[i][2] < polyhedron.vertexList[indexVert[4]][2])
        indexVert[4] = i;
    
      //<MIN Z
      if(polyhedron.vertexList[i][2] > polyhedron.vertexList[indexVert[5]][2])
        indexVert[5] = i;
    }
  
    // Choose an extreme random vertex at random
    int index = LRand() % 6;
    Vector3D x= polyhedron.vertexList[indexVert[index]];
    CFG tmp;
    for(int i=0;i<3;i++)
      tmp.SetSingleParam(i, x[i]);
    for(int i=3;i<6;i++)
      tmp.SetSingleParam(i, 0.0);
    return tmp;
  }

  shared_ptr<MultiBody> initializeBody(Environment* env )
  {
   int N=env->GetMultiBodyCount();
   int roboindex = env->GetRobotIndex(); 
   int obstacleIndex;
   do
   {
     obstacleIndex=LRand() % N;   
   } while(obstacleIndex==roboindex);
   return env->GetMultiBody(obstacleIndex);// choose a multiBody randomly
 }
 
  virtual CFG ChooseASample(CFG cfg_in, Environment* env)
  {
    shared_ptr<MultiBody> mBody = initializeBody(env);
    bool isFreeBody = false;

    CFG temp;
    if(pointSelection.compare("cM")==0 )
      temp  = ChooseCenterOfMass(mBody);
    else if(pointSelection.compare("rV")==0 )
      temp  = ChooseRandomVertex(mBody,isFreeBody);
    else if(pointSelection.compare("rT")==0 )
      temp  = ChooseRandomTriangle(mBody,isFreeBody);
    else if(pointSelection.compare("rW")==0 )
      temp  = ChooseRandomWeightedTriangle(mBody,isFreeBody);
    else if(pointSelection.compare("eV")==0 )
      temp  = ChooseExtremeVertex(mBody,isFreeBody);
    else if(pointSelection.compare("cM_rV")==0 )
    { 
      int opt = LRand() % 2;
      if(opt == 0)
        temp  = ChooseCenterOfMass(mBody);
      else 
        temp  = ChooseRandomVertex(mBody,isFreeBody);
    }
    else if(pointSelection.compare("rV_rT")==0 )
    {
      int opt = LRand() % 2;
      if(opt == 0)
        temp  = ChooseRandomTriangle(mBody,isFreeBody);
      else 
        temp  = ChooseRandomVertex(mBody,isFreeBody);
    }
    else if(pointSelection.compare("rV_rW")==0 )
    {
      int opt = LRand() % 2;
      if(opt == 0)
        temp  = ChooseRandomVertex(mBody,isFreeBody);
      else
        temp  = ChooseRandomWeightedTriangle(mBody,isFreeBody);
    }
    else if(pointSelection.compare("all")==0 )
    {
      int opt = LRand() % 5;
      if(opt == 0)
        temp  = ChooseCenterOfMass(mBody);
      else if(opt == 1)
        temp  = ChooseRandomVertex(mBody,isFreeBody);
      else if(opt == 2)
        temp  = ChooseRandomTriangle(mBody,isFreeBody);
      else if(opt == 3)
        temp  = ChooseRandomWeightedTriangle(mBody,isFreeBody);
      else 
        temp  = ChooseExtremeVertex(mBody,isFreeBody);
    }
    else
    {
      cerr << "Select a valid point selection type first.exiting.\n";
      exit(-1);
    }

    return temp;
  }
  
};

#endif
 

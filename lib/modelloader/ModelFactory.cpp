#include <string>

#include "BVHDataLoader.h"
#include "MovieBYULoader.h"
#include "ModelFactory.h"
#include "ObjLoader.h"

using namespace std;
using namespace mathtool;

IModel*
CreateModelLoader(const string& _filename, bool _silent) {
  // Get file extension.
  size_t pos = _filename.rfind('.');
  if(pos == string::npos) {
    if(!_silent)
      cerr << "Error : Can't Recognize file :" << _filename << endl;
    return nullptr;
  }
  string ext = _filename.substr(pos+1);

  // Create model.
  IModel* model = nullptr;
  if(ext == "g")
    model = new CMovieBYULoader();
  else if(ext == "obj")
    model = new CObjLoader();
  else {
    if(!_silent)
      cerr << "Error : Can't Recognize extension *." << ext << endl;
    return nullptr;
  }

  // Parse model.
  model->SetDataFileName(_filename);
  if(!model->ParseFile(_silent)) {
    delete model;
    return nullptr;
  }

  return model;
}


vector<IModel*>&
CreateModelLoaderVec(CBVHDataModelFactory* _modelFactory,
    const string& _filename, double _radius, double _height) {
  int instanceIndex = -1;
  if(_modelFactory->hasBVHDataInstance(_filename, instanceIndex)) {
    //return pre-loaded vector of models
    return _modelFactory->getInstance(instanceIndex)->getModels();
  }
  else {
    CBVHDataInstance* bvhDataInstance = new CBVHDataInstance();
    bvhDataInstance->load(string(_filename), _radius, _height);
    _modelFactory->addInstance(bvhDataInstance);
    return bvhDataInstance->getModels();
  }
}


IModel*
CreateModelLoaderFromPts(vector<Point2d>& boundary) {
  cout << " --CreateModelLoaderFromPts." << endl;

  //compute pt center -- this is the part that assumes covnex
  Point2d center(0,0);
  for(unsigned int i = 0; i < boundary.size(); ++i) {
    center[0] += boundary[i][0];
    center[1] += boundary[i][1];
  }
  center[0] /= boundary.size();
  center[1] /= boundary.size();

  //make dummy CMovieBYULoader
  CMovieBYULoader* byuloader = new CMovieBYULoader();

  //add in pts
  using PtVector = IModel::PtVector;
  PtVector& ptvec = byuloader->GetVertices();
  for(unsigned int i = 0; i < boundary.size(); ++i) {
    Point3d newPt(boundary[i][0], 0, boundary[i][1]);
    ptvec.push_back(newPt);
  }

  //add in center
  Point3d center3(center[0], 0, center[1]);
  ptvec.push_back(center3);
  int centerID = ptvec.size() - 1;

  //push the same triangles in for 3d (at predefined height)
  double height = 1.0;
  for(unsigned int i = 0; i < boundary.size(); ++i) {
    Point3d newPt(boundary[i][0], height, boundary[i][1]);
    ptvec.push_back(newPt);
  }

  //add in center -- 3d
  Point3d center3_2(center[0], height, center[1]);
  ptvec.push_back(center3_2);
  int centerID_2 = ptvec.size() - 1;

  //make triangles
  using Tri = IModel::Tri;
  using TriVector = IModel::TriVector;
  TriVector& triP = byuloader->GetTriP();
  for(unsigned int i = 1; i < boundary.size(); ++i) {
    int id1 = i - 1;
    int id2 = i;
    Tri tri(id1, id2, centerID);
    triP.push_back(tri);
    if(i == boundary.size() - 1) {
      //push in last one (that wraps around)
      id1 = i;
      id2 = 0;
      Tri tri(id1, id2, centerID);
      triP.push_back(tri);
    }
  }

  int index2 = boundary.size() + 2;
  for(unsigned int i = index2; i < ptvec.size() - 1; ++i) {
    int id1 = i - 1;
    int id2 = i;
    Tri tri(id1, id2, centerID_2);
    triP.push_back(tri);
    if(i == ptvec.size() - 2) {
      //push in last one (that wraps around)
      id1 = i;
      id2 = boundary.size() + 1;
      Tri tri(id1, id2, centerID_2);
      triP.push_back(tri);
    }
  }

  byuloader->ComputeFaceNormal();
  cout << " done!--CreateModelLoaderFromPts." << endl;
  return byuloader;
}

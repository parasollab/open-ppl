#include <cstdio>
#include <fstream>

#include "BVHDataLoader.h"

using namespace std;
using namespace mathtool;

/*------------------------------ CBVHDataLoader ------------------------------*/

void
CBVHDataLoader::
load(ifstream& _in) {
  string first, stmp;
  int itmp, keyframenum;

  if(_in >> keyframenum)
    cout << " Reading KF: " << keyframenum << endl;
  else {
    cout << " Problem reading keyframe NUM." << endl;
    exit(-1);
  }

  while(_in >> first) {
    if(first == "edge") {
      _in >> itmp >> stmp;
      Point3d p1, p2;
      if(stmp == "p1" && !(_in >> p1))
        cout << "Problem reading p1" << endl;
      _in >> stmp;
      if(stmp == "p2" && !(_in >> p2))
        cout << "Problem reading p2" << endl;
      Edge e(p1,p2);
      m_edgeList.push_back(e);
    }
    else if(first == "endkeyframe")
      break;
    else
      cout << "ERROR: unknown option: " << first << endl;
  }
}


bool
CBVHDataLoader::
ParseFile(bool) {
   cout << " CBVHDataLoader::ParseFile --> does not ParseFile the way the "
        << "other loaders do..." << endl;
   return true;
}

/*----------------------------- CBVHDataInstance -----------------------------*/

static int BVH_dataInstanceIndex = 0;

void
CBVHDataInstance::
load(string _file, double _radius, double _height) {
   m_id = BVH_dataInstanceIndex++;
   setFileName(_file);

   cout << "CBVHDataInstance--> reading: " << _file << " instance index: "
        << m_id << endl;

   // Assert that file exists and can be opened.
   ifstream infile(_file);
   if(!infile) {
      cerr << "File: '" << _file << "' does not exist." << endl;
      exit(0);
   }
   // Get file extension.
   size_t pos = _file.rfind('.');
   if(pos == string::npos) {
     cerr << "Error: unrecognized file extension for '" << _file << "'" << endl;
     exit(0);
   }
   string ext = _file.substr(pos+1);

   string first, stmp;
   // Load all models from same file (bvh_data)
   while(infile >> first) {
      if(first == "keyframe") { //load keyframe
	 IModel* model = new CBVHDataLoader();
	 if(model == NULL) {
	    cout << " mode eq NULL(in bvh_data)...exit!" << endl;
	    exit(0);
	 }
	 ((CBVHDataLoader*)model)->load(infile);
	 model->skelBBX(((CBVHDataLoader*)model)->GetEdgeList());
	 m_models.push_back(model);
      }
      else {
	 cout << "Should be done reading file '" << _file << "', but more lines"
              << " are present!" << endl;
	 break;
      }
   }

   // find transform (might just want a function to find transform
   // need direction, and compute theta
   Vector2d W_dir(0, 1);//corresponds to x-z dir, y is for height
   for(int i = 0; i < (int)m_models.size(); ++i) {
      IModel* model_i = m_models[i];
      IModel* model_next;
      if(i == ((int)m_models.size() - 1)) {
	 //just use previous
         Vector3d& dir = model_i->GetDir();
         dir = m_models[i - 1]->GetDir();
	 Vector2d dir2d(dir[0], dir[2]);
	 dir2d = dir2d.normalize();
	 double theta_rad = acos(dir2d * W_dir);
	 model_i->setRotRAD(theta_rad);
      }
      else {
	 model_next = m_models[i + 1];
	 model_i->genDir(((CBVHDataLoader*)model_i)->GetEdgeList(),
             ((CBVHDataLoader*)model_next)->GetEdgeList(), 0);
	 Vector2d dir2d(model_i->GetDir()[0], model_i->GetDir()[2]);
	 dir2d = dir2d.normalize();
	 double theta_rad = acos(dir2d * W_dir);
	 model_i->setRotRAD(theta_rad);
      }
   }

   // transform not currently implemented.
   //for(int i = 0; i < (int)m_models.size(); ++i) {
   //   IModel* model_i = m_models[i];
   //   model_i->transform(((CBVHDataLoader*)model_i)->GetEdgeList(), _radius,
   //       _height);
   //}
}

/*--------------------------- CBVHDataModelFactory ---------------------------*/

size_t
CBVHDataModelFactory::
numInstances() {
  return m_BVHDataInstances.size();
}


void
CBVHDataModelFactory::
addInstance(CBVHDataInstance* _i) {
  m_BVHDataInstances.push_back(_i);
}


CBVHDataInstance*
CBVHDataModelFactory::
getInstance(size_t _index) {
  return m_BVHDataInstances[_index];
}


bool
CBVHDataModelFactory::
hasBVHDataInstance(const std::string& _filename, int& _index) {
  for(int i = 0; i < static_cast<int>(m_BVHDataInstances.size()); ++i) {
    if(m_BVHDataInstances[i]->getFileName() == _filename) {
      _index = i;
      return true;
    }
  }
  _index = -1;
  return false;
}

/*----------------------------------------------------------------------------*/

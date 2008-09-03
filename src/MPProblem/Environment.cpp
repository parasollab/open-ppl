#include "Environment.h"
#include "MPProblem.h"
#include <fstream>
#include <iostream>
#include "util.h"


//////////////////////////////////////////////////////////////////////////////////
/**@name Format version for environment (*.env) files
  *      The number breaks down as YearMonthDay so numerical
  *      comparisons can be more easily made.
  *@warning Be consistent.  It should be YYYYMMDD
  *      Inconsistent conversions can be misleading.
  *      For example, comparing 200083  to 20000604.
  */
//@{

//defined by Xinyu Tang, 03/27/2002
//Objective: To enable the obprm to distinguish the external & internal obstacles, 
//           so it would not try to generate nodes on the surfaces of the internal
//           obstacles, which might save a lot of time for obprm;
#define ENV_VER_20020327                   20020327

#define ENV_VER_20001022                   20001022
#define ENV_VER_LEGACY                     0
//@}


//===================================================================
//  Constructors
//===================================================================

Environment::
Environment(int dofs, int pos_dofs) {
  boundaries = new BoundingBox(dofs,pos_dofs);

  pathVersion = PATHVER_20001125;

  multibody.clear();
  externalbodyCount = 0;
  minmax_BodyAxisRange = 0;

  usable_multibody.clear();
  usable_externalbody_count = 0;
  positionRes = POSITION_RES_FACTOR;
  orientationRes = ORIENTATION_RES;

  robotIndex = 0;
  copied_instance = false;

}

/*
Environment::
Environment(int dofs, int pos_dofs, Input * _input) {
  boundaries = new BoundingBox(dofs,pos_dofs);

  pathVersion = PATHVER_20001125;

  multibody.clear();
  externalbodyCount = 0;

  usable_multibody.clear();
  usable_externalbody_count = 0;

  positionRes = POSITION_RES_FACTOR;
  orientationRes = ORIENTATION_RES;

  robotIndex = 0;
  copied_instance = false;

  if (_input != NULL) {
    Read(_input->envFile.GetValue(), EXIT,
	 _input->descDir.GetValue(), _input->cdtype, _input->nprocs);
    FindBoundingBox();
    
    //   if user supplied a bounding box, use it instead
    if (_input->bbox.IsActivated()) {
      std::stringstream i_bbox;
      i_bbox << _input->bbox.GetValue();
      boundaries->Parse(i_bbox);  
    }

    // if user supplied a positional resolution, use it instead
    if ( _input->posres.IsActivated() )
      positionRes = _input->posres.GetValue();
    
    // orientational resolution may be user supplied but at this time
    // is not calculated
    if ( _input->orires.IsActivated() )
      orientationRes = _input->orires.GetValue(); 
    
    // scale boundary
    if ( _input->bbox_scale.IsActivated() )
      boundaries->TranslationalScale(_input->bbox_scale.GetValue());  
  
    // activate objects inside the bounding box and deactivate other
    // objects
    SelectUsableMultibodies();  
  }
}
*/

/**
 * Copy Constructor
 */
Environment::
Environment(Environment &from_env) {
  //boundaries(*(from_env.GetBoundingBox())) {
  boundaries = new BoundingBox(*(from_env.GetBoundingBox()));
  pathVersion = PATHVER_20001125;


  // only usable bodies in from_env will be copied
  multibody.clear();
  usable_multibody.clear();
  for (int i = 0; i < from_env.GetMultiBodyCount(); i++) {
    multibody.push_back(from_env.GetMultiBody(i));
    usable_multibody.push_back(from_env.GetMultiBody(i));
  }

  externalbodyCount = from_env.GetExternalBodyCount();
  usable_externalbody_count = from_env.GetExternalBodyCount();
  positionRes = from_env.GetPositionRes();
  orientationRes = from_env.GetOrientationRes();
  robotIndex = from_env.GetRobotIndex();
  copied_instance = true;
}


/**
 * Copy Constructor, copies from MPProblem's environemnt
 */
Environment::
Environment(MPProblem* in_pProblem) : MPBaseObject(in_pProblem) {
  Environment& from_env = *(in_pProblem->GetEnvironment());
  //boundaries(*(from_env.GetBoundingBox())) {
  boundaries = new BoundingBox(*(from_env.GetBoundingBox()));
  pathVersion = PATHVER_20001125;


  // only usable bodies in from_env will be copied
  multibody.clear();
  usable_multibody.clear();
  for (int i = 0; i < from_env.GetMultiBodyCount(); i++) {
    multibody.push_back(from_env.GetMultiBody(i));
    usable_multibody.push_back(from_env.GetMultiBody(i));
  }

  externalbodyCount = from_env.GetExternalBodyCount();
  usable_externalbody_count = from_env.GetExternalBodyCount();
  positionRes = from_env.GetPositionRes();
  orientationRes = from_env.GetOrientationRes();
  robotIndex = from_env.GetRobotIndex();
  copied_instance = true;
}


/**
 * Copy Constructor
 * receiving a bounding box (not necessarily the same as the original
 * environment
 */
Environment::
Environment(Environment &from_env, BoundingBox &i_boundaries) {
 
  boundaries = new BoundingBox(i_boundaries);

  pathVersion = PATHVER_20001125;

  multibody.clear();
  externalbodyCount = 0;

  // only usable bodies in from_env will be copied
  multibody.clear();
  for (int i = 0; i < from_env.GetMultiBodyCount(); i++) {
    multibody.push_back(from_env.GetMultiBody(i));
  }

  positionRes = from_env.GetPositionRes();
  orientationRes = from_env.GetOrientationRes();

  externalbodyCount = from_env.GetExternalBodyCount();
  robotIndex = from_env.GetRobotIndex();
  
  SelectUsableMultibodies(); // select usable multibodies

  copied_instance = true;
}

///\brief Constructor taking in an XML node for parsing
///\todo Fix hack to input Env file ... Shouln't use Input Class.
///\todo Fix boundaries init

Environment::
Environment(XMLNodeReader& in_Node,  MPProblem* in_pProblem) : MPBaseObject(in_Node, in_pProblem) {
    LOG_DEBUG_MSG("Environment::Environment()");
    pathVersion = PATHVER_20001125;

    multibody.clear();
    externalbodyCount = 0;

    usable_multibody.clear();
    usable_externalbody_count = 0;

    positionRes = POSITION_RES_FACTOR;
    orientationRes = ORIENTATION_RES;

    robotIndex = 0;
    copied_instance = false;
    
    in_Node.verifyName(string("environment"));
    

    //Read(in_Node.stringXMLParameter("input_env", true, "", "env filename").c_str(), EXIT, "", RAPID, 1);
    Read(in_Node.stringXMLParameter("input_env", true, "", "env filename").c_str(), EXIT, "");
    //FindBoundingBox();

      //compute RESOLUTION

        double robot_span;
        multibody[robotIndex]->FindBoundingBox();
        robot_span = multibody[robotIndex]->GetMaxAxisRange();
        double bodies_min_span;
        bodies_min_span = robot_span;
    
        bool first = true;
        double * tmp;
        for(int i = 0 ; i < multibody.size() ; i++){
        if(i != robotIndex){
          if(first){
            multibody[i]->FindBoundingBox();
            first = false;
            bodies_min_span = min(bodies_min_span,multibody[i]->GetMaxAxisRange());
          }
          else{
            multibody[i]->FindBoundingBox();
            bodies_min_span = min(bodies_min_span,multibody[i]->GetMaxAxisRange());
          } 
        }
      }
  
      double min_clearance = robot_span/3.0;
      positionRes = bodies_min_span * POSITION_RES_FACTOR;
      minmax_BodyAxisRange = bodies_min_span;

      cout << "Position Resolution = " << positionRes << endl;
      // END compute RESOLUTION

    XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
      if(citr->getName() == "robot") {
        int num_joints = citr->numberXMLParameter(string("num_joints"),true,0,0,MAX_INT,string("num_joints"));
        in_pProblem->SetNumOfJoints(num_joints);
        
        XMLNodeReader::childiterator citr2;
        for(citr2 = citr->children_begin(); citr2!= citr->children_end(); ++citr2) {
          if (citr2->getName() == "boundary") {
            boundaries = new BoundingBox(*citr2,in_pProblem); 
            //@todo assumption of input bbox not strong. When no bbox provided call FindBoundingBox()
          } else {
            citr2->warnUnknownNode();
          }
        }
  
      } else if(citr->getName() == "resolution") {
        //pChild->ToElement()->QueryDoubleAttribute("pos_res",&positionRes);
        //pChild->ToElement()->QueryDoubleAttribute("ori_res",&orientationRes);
      } else {
        citr->warnUnknownNode();
      }
    }

    SelectUsableMultibodies();
    LOG_DEBUG_MSG("~Environment::Environment()");
}

void Environment::
PrintOptions(ostream& out_os) {
  out_os << "  Environment" << endl;
  out_os << "    positionRes = " << positionRes << "; orientationRes = " << orientationRes << endl;
  out_os << "    bbox = ";
  boundaries->Print(out_os);
  out_os << endl;
}



//===================================================================
//  Destructor
//===================================================================
Environment::
~Environment() {
  LOG_DEBUG_MSG("Environment::~Environment()");
  usable_multibody.clear();

  // release memory from multibody if this instance was not copied
  // from other.
  if (!copied_instance)
    for (int i=0; i < multibody.size(); i++) {
      delete multibody[i];
    }
  LOG_DEBUG_MSG("~Environment::~Environment()");
}


//============================================
//SortBodies so that the external bodies appear first in the array
//============================================
void 
Environment::
SortMultiBodies(){
  externalbodyCount = 1; // the robot counts as an external body
  if(multibody.size() != 1) { //the robot is not the only object
    int i = 0;
    int j = multibody.size()-1;
    while (i < j) {
      //Quicksort
      while((i<multibody.size()) && !multibody[i]->IsInternal()) 
  i++;
      while ((j>0) && (multibody[j]->IsInternal()))
  j--;
      if (i<j) {
  MultiBody *pMidBody = multibody[j];//switch multibody[i] & multibody[j]
  multibody[j] = multibody[i];
  multibody[i] = pMidBody;
      }
    }
    if (i == j+1)
      externalbodyCount = i;
    else
      cout << "Wrong sorting in void Environment::SortMultiBodies(){}"<<endl;
  }
}


//Get rid of obstacles outside the bounding box
void 
Environment::
SelectUsableMultibodies() {
  // get workspace bounding box
  double minx = boundaries->GetRange(0).first; 
  double maxx = boundaries->GetRange(0).second;
  double miny = boundaries->GetRange(1).first; 
  double maxy = boundaries->GetRange(1).second;
  double minz = boundaries->GetRange(2).first; 
  double maxz = boundaries->GetRange(2).second;  

  //save original robot index
  int rob = robotIndex;  
  usable_multibody.clear();
  usable_externalbody_count = 0;
  for (int i = 0; i < multibody.size(); i++) {
    if (i == rob) { // @todo: need a test function in multibody to
        // decide if a multibody is a robot
      usable_multibody.push_back(multibody[i]);
      usable_externalbody_count++; // robot is an external body
      robotIndex = usable_multibody.size()-1;
    } else {
      //see if bounding box of multibody overlaps BB
      multibody[i]->FindBoundingBox();
      double *obb = multibody[i]->GetBoundingBox();
      
      if (((obb[0] <= maxx && obb[0] >= minx) || 
     (obb[1] <= maxx && obb[1] >= minx)) &&
    ((obb[2] <= maxy && obb[2] >= miny) || 
     (obb[3] <= maxy && obb[3] >= miny)) &&
    ((obb[4] <= maxz && obb[4] >= minz) || 
     (obb[5] <= maxz && obb[5] >= minz))) {
  // any point in obstacle's bbox inside boundaries => obstacle is usable
  usable_multibody.push_back(multibody[i]);
  if (!(multibody[i]->IsInternal()))
    usable_externalbody_count++;
      } else { // bounding boxes cross each other 
  if (!(obb[0] > maxx || obb[1] < minx || 
        obb[2] > maxy || obb[3] < miny || 
        obb[4] > maxz || obb[5] < minz)) {
    usable_multibody.push_back(multibody[i]);
    if (!(multibody[i]->IsInternal()))
      usable_externalbody_count++;
  }
      }
    }
  }

}

//===================================================================
//  Write
//
//  Function: Write the Input data for an environment into a 
//  file
//
//===================================================================
void 
Environment::
Write(ostream & _os) {
    _os << usable_multibody.size() << endl;
    for (int i=0; i < usable_multibody.size(); i++)
        usable_multibody[i]->Write(_os);
}

void 
Environment::
FindBoundingBox(){
  double robot_span;
  multibody[robotIndex]->FindBoundingBox();
  robot_span = multibody[robotIndex]->GetMaxAxisRange();
  double bodies_min_span;
  bodies_min_span = robot_span;
    
  bool first = true;
  double * tmp;
  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = maxx = maxy = maxz = 0;
  for(int i = 0 ; i < multibody.size() ; i++){
    if(i != robotIndex){
      if(first){
  multibody[i]->FindBoundingBox();
  tmp = multibody[i]->GetBoundingBox();
  minx = tmp[0]; maxx = tmp[1];
  miny = tmp[2]; maxy = tmp[3];
  minz = tmp[4]; maxz = tmp[5];
  first = false;
  bodies_min_span = min(bodies_min_span,multibody[i]->GetMaxAxisRange());
      }
      else{
  multibody[i]->FindBoundingBox();
  tmp = multibody[i]->GetBoundingBox();
  minx = min(minx,tmp[0]); maxx = max(maxx,tmp[1]);
  miny = min(miny,tmp[2]); maxy = max(maxy,tmp[3]);
  minz = min(minz,tmp[4]); maxz = max(maxz,tmp[5]);
  bodies_min_span = min(bodies_min_span,multibody[i]->GetMaxAxisRange());
      }
    }
  }
  
  double min_clearance = robot_span/3.0;
  vector<double> boundingBox;
  boundingBox.push_back(minx-min_clearance); 
  boundingBox.push_back(maxx+min_clearance);
  boundingBox.push_back(miny-min_clearance); 
  boundingBox.push_back(maxy+min_clearance);
  boundingBox.push_back(minz-min_clearance); 
  boundingBox.push_back(maxz+min_clearance);
  
  boundaries->SetRanges(boundingBox);
  boundaries->TranslationalScale(2); ///\todo fix this default.
  //defaults bbox_scale to 2 when no bbox is defined.

  positionRes = bodies_min_span * POSITION_RES_FACTOR;
  minmax_BodyAxisRange = bodies_min_span;
}

//===================================================================
//  Get_minmax_BodyAxisRange
//===================================================================
double 
Environment::
Getminmax_BodyAxisRange(){
    return minmax_BodyAxisRange;
}

BoundingBox *
Environment::GetBoundingBox() {
  return boundaries;
}



void 
Environment::
Read(const char* in_filename, int action, const char* descDir) {  
  input_filename = string(in_filename);

  VerifyFileExists(in_filename,action);
  
  // open file and read first field
  ifstream is(in_filename);
  
#define LINEMAX 256
  // if first field is a comment delimiter
  char t;
  int envFormatVersion = ENV_VER_LEGACY;
  while((t = is.peek()) == '#') {
    char line[LINEMAX];
    is.getline(line,LINEMAX,'\n');
    char string1[32];
    char string2[32];
    char string3[32];
    if(strstr(line, "Environment")) {
      sscanf(&line[1],"%s %s %d",string2,string3,&envFormatVersion);
      if(!strstr(string3, "Version")) {
	cerr << "\nREAD ERROR: bad file format in \""
	     << in_filename << "\"";
	cerr << "\n            something is wrong w/ the following\n"
	     << "\n            "<<string1<<" "<<string2<<" "<<string3
	     <<"\n\n";
	if(action==EXIT)
	  exit(-1);
      }
    } 
  }
  Read(is, envFormatVersion, action, descDir);
  is.close();
}


void 
Environment::
Read(istream & _is, int envFormatVersion, int action, const char* descDir) {  
  switch(envFormatVersion) {
  case ENV_VER_20020327:
    break;
  case ENV_VER_20001022:
    // put in whatever may be specific to the format
    // ENV_VER_20001022 is equivalent to ENV_VER_LEGACY
    // so nothing is specific here
    break;
  case ENV_VER_LEGACY:
    break;
  default:
    cerr << "\nREAD ERROR: Unrecognized Environment Version \""
	 << envFormatVersion << "\""
	 <<"\n\n";
    exit(-1);
    break;
  }
 
  int multibodyCount; 
  _is >> multibodyCount;      // # of MultiBodys'
  for (int m=0; m<multibodyCount; m++) {    
    MultiBody * mb = new MultiBody(this);
    mb->Read(_is, action, descDir);
    multibody.push_back(mb);
  }
}


void
Environment::
buildCDstructure(cd_predefined cdtype, int nprocs)
{
  for(vector<MultiBody*>::iterator M = multibody.begin(); M != multibody.end(); ++M)
    (*M)->buildCDstructure(cdtype, nprocs);
}

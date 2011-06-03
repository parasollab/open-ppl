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
Environment(int dofs, int pos_dofs) :
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(0),
  robotIndex(0),
  positionRes(POSITION_RES_FACTOR),
  orientationRes(ORIENTATION_RES),
  minmax_BodyAxisRange(0),
  input_filename("")
{
  boundaries = shared_ptr<BoundingBox>(new BoundingBox(dofs, pos_dofs));
}


/*
Environment::
Environment(int dofs, int pos_dofs, Input * _input) :
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(0),
  robotIndex(0),
  positionRes(POSITION_RES_FACTOR),
  orientationRes(ORIENTATION_RES),
  minmax_BodyAxisRange(0),
  input_filename("")
{
  boundaries = new BoundingBox(dofs, pos_dofs);

  if (_input != NULL) {
    Read(_input->envFile.GetValue(), PMPL_EXIT,
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
Environment(const Environment &from_env) :
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(from_env.usable_externalbody_count),
  robotIndex(from_env.robotIndex),
  positionRes(from_env.positionRes),
  orientationRes(from_env.orientationRes),
  minmax_BodyAxisRange(from_env.minmax_BodyAxisRange),
  input_filename(from_env.input_filename)
{
  boundaries = shared_ptr<BoundingBox>(new BoundingBox(*(from_env.GetBoundingBox())));
  if(boundaries->GetDOFs() + boundaries->GetPosDOFs() == 0)
    cout << "FOUND EMPTY BBOX! (Environment copy constructor)\n";

  // only usable bodies in from_env will be copied
  for (int i = 0; i < from_env.GetMultiBodyCount(); i++) {
    multibody.push_back(from_env.GetMultiBody(i));
    //usable_multibody.push_back(from_env.GetMultiBody(i));
  }
  
  SelectUsableMultibodies();
}


/**
 * Copy Constructor, copies from MPProblem's environemnt
 */
Environment::
Environment(MPProblem* in_pProblem) : 
  MPBaseObject(in_pProblem),
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(in_pProblem->GetEnvironment()->usable_externalbody_count),
  robotIndex(in_pProblem->GetEnvironment()->robotIndex),
  positionRes(in_pProblem->GetEnvironment()->positionRes),
  orientationRes(in_pProblem->GetEnvironment()->orientationRes),
  minmax_BodyAxisRange(in_pProblem->GetEnvironment()->minmax_BodyAxisRange),
  input_filename(in_pProblem->GetEnvironment()->input_filename)
{
  Environment& from_env = *(in_pProblem->GetEnvironment());
  
  boundaries = shared_ptr<BoundingBox>(new BoundingBox(*(from_env.GetBoundingBox())));
  if(boundaries->GetDOFs() + boundaries->GetPosDOFs() == 0)
    cout << "FOUND EMPTY BBOX! (MPProblem constructor) \n";

  for (int i = 0; i < from_env.GetMultiBodyCount(); i++) {
    multibody.push_back(from_env.GetMultiBody(i));
    //usable_multibody.push_back(from_env.GetMultiBody(i));
  }
  
  SelectUsableMultibodies();
}


/**
 * Copy Constructor
 * receiving a bounding box (not necessarily the same as the original
 * environment
 */
Environment::
Environment(const Environment &from_env, const BoundingBox &i_boundaries) :
  pathVersion(PATHVER_20001125),
  robotIndex(from_env.robotIndex),
  positionRes(from_env.positionRes),
  orientationRes(from_env.orientationRes),
  minmax_BodyAxisRange(from_env.minmax_BodyAxisRange),
  input_filename(from_env.input_filename)
{
  boundaries = shared_ptr<BoundingBox>(new BoundingBox(i_boundaries));

  for (int i = 0; i < from_env.GetMultiBodyCount(); i++) {
    multibody.push_back(from_env.GetMultiBody(i));
  }
  
  SelectUsableMultibodies(); // select usable multibodies
}

///\brief Constructor taking in an XML node for parsing
///\todo Fix hack to input Env file ... Shouln't use Input Class.
///\todo Fix boundaries init

Environment::
Environment(XMLNodeReader& in_Node,  MPProblem* in_pProblem) : 
  MPBaseObject(in_Node, in_pProblem),
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(0),
  robotIndex(0),
  positionRes(POSITION_RES_FACTOR),
  orientationRes(ORIENTATION_RES),
  minmax_BodyAxisRange(0),
  input_filename("")
{
    LOG_DEBUG_MSG("Environment::Environment()");

    in_Node.verifyName(string("environment"));

    ///\todo fix hack.  This hack gets env_filename from environment xml tag
    //const char* env_filename = in_pNode->ToElement()->Attribute("input_env");
    //const char* env_filename = GetMPProblem()->GetEnvFileName().c_str();
    //Read(env_filename, PMPL_EXIT, "", RAPID, 1);
    ///\todo fix hack. This hack assigns RAPID as the cd library and the main directory as "".
    Read(in_Node.stringXMLParameter("input_env", true, "", "env filename").c_str(), PMPL_EXIT, "");
    //FindBoundingBox();

    //compute RESOLUTION

    multibody[robotIndex]->FindBoundingBox();
    double robot_span = multibody[robotIndex]->GetMaxAxisRange();
    double bodies_min_span = robot_span;
    
    for(size_t i = 0 ; i < multibody.size() ; i++){
      if((int)i != robotIndex){
        multibody[i]->FindBoundingBox();
        bodies_min_span = min(bodies_min_span,multibody[i]->GetMaxAxisRange());
      }
    }
  
    positionRes = bodies_min_span * POSITION_RES_FACTOR;
    minmax_BodyAxisRange = bodies_min_span;
 
    // END compute RESOLUTION
 
      XMLNodeReader::childiterator citr;
      for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
        if(citr->getName() == "robot") {
          int num_joints = citr->numberXMLParameter(string("num_joints"),true,0,0,MAX_INT,string("num_joints"));
          in_pProblem->SetNumOfJoints(num_joints);
         
          XMLNodeReader::childiterator citr2;
          for(citr2 = citr->children_begin(); citr2!= citr->children_end(); ++citr2) {
            if (citr2->getName() == "boundary") {
              boundaries = shared_ptr<BoundingBox>(new BoundingBox(*citr2,in_pProblem)); 
              //@todo assumption of input bbox not strong. When no bbox provided call FindBoundingBox()
            } else {
              citr2->warnUnknownNode();
            }
          }
  
        } else if(citr->getName() == "resolution") {
          double pos_res = citr->numberXMLParameter("pos_res", false, -1.0, -1.0, double(MAX_INT), "position resolution");
          if(pos_res != -1.0)
            positionRes = pos_res;
          double ori_res = citr->numberXMLParameter("ori_res", false, -1.0, -1.0, double(MAX_INT), "orientation resolution");
          if(ori_res != -1.0)
            orientationRes = pos_res;
        } else {
          citr->warnUnknownNode();
        }
      }
 
      cout << "Position Resolution = " << positionRes << endl;
    
    SelectUsableMultibodies();
    LOG_DEBUG_MSG("~Environment::Environment()");
}



///////////////////////////////

Environment::
Environment(const Environment& from_env, string filename) : 
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(0),
  robotIndex(0),
  positionRes(from_env.positionRes),
  orientationRes(from_env.orientationRes),
  minmax_BodyAxisRange(0),
  input_filename(filename)
{
  boundaries = shared_ptr<BoundingBox>(new BoundingBox(*(from_env.GetBoundingBox())));
  if(boundaries->GetDOFs() + boundaries->GetPosDOFs() == 0)
    cout << "FOUND EMPTY BBOX! (Environment copy constructor)\n";
         
  Read(filename.c_str(), PMPL_EXIT, "");
  FindBoundingBox();

  //compute RESOLUTION
  multibody[robotIndex]->FindBoundingBox();
  double bodies_min_span = multibody[robotIndex]->GetMaxAxisRange();
    
  bool first = true;
  for(size_t i = 0 ; i < multibody.size() ; i++){
    if((int)i != robotIndex){
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
  minmax_BodyAxisRange = bodies_min_span;
  // END compute RESOLUTION

  SelectUsableMultibodies();
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
  LOG_DEBUG_MSG("~Environment::~Environment()");
}


//============================================
//SortBodies so that the external bodies appear first in the array
//============================================
void 
Environment::
SortMultiBodies(){
  if(multibody.size() != 1) { //the robot is not the only object
    int i = 0;
    int j = multibody.size()-1;
    while (i < j) {
      //Quicksort
      while((i<(int)multibody.size()) && !multibody[i]->IsInternal()) 
        i++;
      while ((j>0) && (multibody[j]->IsInternal()))
        j--;
      if (i<j) {
        shared_ptr<MultiBody> pMidBody = multibody[j];//switch multibody[i] & multibody[j]
        multibody[j] = multibody[i];
        multibody[i] = pMidBody;
      }
    }
    if (i != j+1)
      cout << "Wrong sorting in void Environment::SortMultiBodies(){}"<<endl;
  }
}


//Get rid of obstacles outside the bounding box
void 
Environment::
SelectUsableMultibodies() {
  usable_multibody.clear();
  usable_externalbody_count = 0;

  //add robot to list of usable_multibodies
  int original_robotIndex = robotIndex;
  if(robotIndex < (int)multibody.size())
  {
    usable_multibody.push_back(multibody[robotIndex]);
    robotIndex = usable_multibody.size()-1;
    usable_externalbody_count++;
  }

  if(boundaries->GetDOFs() < 3)
    return;

  // get workspace bounding box
  double minx = boundaries->GetRange(0).first; 
  double maxx = boundaries->GetRange(0).second;
  double miny = boundaries->GetRange(1).first; 
  double maxy = boundaries->GetRange(1).second;
  double minz = boundaries->GetRange(2).first; 
  double maxz = boundaries->GetRange(2).second;  

  for (size_t i = 0; i < multibody.size(); i++) 
    if((int)i != original_robotIndex) { // @todo: need a test function in multibody to
      //see if bounding box of multibody overlaps BB
      multibody[i]->FindBoundingBox();
      const double *obb = multibody[i]->GetBoundingBox();
      
      if (((obb[0] <= maxx && obb[0] >= minx) || (obb[1] <= maxx && obb[1] >= minx)) &&
          ((obb[2] <= maxy && obb[2] >= miny) || (obb[3] <= maxy && obb[3] >= miny)) &&
          ((obb[4] <= maxz && obb[4] >= minz) || (obb[5] <= maxz && obb[5] >= minz))) {
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
    for (size_t i=0; i < usable_multibody.size(); i++)
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
  const double * tmp;
  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = maxx = maxy = maxz = 0;
  for(size_t i = 0 ; i < multibody.size() ; i++){
    if((int)i != robotIndex){
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

shared_ptr<BoundingBox>
Environment::GetBoundingBox() const {
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
	if(action==PMPL_EXIT)
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
    shared_ptr<MultiBody> mb(new MultiBody());
    mb->Read(_is, action, descDir);
    multibody.push_back(mb);
  }
}


void
Environment::
buildCDstructure(cd_predefined cdtype, int nprocs)
{
  for(vector<shared_ptr<MultiBody> >::iterator M = multibody.begin(); M != multibody.end(); ++M)
    (*M)->buildCDstructure(cdtype, nprocs);
}


bool
Environment::
operator==(const Environment& e) const
{
  if(multibody.size() != e.multibody.size())
    return false;
  for(size_t i=0; i<multibody.size(); ++i)
    if(*(multibody[i]) != (*(e.multibody[i])))
      return false;

  if(usable_multibody.size() != e.usable_multibody.size())
    return false;
  for(size_t i=0; i<usable_multibody.size(); ++i)
    if(*(usable_multibody[i]) != (*(e.usable_multibody[i])))
      return false;

  return (pathVersion == e.pathVersion) &&
         (usable_externalbody_count == e.usable_externalbody_count) &&
         (robotIndex == e.robotIndex) &&
         (*boundaries == *e.boundaries) &&
         (positionRes == e.positionRes) &&
         (orientationRes == e.orientationRes) &&
         (minmax_BodyAxisRange == e.minmax_BodyAxisRange);
}

#include "Environment.h"
#include "MPProblem.h"
#include <fstream>
#include <iostream>
//#include "tinyxml.h"
#include "util.h"


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
    _input->Read(EXIT); // read only input
    GetBodies(_input);
    FindBoundingBox();
    
    //	 if user supplied a bounding box, use it instead
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
Environment(TiXmlNode* in_pNode,  MPProblem* in_pProblem) : MPBaseObject(in_pNode, in_pProblem) {
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
    
    if(!in_pNode) {
      LOG_ERROR_MSG("Error reading <environment> tag...."); exit(-1);
    }
    if(string(in_pNode->Value()) != "environment") {
      LOG_ERROR_MSG("Error reading <environment> tag...."); exit(-1);
    }

    ///\todo fix hack.  This hack gets env_filename from environment xml tag
    //const char* env_filename = in_pNode->ToElement()->Attribute("input_env");
    const char* env_filename = GetMPProblem()->GetEnvFileName().c_str();
    ///\todo fix hack.  This hack creates a temp Input to parse environment file.
    Input* pinput;
    pinput = new Input;
    ///\todo fix hack. This hack assigns RAPID as the cd library
    pinput->cdtype = RAPID;
    
    pinput->Read(env_filename,EXIT);
    
    GetBodies(pinput);
    //    FindBoundingBox();

    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "robot") {
	int num_joints;
	pChild->ToElement()->QueryIntAttribute("num_joints", &num_joints);
	in_pProblem->SetNumOfJoints(num_joints);

	for ( TiXmlNode* rChild = pChild->FirstChild(); rChild != 0; rChild = rChild->NextSibling() ) {
	  if (string(rChild->Value()) == "boundary") {
	    boundaries = new BoundingBox(rChild,in_pProblem); //@todo assumption of input bbox not strong. When no bbox provided call FindBoundingBox()
	  } else if(!rChild->Type() == TiXmlNode::COMMENT) {
	    cout << "  I don't know: " << *pChild << endl;
	  }
	  
	}
	
      } else if(string(pChild->Value()) == "resolution") {
	pChild->ToElement()->QueryDoubleAttribute("pos_res",&positionRes);
	pChild->ToElement()->QueryDoubleAttribute("ori_res",&orientationRes);
	
      } else {
        if(!pChild->Type() == TiXmlNode::COMMENT) {
          cout << "  I don't know: " << *pChild << endl;
        }
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

void Environment::
GetBodies(Input * _input) {
  // read multibodies in the environment (robot and obstacles)
  multibody.clear();
  externalbodyCount = 0;
  for (int i = 0; i < _input->multibodyCount; i++) {
    MultiBody * mb = new MultiBody(this);
    mb->Get(_input, i);
    multibody.push_back(mb);
  }
  // put the external bodies in the beginning part of the multibody array;
  SortMultiBodies();
  // compute bounding box and positional resolution

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

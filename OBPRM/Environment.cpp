// $Id$
/////////////////////////////////////////////////////////////////////
//  Environment.c
//
//  Created   3/ 1/98 Aaron Michalk
//
/////////////////////////////////////////////////////////////////////

#include "Environment.h"

///////////////////////////////////////////////////////////////////////////
#include "MultiBody.h"
#include "Input.h"
#include "OBPRMDef.h"	//For POSITION_RES_FACTOR , which is used in this file

#define DOF   6

//===================================================================
//  Constructors
//===================================================================
Environment::Environment() {
    pathVersion = PATHVER_20001125;    // Format version for path files
    multibodyCount = 0;
    externalbodyCount = 0;
    multibody = 0;
    robotIndex = 0;
}
Environment::Environment(int index) {
    pathVersion = PATHVER_20001125;    // Format version for path files
    multibodyCount = 0;
    externalbodyCount = 0;
    multibody = 0;
    robotIndex = index;
}

//===================================================================
//  Destructor
//===================================================================
Environment::~Environment() {
    for (int i=0; i < multibodyCount; i++)
        delete multibody[i];
}

//===================================================================
//  AddMultiBody
//===================================================================
void Environment::AddMultiBody(MultiBody * _multibody) {
    multibodyCount++;
    multibody = (MultiBody **)realloc(multibody, multibodyCount * sizeof(MultiBody *));
    multibody[multibodyCount-1] = _multibody;
}

//===================================================================
//  Couple
//===================================================================
void Environment::Couple(MultiBody *_multibody[], int number) {
}

//===================================================================
//  Decouple
//===================================================================
void Environment::Decouple(MultiBody *_multibody[], int number) {
}

//============================================
//SortBodies so that the external bodies appear first in the array
//============================================
void Environment::SortMultiBodies(){
  int i,j;
  externalbodyCount = 1; //the only object is the robot
  i = 0;
  j = multibodyCount-1;
  while (i < j)
    {//Quicksort
      while((i<multibodyCount) && !multibody[i]->IsInternal()) 
	i++;
      while ((j>0)&&(multibody[j]->IsInternal()))
	j--;
      if (i<j)
	{
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

//===================================================================
//  Get
//===================================================================
void Environment::Get(Input * _input) {
	
    for (int i = 0; i < _input->multibodyCount; i++) {
        MultiBody * mb = new MultiBody(this);
		mb->Get(_input, i);
		AddMultiBody(mb);
    }

    //put the external bodies in the beginning part of the multibody array;
    SortMultiBodies();
    // calculate bounding box
    FindBoundingBox();
    UpdateBoundingBox(_input);
	
    // if user supplied a positional resolution, overwrite the one
    // calculated by "FindBoundingBox"
    if ( _input->posres.IsActivated() ) {
		positionRes = _input->posres.GetValue();
    }
	
    // orientational resolution may be user supplied but at this time
    // is not calculated
    orientationRes = _input->orires.GetValue(); 
	
}



//Get rid of obstacles outside the bounding box
void Environment::DeleteObstaclesOutsideBoundingBox() {
    double minx, miny, minz, maxx, maxy, maxz;
    minx = miny = minz = maxx = maxy = maxz = 0;

    minx=boundingBox[0]; maxx=boundingBox[1];
    miny=boundingBox[2]; maxy=boundingBox[3];
    minz=boundingBox[4]; maxz=boundingBox[5];

    vector< MultiBody* > in_obstacles;
    double * obb;
    int new_rob_index;
    int rob = this->GetRobotIndex();
    GetMultiBody(rob)->FindBoundingBox();

    for (int i = 0; i < this->GetMultiBodyCount(); i++) {
      //see if bounding box of multibody overlaps BB
      GetMultiBody(i)->FindBoundingBox();
      if (i == rob) {
	in_obstacles.push_back(GetMultiBody(i));
	new_rob_index = in_obstacles.size()-1;
      } else {

	obb = GetMultiBody(i)->GetBoundingBox();
	
	//	if obstacle not in collision with the bounding box
	if (((obb[0] <= maxx && obb[0] >= minx) || 
	     (obb[1] <= maxx && obb[1] >= minx)) &&
	    ((obb[2] <= maxy && obb[2] >= miny) || 
	     (obb[3] <= maxy && obb[3] >= miny)) &&
	    ((obb[4] <= maxz && obb[4] >= minz) || 
	     (obb[5] <= maxz && obb[5] >= minz))) 
	  in_obstacles.push_back(GetMultiBody(i));
	else {
	  //if bounding boxes cross each other	
	  if (obb[0] > maxx || obb[1] < minx || 
	      obb[2] > maxy || obb[3] < miny || 
	      obb[4] > maxz || obb[5] < minz) { 
	    cout << "Deleting obstacle " << i << endl;
	    delete GetMultiBody(i);
	  } else {
	    in_obstacles.push_back(GetMultiBody(i));
	  }
	}
      }
    }
    for (int i=0; i <in_obstacles.size(); i++) {
      multibody[i] = in_obstacles[i];
    }
    SetRobotIndex(new_rob_index);
    multibodyCount = in_obstacles.size();
    externalbodyCount = multibodyCount;
}

void Environment::UpdateBoundingBox(Input * _input) {
	
	
    // if user supplied a bounding box, overwrite calculated boundingBox
    if ( _input->bbox.IsActivated() ) {
		
		sscanf(_input->bbox.GetValue(),"[%lf,%lf,%lf,%lf,%lf,%lf]",
			&boundingBox[0], &boundingBox[1], &boundingBox[2],
			&boundingBox[3], &boundingBox[4], &boundingBox[5]);
		
    }
	
    // use bounding box scale factor
    if ( _input->bbox_scale.GetValue() != 1.0 ) {
		
		
		// determine center of mass (com) of bounding box
		Vector3D bb_min(boundingBox[0],boundingBox[2],boundingBox[4]);
		Vector3D bb_max(boundingBox[1],boundingBox[3],boundingBox[5]);
		Vector3D com = (bb_max+bb_min)/2;
		
		// for each vertex of bounding box
		//    coord = (coord - com)*scale_factor + com
		bb_min = (bb_min - com)*_input->bbox_scale.GetValue() + com;
		bb_max = (bb_max - com)*_input->bbox_scale.GetValue() + com;
		boundingBox[0]=bb_min[0];   boundingBox[1]=bb_max[0];
		boundingBox[2]=bb_min[1];   boundingBox[3]=bb_max[1];
		boundingBox[4]=bb_min[2];   boundingBox[5]=bb_max[2];
		
    }//endif bbox_scale != 1
	
}

void Environment::PutBoundingBox(double x,double X,
								 double y,double Y,
								 double z,double Z){
	
    boundingBox[0] = x;   boundingBox[1] = X;
    boundingBox[2] = y;   boundingBox[3] = Y; 
    boundingBox[4] = z,   boundingBox[5] = Z;
}

//===================================================================
//  Write
//
//  Function: Write the Input data for an environment into a 
//  file
//
//===================================================================
void Environment::Write(ostream & _os) {
    _os << multibodyCount << endl;
    for (int i=0; i < multibodyCount; i++)
        GetMultiBody(i)->Write(_os);
}

//===================================================================
//  FindBoundingBox
//  8/19/98  Daniel Vallejo
//===================================================================
void Environment::FindBoundingBox(){
	
    int rob = this->GetRobotIndex();
    GetMultiBody(rob)->FindBoundingBox();
    double ss, posres;
    ss = GetMultiBody(rob)->GetMaxAxisRange();
    posres = ss;
	
    int nmulti = this->GetMultiBodyCount();
	
    int first = 1;
    double * tmp;
    double minx, miny, minz, maxx, maxy, maxz;
    minx = miny = minz = maxx = maxy = maxz = 0;
    for(int i = 0 ; i < nmulti ; i++){
        if(i != rob){
            if(first){
                GetMultiBody(i)->FindBoundingBox();
                tmp = GetMultiBody(i)->GetBoundingBox();
                minx = tmp[0]; maxx = tmp[1];
                miny = tmp[2]; maxy = tmp[3];
                minz = tmp[4]; maxz = tmp[5];
                first = 0;
                posres = min(posres,GetMultiBody(i)->GetMaxAxisRange());
            }
            else{
                GetMultiBody(i)->FindBoundingBox();
                tmp = GetMultiBody(i)->GetBoundingBox();
                minx = min(minx,tmp[0]); maxx = max(maxx,tmp[1]);
                miny = min(miny,tmp[2]); maxy = max(maxy,tmp[3]);
                minz = min(minz,tmp[4]); maxz = max(maxz,tmp[5]);
                posres = min(posres,GetMultiBody(i)->GetMaxAxisRange());
            }
        }
    }

    double sf = ss/3.0;
    boundingBox[0] = minx-sf; boundingBox[1] = maxx+sf;
    boundingBox[2] = miny-sf; boundingBox[3] = maxy+sf;
    boundingBox[4] = minz-sf; boundingBox[5] = maxz+sf;
	
    positionRes = posres * POSITION_RES_FACTOR;
    minmax_BodyAxisRange = posres;
	
}

//===================================================================
//  Get_minmax_BodyAxisRange
//===================================================================
double Environment::Getminmax_BodyAxisRange(){
    return minmax_BodyAxisRange;
}


//===================================================================
//  GetBoundingBox
//  8/19/98  Daniel Vallejo
//===================================================================
double * Environment::GetBoundingBox(){
    return boundingBox;
}


//===================================================================
//  DisplayBoundingBox
//  9/6/98  Daniel Vallejo
//===================================================================
void Environment::DisplayBoundingBox(ostream & _os){
	
    _os << endl << "Bounding Box: " << endl;
    for(int i = 0 ; i < 6 ; i++){
        if(i % 2 == 0) _os << endl;
        _os << boundingBox[i] << ", ";
    }
    _os << endl;
}


// $Id$
/////////////////////////////////////////////////////////////////////
//  Environment.c
//
//  Created   3/ 1/98 Aaron Michalk
//  Modified  4/13/98 Aaron Michalk
//  Modified  4/16/98 Wookho Son
//  Added     4/21/98 Wookho Son
//  Added     5/15/98 Wookho Son
//  Modified  7/14/98 Wookho Son
//  Added/Modified  7/31/98 Wookho Son
//
/////////////////////////////////////////////////////////////////////

#include "Environment.h"
#include "MultiBody.h"
#include "stdlib.h"

#define DOF   6


//===================================================================
//  Constructors
//===================================================================
Environment::Environment() {
    multibodyCount = 0;
    multibody = 0;
    robotIndex = 0;
}
Environment::Environment(int index) {
    multibodyCount = 0;
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

//===================================================================
//  Get
//===================================================================
void Environment::Get(Input * _input) {

    for (int i = 0; i < _input->multibodyCount; i++) {
        MultiBody * mb = new MultiBody(this);
	mb->Get(_input, i);
	AddMultiBody(mb);
    }


    // it seems this method does more than its name implies so...
    // calculate bounding box
    FindBoundingBox();

    // if user supplied a bounding box, overwrite calculated boundingBox
    if ( _input->bbox.IsActivated() ) {

       sscanf(_input->bbox.GetValue(),"[%lf,%lf,%lf,%lf,%lf,%lf]",
          &boundingBox[0], &boundingBox[1], &boundingBox[2],
          &boundingBox[3], &boundingBox[4], &boundingBox[5]);

    }

    // if user supplied a positional resolution, overwrite the one
    // calculated by "FindBoundingBox"
    if ( _input->posres.IsActivated() ) {
       positionRes = _input->posres.GetValue();
    }

    // orientational resolution may be user supplied but at this time
    // is not calculated
    orientationRes = _input->orires.GetValue();

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
//  DisplayBB
//  9/6/98  Daniel Vallejo
//===================================================================
void Environment::DisplayBB(ostream & _os){

    _os << endl << "Bounding Box: " << endl;
    for(int i = 0 ; i < 6 ; i++){
        if(i % 2 == 0) _os << endl;
        _os << boundingBox[i] << ", ";
    }
    _os << endl;
}


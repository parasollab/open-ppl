// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Debug.c
//
//  General Description
//      Procedures used for debugging purposes
//
//  Created
//      xx/xx/xx Daniel Vallejo
//
//  Last Modified By:
//      8/21/99  Lucia K. Dale - added header 
//
/////////////////////////////////////////////////////////////////////

#include "Debug.h"


//===================================================================
//  DisplayCfgs
//===================================================================
void
DisplayCfgs(vector<Cfg> cfgs){

    cout << endl << "Num cfgs = " << cfgs.size();
    for(int i = 0 ; i < cfgs.size() ; i++){
        cout << endl << cfgs[i];
    }
    cout << endl;
}

//===================================================================
//  DisplayPoints
//===================================================================
void DisplayPoints(vector<Vector3D> pts){

    cout << endl << "Num pts = " << pts.size() << endl;
    for(int i = 0 ; i < pts.size() ; i++){
        cout << endl << pts[i];
    }
    cout << endl;
}


//===================================================================
//  DisplayVector
//===================================================================
void DisplayVector(vector<double> pts){

    cout << endl << "Num Elements = " << pts.size() << endl;
    for(int i = 0 ; i < pts.size() ; i++){
        cout << endl << pts[i];
    }
    cout << endl;
}

//===================================================================
void DisplayVector(vector<int> pts){
//===================================================================

    cout << endl << "Num Elements = " << pts.size() << endl;
    for(int i = 0 ; i < pts.size() ; i++){
        cout << endl << pts[i];
    }
    cout << endl;
}


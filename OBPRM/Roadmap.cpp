// $Id$
/////////////////////////////////////////////////////////////////////
//
//   Roadmap.c
//
//   General Description
//      This is the main OBPRM class which contains data and methods
//      to manipulate the environment with specified moving bodies
//      (ie, robot(s)) and the corresponding roadmap.
//
//      This file contains the definitions for the prototypes declared
//      in the file "Roadmap.h".
//  Created   
//      07/16/98  Lucia K. Dale
//  Modified   
//      07/07/99  Lucia K. Dale
/////////////////////////////////////////////////////////////////////

#include "Roadmap.h"
#include <string.h>

//===================================================================
//  Roadmap class Methods: Constructors and Destructor
//===================================================================
Roadmap::
Roadmap(){
};

Roadmap::
Roadmap(Input *input,
	CollisionDetection *cd,
	DistanceMetric *dm,
	LocalPlanners *lp,
	Environment * env)
{
    InitRoadmap(input,cd,dm,lp,NULL,env); 
    RoadmapVersionNumber = 62000;
};

Roadmap::
~Roadmap(){
};

//////////////////////////////////////////////////////////////////////
//  Initialize roadmap according to command line arguments
//  -- reserve space for nodes/edges in roadmap graph
//  -- read roadmap's environment 
//////////////////////////////////////////////////////////////////////
void
Roadmap::
InitRoadmap(Input *input, 
	    CollisionDetection *cd,
	    DistanceMetric *dm,
	    LocalPlanners *lp,
	    char *ExistingMap, 
	    Environment* env)
{

   //---------------------------------------------------------
   // initialize roadmap, from scratch or previously built map
   //---------------------------------------------------------

   if ( ExistingMap != NULL ) {           // start from previous map
      ReadRoadmap( input, cd,dm,lp, ExistingMap );
   } else {                                   // start from scratch
      // 'reserve' space for nodes/edges in roadmap graph
      roadmap.Init(
		input->numNodes.GetValue(),
		input->numEdges.GetValue());
   }

   //-----------------------------------------------
   // initialize problem environment
   //-----------------------------------------------

   if(env == NULL){
      environment = new Environment;
      InitEnvironment( input );
   }
   else{
      InitEnvironment(env);      
   }

   //-----------------------------------------------
   // set number of nodes correctly
   //-----------------------------------------------
   if ( input->numNodesPerObst.IsActivated() ) 
	input->numNodes.PutValue(
		input->numNodesPerObst.GetValue() * input->multibodyCount);

};


//////////////////////////////////////////////////////////////////////
//  Initialize roadmap's environment from a file
//
//  **NOTE** Not necessary if constructor "Roadmap(&input)" or
//           method "InitRoadmap(input)" is used. 
//////////////////////////////////////////////////////////////////////
void
Roadmap::
InitEnvironment(Input *input){

  // open 'init' file, read it into Input object, & close it
  input->Read();
  // read environment files & put them in Environment object
  environment->Get(input);

};


void
Roadmap::
InitEnvironment(Environment* env)
{
  environment = env;
};


                
//========================================================================
// Roadmap class Methods: Getting Data & Statistics
//========================================================================

//
// Get a pointer to roadmap's environment
//
Environment *
Roadmap::
GetEnvironment(){ 
  return environment;
};


void ReadRoadmap(const char* _fname){
   ifstream  myifstream(_fname);
   if (!myifstream) {
         cout << endl << "In ReadRoadmap: can't open infile: " << _fname ;
         return;
   }
   
};
//========================================================================
// Roadmap class Methods: Display, Input, Output
//========================================================================
void 
Roadmap::
ReadRoadmapGRAPHONLY(const char* _fname){
   cout << endl << "getting nodes from Read: " << _fname << endl;

   ifstream  myifstream(_fname);
   if (!myifstream) {
         cout << endl << "In ReadRoadmap: can't open infile: " << _fname ;
         return;
   }

  // skip over stuff up to and including DMSTOP, next line should contain GRAPHSTART
  char tagstring[400]; bool moreFile=true;
  while(moreFile){
    myifstream  >> tagstring;
    if ( strstr(tagstring,"DMSTOP") ) 
	moreFile = false;
  }

  roadmap.ReadGraph(myifstream);           // reads verts & adj lists

  myifstream.close();

}


//
// Read roadmap from a file
//
void 
Roadmap::
ReadRoadmap(Input *input,
CollisionDetection *cd,
DistanceMetric *dm,
LocalPlanners *lp,
const char* _fname
){
	cout << endl << "STUB-Read: " << _fname << endl;

   ifstream  myifstream(_fname);
   if (!myifstream) {
         cout << endl << "In ReadRoadmap: can't open infile: " << _fname ;
         return;
   }


   // Read throwaway tag strings..."Roadmap Version Number"
   char tagstring[30];
   myifstream  >>tagstring >>tagstring >>tagstring; 

   myifstream >> RoadmapVersionNumber;

   input->ReadPreamble(myifstream); // do nothing now (could init defaults)
   input->ReadEnvFile(myifstream);    
   lp->planners.ReadLPs(myifstream);
   cd->collisionCheckers.ReadCDs(myifstream);
   dm->distanceMetrics.ReadDMs(myifstream);

   roadmap.ReadGraph(myifstream);           // reads verts & adj lists

   myifstream.close();

};

//
// Write roadmap to a file
//
void 
Roadmap::
WriteRoadmap(Input *input,
CollisionDetection *cd,
DistanceMetric *dm,
LocalPlanners *lp,
const char* _fname
) {

   char outfile[200];

   if ( _fname == 0 ) {
      strcpy(outfile,input->mapFile.GetValue() );
   } else {
      strcpy( outfile,_fname );
   }

   ofstream  myofstream(outfile);
   if (!myofstream) {
      cout << endl << "In WriteRoadmap: can't open outfile: " << outfile ;
   }
   
   myofstream << "Roadmap Version Number " << RoadmapVersionNumber;

   input->WritePreamble(myofstream); // for now just write commandline
   input->WriteEnvFile(myofstream);
   lp->planners.WriteLPs(myofstream);
   cd->collisionCheckers.WriteCDs(myofstream);
   dm->distanceMetrics.WriteDMs(myofstream);

   roadmap.WriteGraph(myofstream);         // writes verts & adj lists
   myofstream.close();
};

//
// Write roadmap's environment to an output stream
//
void 
Roadmap::
WriteEnvironment(ostream & _os){
  environment->Write(_os);
};

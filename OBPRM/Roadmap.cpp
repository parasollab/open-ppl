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
  ReadExtraCfgInfo(myifstream);            // appends extra "info" to each vert

  myifstream.close();

}

//
// Due to "enhanced" InfoCfg's and not wanting to change Vizmo this is
// a bit of a hack...  It would be better to change Vizmo ... Lucia 7/7/99
//
void 
Roadmap::
ReadExtraCfgInfo(istream& _myistream) {
                                            // appends extra "info" to each vert
  vector<Cfg> v=roadmap.GetVerticesData();
  int vid,obst;
  for (int i=0; i<v.size(); ++i){
       _myistream >> vid >> v[i].info.obst;
       roadmap.PutData(vid,v[i]);
  };
};


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

   input->ReadPreamble(myifstream); // do nothing now (could init defaults)
   input->ReadEnvFile(myifstream);    
   lp->planners.ReadLPs(myifstream);
   cd->collisionCheckers.ReadCDs(myifstream);
   dm->distanceMetrics.ReadDMs(myifstream);

   roadmap.ReadGraph(myifstream);           // reads verts & adj lists
   ReadExtraCfgInfo(myifstream);            // appends extra "info" to each vert

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
   
   input->WritePreamble(myofstream); // for now just write commandline
   input->WriteEnvFile(myofstream);
   lp->planners.WriteLPs(myofstream);
   cd->collisionCheckers.WriteCDs(myofstream);
   dm->distanceMetrics.WriteDMs(myofstream);

   // Due to "enhanced" InfoCfg's and not wanting to change Vizmo this is
   // a bit of a hack...  It would be better to change Vizmo ... Lucia 7/7/99
   roadmap.WriteGraph(myofstream);         // writes verts & adj lists
   vector<VID> v=roadmap.GetVerticesVID(); // appends extra "info" from each vert
   for (int vid=0; vid < v.size(); ++vid){
	myofstream << v[vid];
        myofstream << " ";
        myofstream << roadmap.GetData(v[vid]).info.obst;
        myofstream << endl;
   };

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

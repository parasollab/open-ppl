// $Id$
/////////////////////////////////////////////////////////////////////
//
//   Roadmap.c
//
//   General Description
//      This is the main OBPRM class which contains data and methods
//      to manipulate the environment with specified moving bodies
//      (ie, robot(s)) and the corresponding m_pRoadmap->
//
//      This file contains the definitions for the prototypes declared
//      in the file "Roadmap.h".
//  Created   
//      07/16/98  Lucia K. Dale
/////////////////////////////////////////////////////////////////////

#include <string.h>

#include "BasicDefns.h"
#include "Roadmap.h"
#include "Environment.h"
#include "DistanceMetrics.h"
#include "util.h"

//===================================================================
//  Roadmap class Methods: Constructors and Destructor
//===================================================================
Roadmap::
Roadmap(){
	m_pRoadmap = new RoadmapGraph<Cfg,WEIGHT>;
}

Roadmap::
Roadmap(Input *input,
		CollisionDetection *cd,
		DistanceMetric *dm,
		LocalPlanners *lp,
		Environment * env)
{
	m_pRoadmap = new RoadmapGraph<Cfg,WEIGHT>;

    InitRoadmap(input,cd,dm,lp,NULL,env); 
    RoadmapVersionNumber = RDMPVER_CURRENT;
}

Roadmap::
~Roadmap(){
	if( m_pRoadmap != NULL )
		delete m_pRoadmap;
	m_pRoadmap = NULL;
}

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
	input->Read(EXIT);
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
	
	m_pRoadmap->ReadGraph(myifstream);           // reads verts & adj lists
	
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
	
	if ( !CheckVersion(_fname) ){
		cout << endl << "In ReadRoadmap: don't recognize format in: " << _fname ;
		return;
	}
	
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
	
	m_pRoadmap->ReadGraph(myifstream);           // reads verts & adj lists
	
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
	
	if ( _fname == NULL ) {
		strcpy(outfile,input->mapFile.GetValue() );
	} else {
		strcpy( outfile,_fname );
	}
	
	ofstream  myofstream(outfile);
	if (!myofstream) {
		cout << endl << "In WriteRoadmap: can't open outfile: " << outfile ;
	}
	
	myofstream << "Roadmap Version Number " << RDMPVER_CURRENT_STR;
	
	input->WritePreamble(myofstream); // for now just write commandline
	input->WriteEnvFile(myofstream);
	lp->planners.WriteLPs(myofstream);
	cd->collisionCheckers.WriteCDs(myofstream);
	dm->distanceMetrics.WriteDMs(myofstream);
	
	m_pRoadmap->WriteGraph(myofstream);         // writes verts & adj lists
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

//////////////////////////////////////////////////////////////////////
// Roadmap Version Check and Convertors
//////////////////////////////////////////////////////////////////////
bool 
Roadmap::
CheckVersion(const char* _fname)
{
	ifstream  myifstream(_fname);
	if (!myifstream) {
		cout << endl << "In CheckVersion: can't open infile: " << _fname ;
		return false;
	}
	
	int  thisVersion; 
	char tagstring[30];
	
	myifstream  >> tagstring; 
	if ( strstr(tagstring,"PREAMBLESTART") ) {
		thisVersion = RDMPVER_LEGACY;
	} else if ( strstr(tagstring,"Roadmap") ) {
		myifstream  >>tagstring >> tagstring;  // read "Version" and "Number"
		myifstream >> thisVersion;                 
	}
	
	myifstream.close();
	
	if ( thisVersion == RDMPVER_CURRENT  ) {
		return true;
	} else {
		return ConvertToCurrentVersion(_fname,thisVersion);
	}
	
} 

//
// Convert To Current Roadmap Version 
//
bool 
Roadmap::
ConvertToCurrentVersion(const char* _fname, int thisVersion)
{
	
	char infile[200], outfile[200];
	
	// copy current file _fname to "_fname.XXX"  where XXX=thisVersion
	// construct infile=_fname.XXX  and outfile=_fname names
	SaveCurrentVersion(_fname, thisVersion, infile, outfile);
	
	ifstream  myifstream(infile);
	if (!myifstream) {
		cout << endl << "In ConvertToCurrentVersion: can't open infile: " << infile ;
		return false;
	}
	
	ofstream  myofstream(outfile);
	if (!myofstream) {
		cout << endl << "In ConvertToCurrentVersion: can't open outfile: " << outfile ;
		return false;
	}
	
	char tagstring[500];
	
	if ( thisVersion != RDMPVER_LEGACY ) {
		// read tags "Roadmap Version Number XXX" -- don't exist in LEGACY VERSION
		myifstream >> tagstring >> tagstring >> tagstring >> tagstring;
	}
	myofstream << "Roadmap Version Number " << RDMPVER_CURRENT_STR; 
	
	// copy myifstream into myofstream until get to graph...
	while ( !strstr(tagstring,"DMSTOP") ) {
		myifstream.getline(tagstring,499);
		myofstream << tagstring << endl;
	}
	
	if ( thisVersion == RDMPVER_LEGACY) {
		// legacy: 0 cfg fields, 1 wt field (lp)
		ConvertGraph(myifstream, myofstream, 
			RDMPVER_LEGACY_CFG_FIELDS, RDMPVER_LEGACY_EDGEWT_FIELDS);
		myofstream << endl << "Converted from ROADMAP VERSION LEGACY";
		
	} else if (thisVersion == RDMPVER_62000) {
		// 62000: 2 cfg fields (obst, tag), 2 wt fields (lp, ticks/weight)
		ConvertGraph(myifstream, myofstream, 
			RDMPVER_62000_CFG_FIELDS, RDMPVER_62000_EDGEWT_FIELDS);
		myofstream << endl << "Converted from ROADMAP VERSION 62000";
		
	} else if (thisVersion == RDMPVER_061100) {
		// 061100: 2 cfg fields (obst, tag), 2 wt fields (lp, ticks/weight)
		ConvertGraph(myifstream, myofstream, 
			RDMPVER_061100_CFG_FIELDS, RDMPVER_061100_EDGEWT_FIELDS);
		myofstream << endl << "Converted from ROADMAP VERSION 061100";
		
	} else if (thisVersion == RDMPVER_061300) {
		// 061300: 3 cfg fields (obst, tag, clearance), 2 wt fields (lp, ticks/weight)
		ConvertGraph(myifstream, myofstream,
			RDMPVER_061300_CFG_FIELDS, RDMPVER_061300_EDGEWT_FIELDS);
		myofstream << endl << "Converted from ROADMAP VERSION 061300";
		
	} else {
		cout << "In ConvertToCurrentVersion: unknown roadmap version in " << _fname << endl;
		return false;
	}
	
	myifstream.close();
	myofstream.close();
	return true;
}

//
// copies file _fname to "_fname.XXX" where XXX==thisVersion
// and constructs infile = _fname.XXX and outfile = _fname
//
void
Roadmap::
SaveCurrentVersion(const char* _fname, int thisVersion, 
				   char* infile, char* outfile)
{
	char tagstring[500];
	
	// construct names for infile (_fname) and outfile (_fname.XXX)
	strcpy( infile, _fname );
	strcpy( outfile, _fname );
	strcat( outfile, ".RdmpVer-" );
	if ( thisVersion == RDMPVER_LEGACY ) {
		strcat ( outfile, "LEGACY" );
	} else if ( thisVersion == RDMPVER_62000 ) {
		strcat ( outfile, "62000" );
	} else if ( thisVersion == RDMPVER_061100 ) {
		strcat ( outfile, "061100" );
	} else if ( thisVersion == RDMPVER_061300 ) {
		strcat ( outfile, "061300" );
	} else {
		strcat ( outfile, "UNKNOWN_VERSION" );
	}
	
	// open infile and outfile
	ifstream  myifstream(infile);
	if (!myifstream) {
		cout << endl << "In SaveCurrentVersion: cannot open infile: " << infile;
		return;
	}
	ofstream  myofstream(outfile);
	if (!myofstream) {
		cout << endl << "In SaveCurrentVersion: cannot open outfile: " << outfile;
		return;
	}
	
	// copy infile into outfile
	while ( myifstream.getline(tagstring,499) ){
		myofstream << tagstring << endl;
	}
	
	// now, swap the names so we'll read from _fname.XXX and write to _fname
	strcpy( infile, outfile ); 
	strcpy( outfile, _fname ); 
}


//
// This function will read and convert the lines of the graph file relating
// to the adjacency lists *ONLY*.
//
// ASSUMPTION: the old version is a subset of the new version, missing only
//             some values in either the InfoCfg field or in the Edge Weights
// REMEDY: the missing fields will be padded with some "null" value - note
//         this is not a robust approach and may yield trouble later...
//
void 
Roadmap::
ConvertGraph(istream&  myifstream, ostream& myofstream, 
			 int presentCfgFields, int presentEdgeWtFields) 
{
	
	char tagstring[200];
	int v, i, j, nverts, nedges, vids;
	
	// GRAPHSTART TAG
	myifstream >> tagstring;
	myofstream << tagstring << endl;
	// GRAPH info (#verts #edges #vids) 
	myifstream >> nverts >> nedges >> vids;
	myofstream << nverts << " " << nedges << " " << vids << endl;
	
	Cfg c;
	int dofs = c.DOFs();
	int missingCfgFields     = RDMPVER_CURRENT_CFG_FIELDS - presentCfgFields;
	int missingEdgeWtFields  = RDMPVER_CURRENT_EDGEWT_FIELDS - presentEdgeWtFields;
	
	for (v=0; v < nverts; v++) {
		myifstream >> vids; 
		myofstream << vids << " "; 
		for (i=0; i < dofs; i++) {
			myifstream >> tagstring;
			myofstream << tagstring << " "; 
		}
		for (i=0; i < presentCfgFields; i++) {
			myifstream >> tagstring;
			myofstream << tagstring << " "; 
		}
		for (i=0; i < missingCfgFields; i++){
			myofstream << InfoCfg::NULL_INFO << " "; 
		}
		
		int adjedges;
		myifstream >> adjedges; 
		myofstream << adjedges << " "; 
		for (i=0; i < adjedges; i++){
			myifstream >> tagstring;         // vid of adj vertex for this edge
			myofstream << tagstring << " "; 
			for (j=0; j < presentEdgeWtFields; j++){
				myifstream >> tagstring;
				myofstream << tagstring << " "; 
			}
			for (j=0; j < missingEdgeWtFields; j++){
				myofstream << NULL_WT_INFO << " "; 
			}
		}
		myofstream << endl;
	}
	
	// GRAPHSTOP TAG
	myifstream >> tagstring;
	myofstream << tagstring << endl;
	
	return;
}



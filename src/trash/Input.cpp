/////////////////////////////////////////////////////////////////////
//  Input.c
//
//  Created  7/22/98 Wookho Son
/////////////////////////////////////////////////////////////////////

#include "Input.h"
#include "CfgTypes.h"
#include "util.h"

// Need for retreiving default parameters of each
// in executing the parameter "-defaults"
#include "DistanceMetrics.h"
#include "GenerateMapNodes.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "MapEvaluator.h"

//===================================================================
//  Input class
//  Constructors and Destructor
//===================================================================
Input::Input():
        defaultFile      ("-f"),
        descDir          ("-descDir"),
        envFile          ("-envFile"),
        mapFile          ("-outmapFile"),
        inmapFile        ("-inmapFile"),
	inmapFile2        ("-inmapFile2"),
	partitionType    ("-partition"),
	integrationType ("-integration"),
                       //  default   min,max
        addPartialEdge   ("-addPartialEdge",     0,  0,    1),
	addAllEdges      ("-addAllEdges",        0,  0,    1),
	seedByChunk      ("-seedByChunk",        1,  0,    1),
        posres           ("-posres",          0.05, -1000, 1000),
        orires           ("-orires", ORIENTATION_RES, -1000, 1000),
        bbox_scale       ("-bbox_scale",       2.0,  -50000, 50000),
        bbox             ("-bbox",""),
	bbox_ref         ("-bbox_ref",""),
        collPair         ("-collPair","cM rT "),    // center of Mass
        freePair         ("-freePair","cM rV "),    // center of Mass
	numofJoints      ("-numofjoints", 0, 0, 1000000)
        {

    cfgSet=false;
    addPartialEdge.PutDesc   ("INTEGER","");
    addAllEdges.PutDesc      ("INTEGER","");
    seedByChunk.PutDesc   ("INTEGER"," seed for each chunk");

    envFile.PutDesc    ("STRING ","");
    mapFile.PutDesc    ("STRING ","");
    inmapFile.PutDesc  ("STRING read in a map file","");
    inmapFile2.PutDesc  ("STRING *causes Node Generation to be SKIPPED","");

    partitionType.PutDesc("STRING determine what kind of partitioning will be used","");
    integrationType.PutDesc("STRING determine what kind of integration method will be used","");

    defaultFile.PutDesc("STRING ",
        " default filename prefix for common files");
    descDir.PutDesc    ("STRING ",
        " directory where program will look for the data"
        "\n\t\t\t    files specified in environment specification file");

    bbox.PutDesc       ("STRING ", "[Xmin,Xmax,Ymin,Ymax,Zmin,Zmax]"
                                    "\n\t\t\t(default calculated from environment)"
                                    "\n\t\t\tATTN: robust if no spaces are used.");

    bbox_ref.PutDesc       ("STRING ", "[Xmin,Xmax,Ymin,Ymax,Zmin,Zmax]"
			"\n\t\t\t(default calculated from environment)"
			"\n\t\t\tATTN: robust if no spaces are used.");

    bbox_scale.PutDesc        ("FLOAT  ","");
    posres.PutDesc            ("FLOAT  "," *CALCULATED*   position    resolution");
    orires.PutDesc            ("FLOAT  "," **HARDCODED**  orientation resolution");
    numofJoints.PutDesc       ("INT    "," number of articulated linkages");

    int i;

    numGNs = 0;
    for (i=0;i<MAX_GN;++i)
        GNstrings[i]=new n_str_param("-gNodes");
    numCNs = 0;
    for (i=0;i<MAX_CN;++i)
        CNstrings[i]=new n_str_param("-cNodes");
    numLPs = 0;
    for (i=0;i<MAX_LP;++i)
        LPstrings[i]=new n_str_param("-lp");
    numCDs = 0;
    for (i=0;i<MAX_CD;++i)
        CDstrings[i]=new n_str_param("-cd");
    numDMs = 0;
    for (i=0;i<MAX_DM;++i)
        DMstrings[i]=new n_str_param("-dm");
    numMEs = 0;
    for(i=0;i<MAX_GN;++i)
	MEstrings[i]=new n_str_param("-eval");

    GNstrings[0]->PutDesc("STRING",
        "\n\t\t\tPick any combo: default BasicPRM"
        "\n\t\t\t  BasicPRM    nodes     INT (number of nodes, default 10)"
        "\n\t\t\t              exact     INT (Whether to generate exact num of nodes, default 0)"
	"\n\t\t\t              chunkSize INT (chunk size, default 10)"
        "\n\t\t\t  BasicOBPRM  nodes     INT (number of nodes, default 10)"
        "\n\t\t\t              exact     INT (Whether to generate exact num of nodes, default 0)"
	"\n\t\t\t              chunkSize INT (chunk size, default 10)"
        "\n\t\t\t              shells    INT (number of shells, default 3)"
        "\n\t\t\t  OBPRM       nodes     INT (number of nodes, default 10)"
        "\n\t\t\t              exact     INT (Whether to generate exact num of nodes, default 0)"
	"\n\t\t\t              chunkSize INT (chunk size, default 10)"
	"\n\t\t\t              shells    INT (number of shells, default 3)"
        "\n\t\t\t              collPair  STRING STRING (default cM, rT)"
        "\n\t\t\t                        Specify 2 of the following recognized mnemonics:"
        "\n\t\t\t                        cM    \"center of mass\""
        "\n\t\t\t                        rV    \"random vertex\""
        "\n\t\t\t                        rT    \"point in random triangle\""
        "\n\t\t\t                        rE    \"random extreme vertex\""
        "\n\t\t\t                        rW    \"point in random weighted triangle\""
        "\n\t\t\t                        cM_rV \"cg/random vertex\""
        "\n\t\t\t                        rV_rT \"random vertex/point in random          triangle\""
        "\n\t\t\t                        rV_rW \"random vertex/point in random weighted triangle\""
        "\n\t\t\t                        N_rT  \"normal of random triangle\""
        "\n\t\t\t                        all   \"all of the above\""
        "\n\t\t\t              freePair  STRING STRING (default cM, rV)"
        "\n\t\t\t                        Same as above"
        "\n\t\t\t              clearFact DOUBLE (clearance factor, default 1.0)"
        "\n\t\t\t              pctSurf   DOUBLE (proportion surface nodes, default 1.0)"
        "\n\t\t\t  GaussPRM    nodes     INT (number of nodes, default 10)" 
        "\n\t\t\t              exact     INT (Whether to generate exact num of nodes, default 0)"
	"\n\t\t\t              chunkSize INT (chunk size, default 10)"
        "\n\t\t\t              d         INT (distance, default based on environment)"
        "\n\t\t\t  BasicMAPRM  nodes     INT (number of nodes, default 10)"
        "\n\t\t\t              exact     INT (Whether to generate exact num of nodes, default 0)"
	"\n\t\t\t              chunkSize INT (chunk size, default 10)"
	"\n\t\t\t              approx    INT (using approximation or exact computation, default 1)"
        "\n\t\t\t              approx_ray INT (number of rays for approximation penetration, default 10)"
        "\n\t\t\t  CSpaceMAPRM nodes     INT (number of nodes, default 10)"
        "\n\t\t\t              exact     INT (Whether to generate exact num of nodes, default 0)"
	"\n\t\t\t              chunkSize INT (chunk size, default 10)"
	"\n\t\t\t              clearance INT (number of rays for approx clearance calulation, default 5)"
	"\n\t\t\t              penetration INT (number of rays for approx penetration calculation, default 5)"
	"\n\t\t\t  OBMAPRM     nodes     INT (number of nodes, default 10)"
        "\n\t\t\t              exact     INT (Whether to generate exact num of nodes, default 0)"
	"\n\t\t\t              chunkSize INT (chunk size, default 10)"
        "\n\t\t\t              clearance   INT (number of rays for approx clearance calculation, default 5)"
        "\n\t\t\t              penetration INT (number of rays for approx penetration calculation, default 5)"
	"\n\t\t\t              shells      INT (number of shells, default 3)"
        "\n\t\t\t              collPair    STRING STRING (default cM, rT)"
        "\n\t\t\t                          Same as OBPRM"
        "\n\t\t\t              freePair    STRING STRING (default cM, rV)"
        "\n\t\t\t                          Same as OBPRM"
        "\n\t\t\t              clearFact   DOUBLE (clearance factor, default 1.0)"
        "\n\t\t\t              pctSurf     DOUBLE (proportion surface nodes, default 1.0)"
        );
    CNstrings[0]->PutDesc("STRING",
        "\n\t\t\tPick any combo: default closest 10"
        "\n\t\t\t  random"
        "\n\t\t\t  closest        INT         (k:5)"
        "\n\t\t\t  closestVE      INT         (k:5)"
        "\n\t\t\t  components     INT INT     (kpairs:3 smallcc:4)"
        "\n\t\t\t  obstBased      INT INT     (other:10 self:3)"
        "\n\t\t\t  RRTexpand      INT INT INT INT INT (iter:50 step:10000 smallcc:3 \n\t\t\t\t\t obst_clearance: 1 clearance_from_node 1)"
        "\n\t\t\t  RRTcomponents  INT INT INT INT INT (iter:50 step:10000 smallcc:3 \n\t\t\t\t\t obst_clearance: 1 clearance_from_node 1)"
        "\n\t\t\t  RayTracer      STRING INT INT INT STRING INT INT (\n\t\t\t\t\t bouncingMode:targetOriented \n\t\t\t\t\t maxRays:1 maxBounces:10000 maxRayLength:10000 \n\t\t\t\t\t schedulingMode:largestToSmallest \n\t\t\t\t\t scheduleMaxSize:20 sampleMaxSize:10)"
        "\n\t\t\t  modifiedLM     INT INT INT (kpairs:5, add:20, rfactor:2)"
        );
    LPstrings[0]->PutDesc("STRING",
        "\n\t\t\tPick any combo: default straightline rotate_at_s s 0.5"
	"\n\t\t\t  straightline     lineSegmentLength INT binarySearch INT (def,lineSegment=0, binarySearch=0)"
        "\n\t\t\t  rotate_at_s      s FLOAT   (def,s=0.5) may have multiple values "
        "\n\t\t\t  a_star_distance  tries INT neighbors INT (def,tries=6,neighbors=3)"
        "\n\t\t\t  a_star_clearance tries INT neighbors INT (def,tries=6,neighbors=3)"
        "\n\t\t\t  approx_spheres   n INT     (def,n=3)"
        );
    CDstrings[0]->PutDesc("STRING",
        "\n\t\t\tPick one: default is the first in the list "
#ifdef USE_CSTK
        "\n\t\t\t  cstk"
#endif
#ifdef USE_RAPID
    "\n\t\t\t  RAPID"
#endif
#ifdef USE_VCLIP
        "\n\t\t\t  vclip"
#endif
#ifdef USE_PQP
    "\n\t\t\t  PQP"
#endif
        );

    DMstrings[0]->PutDesc("STRING",
        "\n\t\t\tPick one: default scaledEuclidean 0.9"
        "\n\t\t\t  euclidean"
        "\n\t\t\t  scaledEuclidean FLOAT (default,0.5)"
	"\n\t\t\t  minkowski FLOAT FLOAT FLOAT (default 3 3 0.333)"
	"\n\t\t\t  manhattan"
	"\n\t\t\t  com"

        );

    MEstrings[0]->PutDesc("STRING","\n\t\t  specify map evaluators");

    strcpy(commandLine,"");

   nprocs = 1;
};

Input::~Input() {
}

void Input::ReadCommandLine(int argc, char** argv){
  
  //-- save command line for future reference
  for (int i=0; i < argc; i++) {
    strcat(commandLine,argv[i]);
    strcat(commandLine," ");
  }
  
  #if defined USE_CSTK
    cdtype = CSTK;
  #elif defined USE_RAPID
    cdtype = RAPID;
  #elif defined USE_PQP
    cdtype = PQP;
  #elif defined USE_VCLIP
    cdtype = VCLIP;
  #else
    #ifdef NO_CD_USE
      cdtype = CD_USER1;
    #else
      #error You have to specify at least one collision detection library.
    #endif
  #endif
  
  //-- evaluate command line
  try {
    
    if (argc == 1)
      throw BadUsage();
    
    // process input parameter "-defaults"
    else if ((argc == 2) && (!strcmp(argv[1], "-defaults"))) {
      PrintDefaults();
      exit(-1);
    }
    
    for (int i=1;i<argc; ++i) {
      if ( defaultFile.AckCmdLine(&i, argc, argv) ){
	char tmp[80];
	strcpy(tmp, defaultFile.GetValue() ); strcat(tmp,".env");
	envFile.PutValue(tmp);
	strcpy(tmp, defaultFile.GetValue() ); strcat(tmp,".map");
	mapFile.PutValue(tmp);
      } else if ( descDir.AckCmdLine(&i, argc, argv) ) {
      } else if ( envFile.AckCmdLine(&i, argc, argv) ) {
      } else if ( mapFile.AckCmdLine(&i, argc, argv) ) {
      } else if ( inmapFile.AckCmdLine(&i, argc, argv) ) {
      } else if ( inmapFile2.AckCmdLine(&i, argc, argv) ) {
	} else if ( partitionType.AckCmdLine(&i, argc, argv) ) {
	} else if ( integrationType.AckCmdLine(&i, argc, argv) ) {
      } else if ( addPartialEdge.AckCmdLine(&i, argc, argv) ) {
      } else if ( addAllEdges.AckCmdLine(&i, argc, argv) ) {
      } else if ( seedByChunk.AckCmdLine(&i, argc, argv) ) {
      } else if ( bbox.AckCmdLine(&i, argc, argv) ) {
      } else if ( bbox_ref.AckCmdLine(&i, argc, argv) ) {
      } else if ( bbox_scale.AckCmdLine(&i, argc, argv) ) {
      } else if ( posres.AckCmdLine(&i, argc, argv) ) {
      } else if ( orires.AckCmdLine(&i, argc, argv) ) {
      } else if ( numofJoints.AckCmdLine(&i, argc, argv) ) {
	
      } else if ( GNstrings[numGNs]->AckCmdLine(&i, argc, argv) ) {
	numGNs++;
      } else if ( CNstrings[numCNs]->AckCmdLine(&i, argc, argv) ) {
	numCNs++;
      } else if ( LPstrings[numLPs]->AckCmdLine(&i, argc, argv) ) {
	numLPs++;
      } else if ( CDstrings[numCDs]->AckCmdLine(&i, argc, argv) ) {
        if (!(strncmp(CDstrings[numCDs]->GetValue(),"cstk",4))) {
          #ifdef USE_CSTK
	    cdtype = CSTK;
          #else
	    cout << "CSTK is not supported by current collision detection library. \n Please recompile with  CSTK .\n";
	    exit(5);	  
          #endif
	  
        }else if (!(strncmp(CDstrings[numCDs]->GetValue(),"vclip",5))) {
	  
          #ifdef USE_VCLIP
	    cdtype = VCLIP;
          #else
	    cout << "VCLIP is not supported by current collision detection library. \n Please recompile with   VCLIP .\n";
	    exit(5);
          #endif
	  
        }else if (!(strncmp(CDstrings[numCDs]->GetValue(),"RAPID",5))) {
          #ifdef USE_RAPID
	  
	    cdtype = RAPID;
          #else
	    cout << "RAPID is not supported by current collision detection library. \n Please recompile with RAPID .\n";
	    exit(5);
          #endif
	  
        }else if (!(strncmp(CDstrings[numCDs]->GetValue(),"PQP",3))) {
          #ifdef USE_PQP
	  
	    cdtype = PQP;
          #else
	    cout << "PQP is not supported by current collision detection library. \n Please recompile with PQP .\n";
	    exit(5);
          #endif

	}
	numCDs++;
      } else if ( DMstrings[numDMs]->AckCmdLine(&i, argc, argv) ) {
	numDMs++;
      } else if (MEstrings[numMEs]->AckCmdLine(&i, argc, argv)) { 
	numMEs++;
      } else {
	cout << "\nERROR: Don\'t understand blah\""<< argv[i]<<"\"";
	throw BadUsage();
      } //endif
    } //endfor i
    
    //-- Do some clean up and final checking

    if ( !defaultFile.IsActivated() &&
	 !( envFile.IsActivated() && mapFile.IsActivated() )){
      throw BadUsage();
    }

    if ( inmapFile.IsActivated() ){
      VerifyFileExists(inmapFile.GetValue(),EXIT);
    }
    if ( inmapFile2.IsActivated() ){
      VerifyFileExists(inmapFile2.GetValue(),EXIT);
    }

    descDir.VerifyValidDirName();
    
    //-- Verify INPUT file exists
    VerifyFileExists(envFile.GetValue(),EXIT);


  } //endtry
  catch (BadUsage ) {
    PrintUsage(cout,argv[0]);
    exit(-1);
  } //endcatch

};


//===================================================================
//  Input class
//  PrintUsage
//===================================================================
void
Input::
PrintUsage(ostream& _os,char *executablename){

        _os << "\nUsage: " << executablename << " [-flag options]";
        _os << "\n\nAvailable flags & options are:";

    cout.setf(ios::left,ios::adjustfield);

        _os << "\n\nMANDATORY:\n";
        _os << "\n  "; defaultFile.PrintUsage(_os);
        _os << "\n\t _OR_ ";
        _os << "\n  "; envFile.PrintUsage(_os);
        _os << "\n  "; mapFile.PrintUsage(_os);

        _os << "\n\nOPTIONAL:\n";
        _os << "\n  "; descDir.PrintUsage(_os);
        _os << "\n  "; inmapFile.PrintUsage(_os);
        _os << "\n  "; inmapFile2.PrintUsage(_os);
	_os << "\n  "; partitionType.PrintUsage(_os);
	_os << "\n  "; integrationType.PrintUsage(_os);
        _os << "\n  "; addPartialEdge.PrintUsage(_os);
	_os << "\n  "; addAllEdges.PrintUsage(_os);
	_os << "\n  "; seedByChunk.PrintUsage(_os);
        _os << "\n  "; bbox.PrintUsage(_os);
	_os << "\n  "; bbox_ref.PrintUsage(_os);
        _os << "\n  "; bbox_scale.PrintUsage(_os);
        _os << "\n  "; posres.PrintUsage(_os);
        _os << "\n  "; orires.PrintUsage(_os);
	_os << "\n  "; numofJoints.PrintUsage(_os);
        _os << "\n";
        _os << "\n  "; GNstrings[0]->PrintUsage(_os);
        _os << "\n  "; CNstrings[0]->PrintUsage(_os);
        _os << "\n  "; LPstrings[0]->PrintUsage(_os);
        _os << "\n  "; CDstrings[0]->PrintUsage(_os);
        _os << "\n  "; DMstrings[0]->PrintUsage(_os);
	_os << "\n  "; MEstrings[0]->PrintUsage(_os);

        _os << "\n\n  to see default values only, type \"obprm -defaults\" ";

    cout.setf(ios::right,ios::adjustfield);

        _os << "\n\n";
};

void
Input::
PrintValues(ostream& _os){

  int FW  = 20;
  int F   =  8;

  _os <<"\n"<<setw(FW)<<"defaultFile"<<"\t"<<defaultFile.GetValue();
  _os <<"\n"<<setw(FW)<<"descDir"<<"\t"<<descDir.GetValue();
  _os <<"\n"<<setw(FW)<<"envFile"<<"\t"<<envFile.GetValue();
  _os <<"\n"<<setw(FW)<<"mapFile"<<"\t"<<mapFile.GetValue();
  _os <<"\n"<<setw(FW)<<"inmapFile"<<"\t"<<inmapFile.GetValue();
  _os <<"\n"<<setw(FW)<<"inmapFile2"<<"\t"<<inmapFile2.GetValue();
  _os <<"\n"<<setw(FW)<<"partitionType"<<"\t"<<partitionType.GetValue();
  _os <<"\n"<<setw(FW)<<"integrationType"<<"\t"<<integrationType.GetValue();
  _os <<"\n"<<setw(FW)<<"addPartialEdge"<<"\t"<<addPartialEdge.GetValue();
  _os <<"\n"<<setw(FW)<<"addAllEdges"<<"\t"<<addAllEdges.GetValue();
  _os <<"\n"<<setw(FW)<<"seedByChunk"<<"\t"<<seedByChunk.GetValue();
  _os <<"\n"<<setw(FW)<<"bbox"<<"\t"<<bbox.GetValue();
  _os <<"\n"<<setw(FW)<<"bbox"<<"\t"<<bbox_ref.GetValue();
  _os <<"\n"<<setw(FW)<<"bbox_scale"<<"\t"<<bbox_scale.GetValue();
  _os <<"\n"<<setw(FW)<<"posres"<<"\t"<<posres.GetValue();
  _os <<"\n"<<setw(FW)<<"orires"<<"\t"<<orires.GetValue();
  _os <<"\n"<<setw(FW)<<"numofjoints"<<"\t"<<numofJoints.GetValue();

  int i;
  for(i=0;i<numGNs;++i)
    _os << "\n"<<setw(FW)<< "GNstrings"<<i
        <<"\t"<<GNstrings[i]->GetValue();
  for(i=0;i<numCNs;++i)
    _os << "\n"<<setw(FW)<< "CNstrings"<<i
        <<"\t"<<CNstrings[i]->GetValue();
  for(i=0;i<numLPs;++i)
    _os << "\n"<<setw(FW)<< "LPstrings"<<i
        <<"\t"<<LPstrings[i]->GetValue();
  for(i=0;i<numCDs;++i)
    _os << "\n"<<setw(FW)<< "CDstrings"<<i
        <<"\t"<<CDstrings[i]->GetValue();
  for(i=0;i<numDMs;++i)
    _os << "\n"<<setw(FW)<< "DMstrings"<<i
        <<"\t"<<DMstrings[i]->GetValue();
  for(i=0;i<numMEs;++i)
    _os << "\n" << setw(FW) << "MEstrings" << i << "\t" << MEstrings[i]->GetValue();

        _os << "\n\n";
};

void
Input::PrintDefaults(){

   int FW  = 30;
   int F   =  8;

   cout << setw(FW) << "defaultFile : no default string for this parameter" << endl;
   cout << setw(FW) << " (those parameters not listed here have no default value)" << endl << endl;
   cout << setw(FW) << "add partial edge" << " (" << addPartialEdge.GetFlag() << ") : " <<
            addPartialEdge.GetDefault() << endl << endl;
   cout << setw(FW) << "add all edges" << " (" << addAllEdges.GetFlag() << ") : " <<
            addAllEdges.GetDefault() << endl << endl;
   cout << setw(FW) << "seed by chunk " << " (" << seedByChunk.GetFlag() << ") : " <<
            seedByChunk.GetDefault() << endl << endl;
   cout << setw(FW) << "position resolution" << " (" << posres.GetFlag() << ") : " <<
            posres.GetDefault() << endl << endl;
   cout << setw(FW) << "orientation resolution" << " (" << orires.GetFlag() << ") : " <<
            orires.GetDefault() << endl << endl;
   cout << setw(FW) << "number of joints" << " (" << numofJoints.GetFlag() << ") : " <<
            numofJoints.GetDefault() << endl << endl;
   cout << setw(FW) << "bounding box scale" << " (" << bbox_scale.GetFlag() << ") : " <<
            bbox_scale.GetDefault() << endl << endl;

   // get default parameters by initializing each class
   // for some, need to call UserInit() to add sets so that we can display

   // there are possibly many sets available and the particular one
   // "selected" is recognized by the setid number.

   // Generate Map Nodes
   GenerateMapNodes<Cfg_free> gn;
   cout << setw(FW) << "Generate Map Nodes" << " (" << GNstrings[0]->GetFlag() <<
     ") : default = ";
   gn.PrintDefaults(cout);

   // Connect Map Nodes
   ConnectMap<Cfg_free, DefaultWeight> cn;          // default value of cnInfo.cnsetid is set
   cout << setw(FW) << endl << endl
        << "Connect Map (" << CNstrings[0]->GetFlag() <<
      ") : default ";
   cn.PrintDefaults(cout);

   // Local Planners
   LocalPlanners<Cfg_free, DefaultWeight> lp;
   // this default is already set in ConnectMapNodes::DefaultInit()
   cout << setw(FW) << endl << endl << "Local Planners" << " (" << LPstrings[0]->GetFlag() <<
      ") : default = ";
   lp.PrintDefaults(cout);

   // Distance Metric
   DistanceMetric dm;
   // this default is already set in ConnectMapNodes::DefaultInit()
   cout << setw(FW) << endl << endl << "Distance Metric" << " (" << DMstrings[0]->GetFlag() <<
      ") : default = ";
   dm.PrintDefaults(cout);

   //Map Evaluator
   MapEvaluator<Cfg_free, DefaultWeight> me;
   cout << setw(FW) << endl << endl << "Map Evaluators" << " (" << MEstrings[0]->GetFlag() << ") : default = ";
   me.PrintDefaults(cout);

   // Collision Detection
   CollisionDetection cd;
   // this default is already set in ConnectMapNodes::DefaultInit()
   cout << setw(FW) << endl << endl << "Collision Detection" << " (" << CDstrings[0]->GetFlag() <<
      ") : default = ";
   cd.PrintDefaults(cout);

   cout << endl << flush; 
}


//===================================================================
//  Input class
//  WritePreamble
//===================================================================
void Input::WritePreamble(ostream& _myostream) const {
  _myostream << endl << "#####PREAMBLESTART#####";
  _myostream << endl << commandLine;
  _myostream << endl << "#####PREAMBLESTOP#####";
};

//===================================================================
//  Input class
//  ReadPreamble  -- do nothing...
//===================================================================
void Input::ReadPreamble(istream& _myistream) {

  char tagstring[4*ARGSTRING_LENGTH];

  _myistream  >> tagstring;
  if ( !strstr(tagstring,"PREAMBLESTART") ) {
    cout << endl << "In ReadEnvFile: didn't read PREAMBLESTART tag right";
    return;
  }

  _myistream.getline(tagstring,100,'\n');  // throw out prev line
  _myistream.getline(tagstring,4*ARGSTRING_LENGTH,'\n'); // do nothing for now...

  _myistream >> tagstring;
  if ( !strstr(tagstring,"PREAMBLESTOP") ) {
    cout << endl << "In ReadEnvFile: didn't read PREAMBLESTOP tag right";
    return;
  }

};

//===================================================================
//  Input class
//  WriteEnvFile
//===================================================================
void Input::WriteEnvFile(ostream& _myostream) {
  _myostream << endl << "#####ENVFILESTART#####";
  _myostream << endl << envFile.GetValue();
  _myostream << endl << "#####ENVFILESTOP#####";
};

//===================================================================
//  Input class
//  ReadEnvFile
//===================================================================
void Input::ReadEnvFile(istream& _myistream) {

  char tagstring[ARGSTRING_LENGTH];

  _myistream >> tagstring;
  if ( !strstr(tagstring,"ENVFILESTART") ) {
    cout << endl << "In ReadEnvFile: didn't read ENVFILESTART tag right";
    return;
  }

  _myistream >> envFile.GetValue();

  _myistream >> tagstring;
  if ( !strstr(tagstring,"ENVFILESTOP") ) {
    cout << endl << "In ReadEnvFile: didn't read ENVFILESTOP tag right";
    return;
  }

};

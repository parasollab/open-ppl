// $Id$
/////////////////////////////////////////////////////////////////////
//  Input.c
//
//  Created  7/22/98 Wookho Son
/////////////////////////////////////////////////////////////////////

#include "Input.h"
#include "Environment.h"
#include "FreeBody.h"
#include "FixedBody.h"

#include <string.h>
#include <vector.h>
#include <stdlib.h>
#include <math.h>

//#include <strstream.h>

#include "Cfg.h"
#include "CfgManager.h"
#include "Cfg_free.h"
#include "Cfg_fixed_PRR.h"
#include "Cfg_free_tree.h"
#include "Cfg_fixed_tree.h"
#include "Cfg_2D.h"

#include "util.h"

// Need for retreiving default parameters of each
// in executing the parameter "-defaults"
#include "LocalPlanners.h"
#include "DistanceMetrics.h"
#include "ConnectMapNodes.h"
#include "GenerateMapNodes.h"
#include "CollisionDetection.h"


#include <iostream.h>
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
                       //  default   min,max
        lineSegment      ("-lineSegment",        0,  0, 5000),
        usingClearance   ("-clearance",          0,  0,    1),
        addPartialEdge   ("-addPartialEdge",     0,  0,    1),
        posres           ("-posres",          0.05, -1000, 1000),
        orires           ("-orires", ORIENTATION_RES, -1000, 1000),
        bbox_scale       ("-bbox_scale",       2.0,  -50000, 50000),
        bbox             ("-bbox",""),
        collPair         ("-collPair","cM rT "),    // center of Mass
        freePair         ("-freePair","cM rV "),    // center of Mass
        calcClearance    ("-calcClear",          0,  0,    100),
	calcPenetration  ("-calcPenetration",    0,  0,    1) //set default as false.
        {

    //Cfg::CfgHelper=NULL; 
    cfgSet=false;
    lineSegment.PutDesc      ("INTEGER","");
    usingClearance.PutDesc   ("INTEGER","");
    addPartialEdge.PutDesc   ("INTEGER","");

    envFile.PutDesc    ("STRING ","");
    mapFile.PutDesc    ("STRING ","");
    inmapFile.PutDesc  ("STRING *causes Node Generation to be SKIPPED","");

    defaultFile.PutDesc("STRING ",
        " default filename prefix for common files");
    descDir.PutDesc    ("STRING ",
        " directory where program will look for the data"
        "\n\t\t\t    files specified in environment specification file");

    calcClearance.PutDesc    ("INTEGER","");
	calcPenetration.PutDesc  ("BOOLEAN","\t*NOTE*: Affect Only if -calcClear is set.");

    bbox.PutDesc       ("STRING ", "[Xmin,Xmax,Ymin,Ymax,Zmin,Zmax]"
                                    "\n\t\t\t(default calculated from environment)"
                                    "\n\t\t\tATTN: robust if no spaces are used.");
    bbox_scale.PutDesc        ("FLOAT  ","");
    posres.PutDesc            ("FLOAT  "," *CALCULATED*   position    resolution");
    orires.PutDesc            ("FLOAT  "," **HARDCODED**  orientation resolution");

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
    numCFGs = 0;
    for (i=0;i<MAX_CFG;++i)
    CFGstrings[i] = new n_str_param("-cfg");
    numDMs = 0;
    for (i=0;i<MAX_DM;++i)
        DMstrings[i]=new n_str_param("-dm");

    GNstrings[0]->PutDesc("STRING",
        "\n\t\t\tPick any combo: default BasicPRM"
        "\n\t\t\t  BasicPRM   nodes INT (number of nodes, default 10)"
        "\n\t\t\t  BasicOBPRM nodes  INT (number of nodes, default 10)"
        "\n\t\t\t             shells INT (number of shells, default 3)"
        "\n\t\t\t  OBPRM      nodes     INT (number of nodes, default 10)"
	"\n\t\t\t             shells    INT (number of shells, default 3)"
        "\n\t\t\t             collPair  STRING STRING (default cM, rT)"
        "\n\t\t\t                       Specify 2 of the following recognized mnemonics:"
        "\n\t\t\t                       cM    \"center of mass\""
        "\n\t\t\t                       rV    \"random vertex\""
        "\n\t\t\t                       rT    \"point in random triangle\""
        "\n\t\t\t                       rE    \"random extreme vertex\""
        "\n\t\t\t                       rW    \"point in random weighted triangle\""
        "\n\t\t\t                       cM_rV \"cg/random vertex\""
        "\n\t\t\t                       rV_rT \"random vertex/point in random          triangle\""
        "\n\t\t\t                       rV_rW \"random vertex/point in random weighted triangle\""
        "\n\t\t\t                       N_rT  \"normal of random triangle\""
        "\n\t\t\t                       all   \"all of the above\""
        "\n\t\t\t             freePair  STRING STRING (default cM, rV)"
        "\n\t\t\t                       Same as above"
        "\n\t\t\t             clearFact DOUBLE (clearance factor, default 1.0)"
        "\n\t\t\t             pctSurf   DOUBLE (proportion surface nodes, default 1.0)"
        "\n\t\t\t  GaussPRM   nodes INT (number of nodes, default 10)" 
        "\n\t\t\t             d     INT (distance, default based on environment)"
        "\n\t\t\t  BasicMAPRM nodes INT (number of nodes, default 10)"
        );
    CNstrings[0]->PutDesc("STRING",
        "\n\t\t\tPick any combo: default closest 10"
        "\n\t\t\t  random"
        "\n\t\t\t  closest        INT         (k:5)"
        "\n\t\t\t  closestVE      INT         (k:5)"
        "\n\t\t\t  components     INT INT     (kpairs:3 smallcc:4)"
        "\n\t\t\t  obstBased      INT INT     (other:10 self:3)"
        "\n\t\t\t  RRTexpand      INT INT INT (iter:10 factor:3 cc:3)"
        "\n\t\t\t  RRTcomponents  INT INT INT (iter:10 factor:3 cc:3)"
        "\n\t\t\t  modifiedLM     INT INT INT (kpairs:5, add:10, rfactor:2)"
        );
    LPstrings[0]->PutDesc("STRING",
        "\n\t\t\tPick any combo: default straightline rotate_at_s 0.5"
        "\n\t\t\t  straightline"
        "\n\t\t\t  rotate_at_s      FLOAT   (def,s=0.5)"
        "\n\t\t\t  a_star_distance  INT INT (def,tries=6,neighbors=3)"
        "\n\t\t\t  a_star_clearance INT INT (def,tries=6,neighbors=3)"
        "\n\t\t\t  approx_spheres   INT     (def,n=3)"
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

    char Cfg_string_1[300];

    strcpy(Cfg_string_1, "\n\t\t\tPick one: default ");
    strcat(Cfg_string_1, Cfg::GetName());
    if (!strcmp(Cfg::GetName(), "Cfg_free_rigid"))
      strcat(Cfg_string_1, " (i.e. Cfg_free)");
    strcat(Cfg_string_1,"\n\t\t\t  Cfg_free_rigid");
    strcat(Cfg_string_1,"\n\t\t\t  Cfg_fixed_PRR");
    strcat(Cfg_string_1,"\n\t\t\t  Cfg_free_tree");
    strcat(Cfg_string_1,"\n\t\t\t  Cfg_fixed_tree");
    strcat(Cfg_string_1,"\n\t\t\t  Cfg_2D");

    CFGstrings[0]->PutDesc("STRING", Cfg_string_1);

    DMstrings[0]->PutDesc("STRING",
        "\n\t\t\tPick one: default scaledEuclidean 0.9"
        "\n\t\t\t  euclidean"
        "\n\t\t\t  scaledEuclidean FLOAT (default,0.5)"
	"\n\t\t\t  minkowski FLOAT FLOAT FLOAT (default 3 3 0.333)"
	"\n\t\t\t  manhattan"
	"\n\t\t\t  com"

        );

    strcpy(commandLine,"");

    // initialize init file contents
    multibodyCount = 0;

   for (int m=0; m<MAX_MULTIBODY; m++){
      BodyCount[m] = 0;
      FreeBodyCount[m] = 0;
      FixedBodyCount[m] = 0;
      for (i=0; i<MAX_FIXEDBODY; i++)
        fixedbodyFileName[m][i] = new char[FILENAME_LENGTH];
      for (i=0; i<MAX_FREEBODY; i++)
        freebodyFileName[m][i] = new char[FILENAME_LENGTH];
      connectionCount[m] = 0;
   }

   nprocs = 1;
};

Input::~Input() {
  return;
}


//===================================================================
//  Input class
//  ReadCfgType(istream &)
//  Guang 12/14/00
//===================================================================
void Input::ReadCfgType(istream &is) {
    if(is) {
        numofJoints = 3;
        is >> cfgName;
        if(is) {
        is >> numofJoints;
        if(numofJoints < 0 || numofJoints > 1000) {
            cerr << "Error in Input.cpp, wrong input for numofJoints !" << endl;
            exit(2);
        }
        }
        if (!(strncmp(cfgName,"Cfg_free_rigid",14))) {
            Cfg::CfgHelper = new Cfg_free();
            cfgSet=true;
        }else if (!(strncmp(cfgName,"Cfg_fixed_PRR",13))) {
            Cfg::CfgHelper = new Cfg_fixed_PRR();
            cfgSet=true;
        }else if (!(strncmp(cfgName,"Cfg_free_tree",15))) {
            Cfg::CfgHelper = new Cfg_free_tree(numofJoints);
            cfgSet=true;
	} else if (!(strncmp(cfgName,"Cfg_fixed_tree", 14))) {
	    Cfg::CfgHelper = new Cfg_fixed_tree(numofJoints);
	    cfgSet = true;
	} else if (!(strncmp(cfgName,"Cfg_2D", 6))) {
	    Cfg::CfgHelper = new Cfg_2D();
	    cfgSet = true;
        }
    } // default is Cfg_free().
}



//===================================================================
//  Input class
//  ReadCommandLine
//===================================================================
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
        } else if ( lineSegment.AckCmdLine(&i, argc, argv) ) {
        } else if ( usingClearance.AckCmdLine(&i, argc, argv) ) {
        } else if ( addPartialEdge.AckCmdLine(&i, argc, argv) ) {
	} else if ( calcClearance.AckCmdLine(&i, argc, argv) ) {
	} else if ( calcPenetration.AckCmdLine(&i, argc, argv) ) {
        } else if ( bbox.AckCmdLine(&i, argc, argv) ) {
        } else if ( bbox_scale.AckCmdLine(&i, argc, argv) ) {
        } else if ( posres.AckCmdLine(&i, argc, argv) ) {
        } else if ( orires.AckCmdLine(&i, argc, argv) ) {

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
    } else if ( CFGstrings[numCFGs]->AckCmdLine(&i, argc, argv) ) {
                numCFGs++;
        } else if ( DMstrings[numDMs]->AckCmdLine(&i, argc, argv) ) {
                numDMs++;
        } else {
                cout << "\nERROR: Don\'t understand \""<< argv[i]<<"\"";
                throw BadUsage();
        } //endif
    } //endfor i

    istrstream cfgstr(CFGstrings[0]->GetValue());
    ReadCfgType(cfgstr);

    //-- Do some clean up and final checking

    if ( !defaultFile.IsActivated() &&
     !( envFile.IsActivated() && mapFile.IsActivated() )){
        throw BadUsage();
    }

    if ( inmapFile.IsActivated() ){
        VerifyFileExists(inmapFile.GetValue(),EXIT);
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
    	_os << "\n  "; calcClearance.PrintUsage(_os);
    	_os << "\n  "; calcPenetration.PrintUsage(_os);
        _os << "\n  "; lineSegment.PrintUsage(_os);
        _os << "\n  "; usingClearance.PrintUsage(_os);
        _os << "\n  "; addPartialEdge.PrintUsage(_os);
        _os << "\n  "; bbox.PrintUsage(_os);
        _os << "\n  "; bbox_scale.PrintUsage(_os);
        _os << "\n  "; posres.PrintUsage(_os);
        _os << "\n  "; orires.PrintUsage(_os);
        _os << "\n";
        _os << "\n  "; GNstrings[0]->PrintUsage(_os);
        _os << "\n  "; CNstrings[0]->PrintUsage(_os);
        _os << "\n  "; LPstrings[0]->PrintUsage(_os);
        _os << "\n  "; CDstrings[0]->PrintUsage(_os);
        _os << "\n  "; CFGstrings[0]->PrintUsage(_os);
        _os << "\n  "; DMstrings[0]->PrintUsage(_os);

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

  _os <<"\n"<<setw(FW)<<"calcClearance"<<"\t"<<calcClearance.GetValue();
  _os <<"\n"<<setw(FW)<<"calcPenetration"<<"\t"<<calcPenetration.GetValue();
  _os <<"\n"<<setw(FW)<<"lineSegment"<<"\t"<<lineSegment.GetValue();
  _os <<"\n"<<setw(FW)<<"usingClearance"<<"\t"<<usingClearance.GetValue();
  _os <<"\n"<<setw(FW)<<"addPartialEdge"<<"\t"<<addPartialEdge.GetValue();
  _os <<"\n"<<setw(FW)<<"bbox"<<"\t"<<bbox.GetValue();
  _os <<"\n"<<setw(FW)<<"bbox_scale"<<"\t"<<bbox_scale.GetValue();
  _os <<"\n"<<setw(FW)<<"posres"<<"\t"<<posres.GetValue();
  _os <<"\n"<<setw(FW)<<"orires"<<"\t"<<orires.GetValue();

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
  for(i=0;i<numCFGs;++i)
    _os << "\n"<<setw(FW)<< "CFGstrings"<<i
        <<"\t"<<CFGstrings[i]->GetValue();
  for(i=0;i<numDMs;++i)
    _os << "\n"<<setw(FW)<< "DMstrings"<<i
        <<"\t"<<DMstrings[i]->GetValue();

        _os << "\n\n";
};


// if the input parameter is "-defaults", print defaults and exit program
void
Input::PrintDefaults(){

   int FW  = 30;
   int F   =  8;

   cout << setw(FW) << "defaultFile : no default string for this parameter" << endl;
   cout << setw(FW) << " (those parameters not listed here have no default value)" << endl << endl;
   cout << setw(FW) << "line segment" << " (" << lineSegment.GetFlag() << ") : " <<
            lineSegment.GetDefault() << endl << endl;
   cout << setw(FW) << "using clearance" << " (" << usingClearance.GetFlag() << ") : " <<
            usingClearance.GetDefault() << endl << endl;
   cout << setw(FW) << "add partial edge" << " (" << addPartialEdge.GetFlag() << ") : " <<
            addPartialEdge.GetDefault() << endl << endl;
   cout << setw(FW) << "position resolution" << " (" << posres.GetFlag() << ") : " <<
            posres.GetDefault() << endl << endl;
   cout << setw(FW) << "orientation resolution" << " (" << orires.GetFlag() << ") : " <<
            orires.GetDefault() << endl << endl;
   cout << setw(FW) << "bounding box scale" << " (" << bbox_scale.GetFlag() << ") : " <<
            bbox_scale.GetDefault() << endl << endl;
   cout << setw(FW) << "calculate clearance" << " (" << calcClearance.GetFlag() << ") : " << 
            calcClearance.GetDefault() << endl << endl;
   cout << setw(FW) << "calculate clearance" << " (" << calcPenetration.GetFlag() << ") : " << 
            calcPenetration.GetDefault() << endl << endl;
   cout << setw(FW) << "Cfg" << " (" << CFGstrings[0]->GetFlag() << ") : " << Cfg::GetName() << endl << endl;

   // get default parameters by initializing each class
   // for some, need to call UserInit() to add sets so that we can display

   // there are possibly many sets available and the particular one
   // "selected" is recognized by the setid number.

   // Generate Map Nodes
   GenerateMapNodes gn;         // default value of gnInfo.gnsetid is set
   Environment env;
   cout << setw(FW) << "Generate Map Nodes" << " (" << GNstrings[0]->GetFlag() <<
      ") : default set id = " << gn.gnInfo.gnsetid;
   gn.UserInit(this, &env);      // to dsiplay, add GN sets
   gn.generators.DisplayGNSet(gn.gnInfo.gnsetid);

   // Connect Map Nodes
   ConnectMapNodes cn;          // default value of cnInfo.cnsetid is set
   cout << setw(FW) << endl << endl << "Connect Map Nodes" << " (" << CNstrings[0]->GetFlag() <<
      ") : default set id = " << cn.cnInfo.cnsetid;
   cn.UserInit(this, &env);           // to dsiplay, add CN sets
   cn.connectors.DisplayCNSet(cn.cnInfo.cnsetid);

   // Local Planners
   LocalPlanners lp;
   // this default is already set in ConnectMapNodes::DefaultInit()
   cout << setw(FW) << endl << endl << "Local Planners" << " (" << LPstrings[0]->GetFlag() <<
      ") : default set id = " << cn.cnInfo.lpsetid;
   lp.planners.DisplayLPSet(cn.cnInfo.lpsetid); // LP set was already added

   // Distance Metric
   DistanceMetric dm;
   // this default is already set in ConnectMapNodes::DefaultInit()
   cout << setw(FW) << endl << endl << "Distance Metric" << " (" << DMstrings[0]->GetFlag() <<
      ") : default set id = " << cn.cnInfo.dmsetid;
   dm.distanceMetrics.DisplayDMSet(cn.cnInfo.dmsetid); // DM set was already added

   // Collision Detection
   CollisionDetection cd;
   // this default is already set in ConnectMapNodes::DefaultInit()
   cout << setw(FW) << endl << endl << "Collision Detection" << " (" << CDstrings[0]->GetFlag() <<
      ") : default set id = " << cn.cnInfo.cdsetid;
   cd.UserInit(this, &gn, &cn);
   cd.collisionCheckers.DisplayCDSet(cn.cnInfo.cdsetid);

   cout << endl << flush;
}



//===================================================================
//  Input class
//  Read
//===================================================================
void Input::Read(int action) {
  int envFormatVersion= ENV_VER_LEGACY;
  char t;

  VerifyFileExists(envFile.GetValue(),action);

  // open file and read first field
  ifstream is(envFile.GetValue());
  char string1[32];
  //is >> string1;

#define LINEMAX 256
  // if first field is a comment delimiter
   while   ((t=is.peek())=='#') {
         char line[LINEMAX];
          is.getline(line,LINEMAX,'\n');
        char string2[32];
        char string3[32];
        if (   strstr(line, "Environment") ){
                  sscanf(&line[1],"%s %s %d",string2,string3,&envFormatVersion);
               if( !strstr(string3, "Version")      ) {
                   cerr << "\nREAD ERROR: bad file format in \""
                        << envFile.GetValue() << "\"";
                   cerr << "\n            something is wrong w/ the following\n"
                    << "\n            "<<string1<<" "<<string2<<" "<<string3
                    <<"\n\n";
                   if(action==EXIT)
                   exit(-1);
               }
        } 
       else if (strstr(line, "Cfg")  && !cfgSet){
       // if Environment has Cfg info and Cfg type is not set through command 
       // line, we use this string from Environment instead to setup Cfg type.
           istrstream cfgstr(&line[1]);
	   ReadCfgType(cfgstr);
       }
 
  }
  Read(is,envFormatVersion,action);

  is.close();


};


void Input::Read(istream & _is, int envFormatVersion,int action) {

    int  i;
    char string[32];
    char tmpFilename[FILENAME_LENGTH*2];

    //Variable added by Xinyu Tang, 03/27/2002
    //Usage: to see whether this version of obprm can distinguish the external obstacles & 
    //       internal obstacles;
    bool bVersionInternal = true;

    switch(envFormatVersion) {
      case ENV_VER_20020327:
	//	bVersionInternal = true;
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

    _is >> multibodyCount;      // # of MultiBodys'

    for (int m=0; m<multibodyCount; m++){
        
      bool bInternal = false;

        //---------------------------------------------------------------
        // Read tag
        //---------------------------------------------------------------
        readfield(_is, &string);              // Tag, "MultiBody"
        readfield(_is, &string);              // Tag, "Active/Passive"

	//	_is.peek >> skipws;
	char cPeek = _is.peek();
  	while ((cPeek== ' ') || (cPeek == '\n'))
	  {
	    _is.get();
	    cPeek = _is.peek();
	  }

	if (cPeek =='I')
	  {
	    readfield(_is, &string);
	    if (!strncmp(string, "Internal", 8))
	      {
		bInternal = true;
		bBodyInternal[m] = true;
	      }
	  }

	readfield(_is, &BodyCount[m]);        // number of bodies

        for (i=0; i<BodyCount[m]; i++){
           readfield(_is, &string,comments[m][i]);// Tag (FixedBody or FreeBody)
            if (!strncmp(string, "FixedBody", 10)){
               isFree[m][i] = 0;
               FixedBodyCount[m]++;

               _is >> BodyIndex[m][i];   // fixed body index
               BodyIndex[m][i] = FixedBodyCount[m] - 1;

               // Data file name for fixed body
               _is >> tmpFilename;
               strcpy(fixedbodyFileName[m][FixedBodyCount[m]-1],
                                                descDir.GetValue());
               strcat(fixedbodyFileName[m][FixedBodyCount[m]-1],
                                                tmpFilename);
               VerifyFileExists(fixedbodyFileName[m][FixedBodyCount[m]-1],action);
               strcpy(tmpFilename,"");

               fixedbodyPosition[m][FixedBodyCount[m]-1] = Vector3D(_is);
               bodyOrientation[m][i] = Vector3D(_is);
               fixedbodyOrientation[m][FixedBodyCount[m]-1] = 
                               Orientation(Orientation::FixedXYZ,
                               bodyOrientation[m][i][2]*TWOPI/360.0, 
                               bodyOrientation[m][i][1]*TWOPI/360.0, 
                               bodyOrientation[m][i][0]*TWOPI/360.0);
            } else { // FreeBody
               isFree[m][i] = 1;
               FreeBodyCount[m]++;

               _is >> BodyIndex[m][i];    // free body index
               BodyIndex[m][i] = FreeBodyCount[m] - 1;
               _is >> tmpFilename;
               strcpy(freebodyFileName[m][FreeBodyCount[m]-1],
                                                descDir.GetValue());
               strcat(freebodyFileName[m][FreeBodyCount[m]-1],
                                                tmpFilename);
               VerifyFileExists(freebodyFileName[m][FreeBodyCount[m]-1],action);
               strcpy(tmpFilename,"");

               if (i==0){  // for the very first body
                   // this whole 'if' statements should be got rid of later. 
                   // not in use. the only purpose here is to be able to use 
                   // old environment file.
                   freebodyPosition[m] = Vector3D(_is);

                   bodyOrientation[m][i] = Vector3D(_is);
                   freebodyOrientation[m] = Orientation(Orientation::FixedXYZ,
                               bodyOrientation[m][i][2]*TWOPI/360.0, 
                               bodyOrientation[m][i][1]*TWOPI/360.0, 
                               bodyOrientation[m][i][0]*TWOPI/360.0);
               }
            } // endelse FreeBody

        } //endfor i

        readfield(_is, &string);     // Tag, "Connection"
        _is >> connectionCount[m];   // # of connections

        /////////////////////////////////////////////////////////////////
        // Read in connectionship data
        /////////////////////////////////////////////////////////////////
        for (i=0; i<connectionCount[m]; i++){
                _is >> previousBodyIndex[m][i];              // first body
                _is >> nextBodyIndex[m][i];                  // second body

                readfield(_is, &string);             // Tag, "Actuated/NonActuated"

                transformPosition[m][i] = Vector3D(_is);
                Vector3D angles = Vector3D(_is);
                transformOrientation[m][i] = Orientation(Orientation::FixedXYZ,
                                 angles[2]*TWOPI/360.0, 
                                 angles[1]*TWOPI/360.0, 
                                 angles[0]*TWOPI/360.0);

                _is >> dhparameters[m][i].alpha;          // DH parameter, alpha
                _is >> dhparameters[m][i].a;              // DH parameter, a
                _is >> dhparameters[m][i].d;              // DH parameter, d
                _is >> dhparameters[m][i].theta;          // DH parameter, theta

                readfield(_is, &string);   // Tag, "Revolute" or "Prismatic"
                if (!strncmp(string, "Revolute", 9))
                   connectionType[m][i] = 0;              // Revolute type
                else
                   connectionType[m][i] = 1;              // Prismatic type

                positionToDHFrame[m][i] = Vector3D(_is);
                angles = Vector3D(_is);
                orientationToDHFrame[m][i] = Orientation(Orientation::FixedXYZ,
                                 angles[2]*TWOPI/360.0, 
                                 angles[1]*TWOPI/360.0, 
                                 angles[0]*TWOPI/360.0);
        } //endfor i

   } //endfor m
};


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

//===================================================================
//  Input class
//  PrintBodyNames    for debugging purposes
//===================================================================
void Input::PrintBodyNames() {
  for (int m=0; m<MAX_MULTIBODY; m++){
    if (strcmp(*fixedbodyFileName[m], "") ||
        strcmp(*freebodyFileName[m], "") ) {
           cout << "\n fixedbodyFileName["<<m<<"] = ("
                << *fixedbodyFileName[m] << ")  ";
           cout << " freebodyFileName["<<m<<"] = ("
                << *freebodyFileName[m] << ")  ";
    }
  }
};

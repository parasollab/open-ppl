// $Id$
/////////////////////////////////////////////////////////////////////
//  Input.c
//
//  Created  7/22/98 Wookho Son
/////////////////////////////////////////////////////////////////////

#include "Input.h"
#include "Environment.h"
#include "Contact.h"
#include "FreeBody.h"
#include "FixedBody.h"

#include <string.h>
#include <vector.h>
#include <stdlib.h>
#include <math.h>

#include <strstream.h>

#include "Cfg.h"
#include "CfgManager.h"
#include "Cfg_free.h"
#include "Cfg_fixed_PRR.h"
#include "Cfg_free_serial.h"


// Need for retreiving default parameters of each
// in executing the parameter "-defaults"
#include "LocalPlanners.h"
#include "DistanceMetrics.h"
#include "ConnectMapNodes.h"
#include "GenerateMapNodes.h"
#include "CollisionDetection.h"
// default of Cfg is set in Cfg.cpp and seems the information cannot be
// retreived in Input.cpp. So, if the default is changed in Cfg.cpp,
// it must be changed here also for correct information display
char Cfg_default_string[] = "Cfg_free_rigid";

//----------------------------------------
//  string (mostly) parameter ( n fields acknowledged from argv )
//----------------------------------------
n_str_param ::n_str_param()
                      :str_param <char*>() {};
n_str_param ::n_str_param(char *_flag)
                      :str_param <char*>(_flag) {};
n_str_param ::n_str_param(char *_flag,char* _initialValue)
                      :str_param<char*>(_flag,_initialValue){};


//===================================================================
//  string (mostly) parameter ( n fields acknowledged from argv )
//  Constructors  & other methods
//===================================================================
bool n_str_param::
AckCmdLine(int *i, int argc, char** argv){

    if (  strlen(flag)==strlen(argv[*i]) &&
        !strncmp(argv[*i],flag,strlen(flag))  ) {
       SetValue("");      // overwrite any default that was set
       bool stop = false;
       while (!stop) {
          // if arguments exhausted
          if (++(*i) == argc || !strncmp(argv[*i],"-",1) ) {
             stop = true;
          } else { //-- concatenate to string
             strcat(tvalue,argv[*i]);
             strcat(tvalue," ");
          }
       } // endwhile
       --(*i);

       if (VerifyValidValue(tvalue)) {
          activated = true;
          return true;
       } else {
          cout << "\nERROR: " << flag << "  missing a VALUE";
          throw BadUsage();
       }

    }

    return false;
};
bool n_str_param::
VerifyValidValue(char *_val){
    if (!strcmp(tvalue,""))
      return false;
    else
      return true;

};


/*
public:
        n_str_param()
                :str_param<char*>(){};
        n_str_param(char *_flag)
                :str_param<char*>(_flag){};
        n_str_param(char *_flag,char* _initialValue)
                :str_param<char*>(_flag,_initialValue){};
        bool AckCmdLine(int *i, int argc, char** argv);
protected:
        bool VerifyValidValue(char* _val);
};

*/

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
        numEdges         ("-edges",              5,  1, 5000),
        numShells        ("-nshells",            3,  1,   50),
        numNodes         ("-nodes",             10,  1,5000000),
        numNodesPerObst  ("-nodesPerObst",      10,  1, 5000),
        lineSegment      ("-lineSegment",        0,  0, 5000),
        usingClearance   ("-clearance",          0,  0,    1),
    addPartialEdge   ("-addPartialEdge",     0,  0,    1),
        proportionSurface("-proportionSurface",1.0,  0,  1.0),
        posres           ("-posres",          0.05, -1000, 1000),
        orires           ("-orires", ORIENTATION_RES, -1000, 1000),
        bbox_scale       ("-bbox_scale",       2.0,  -50000, 50000),
        bbox             ("-bbox",""),
        collPair         ("-collPair","cM rT "),    // center of Mass
        freePair         ("-freePair","cM rV "),    // center of Mass
        calcClearance    ("-calcClear",          0,  0,    1) 
        {
    // brc added
/*
    ARGSTRING_LENGTH = 256;
     MAX_CN           =  10;
    MAX_GN           =  10;
    MAX_LP           =  10;
    MAX_CD           =  10;
    MAX_DM           =  10;
    MAX_CFG          =  10;

    FILENAME_LENGTH  =  80;   // GMS stuff uses these
    MAX_MULTIBODY    =  50;
    MAX_CONNECTION   =  50;
    MAX_FIXEDBODY    =  50;
    MAX_FREEBODY     =  50;
*/


    numEdges.PutDesc         ("INTEGER","");
    numShells.PutDesc        ("INTEGER","");
    numNodes.PutDesc         ("INTEGER","");
    numNodesPerObst.PutDesc  ("INTEGER","");
    proportionSurface.PutDesc("FLOAT  ","");
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
    collPair.PutDesc   ("STRING STRING",
        "\n\t\t\tSpecify 2 of the following recognized mnemonics:"
        "\n\t\t\t  cM    \"center of mass\""
        "\n\t\t\t  rV    \"random vertex\""
        "\n\t\t\t  rT    \"point in random triangle\""
        "\n\t\t\t  rE    \"random extreme vertex\""
        "\n\t\t\t  rW    \"point in random weighted triangle\""
        "\n\t\t\t  cM_rV \"cg/random vertex\""
        "\n\t\t\t  rV_rT \"random vertex/point in random          triangle\""
        "\n\t\t\t  rV_rW \"random vertex/point in random weighted triangle\""
    "\n\t\t\t  N_rT  \"normal of random triangle\""
        "\n\t\t\t  all   \"all of the above\""
        );
    freePair.PutDesc   ("STRING STRING","\n\t\t\tSame as above"
        );

    calcClearance.PutDesc    ("INTEGER","");

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
        "\n\t\t\t  BasicPRM"
        "\n\t\t\t  BasicOBPRM"
        "\n\t\t\t  OBPRM    DOUBLE  (clearanceFactor:1.0)"
        "\n\t\t\t  GaussPRM INTEGER (default,based on environment)"
        "\n\t\t\t  BasicMAPRM"
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
        );

    char Cfg_string_1[300];

    strcpy(Cfg_string_1, "\n\t\t\tPick one: default ");
    // Cfg_default_string was defined in line 38, and used in line 604 again.
    // If default Cfg in Cfg.cpp is changed, then the initialization of
    // Cfg_default_string in line 38 must be changed accordingly.
    strcat(Cfg_string_1, Cfg_default_string);
    if (!strcmp(Cfg_default_string, "Cfg_free_rigid"))
      strcat(Cfg_string_1, " (i.e. Cfg_free)");
    strcat(Cfg_string_1,"\n\t\t\t  Cfg_free_rigid");
    strcat(Cfg_string_1,"\n\t\t\t  Cfg_fixed_PRR");
    strcat(Cfg_string_1,"\n\t\t\t  Cfg_free_serial");

    CFGstrings[0]->PutDesc("STRING", Cfg_string_1);

    DMstrings[0]->PutDesc("STRING",
        "\n\t\t\tPick one: default scaledEuclidean 0.9"
        "\n\t\t\t  euclidean"
        "\n\t\t\t  scaledEuclidean FLOAT (default,0.5)"
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
        //fprintf(stderr,"In Input destru\n");
       return;
    for (int m=0; m<MAX_MULTIBODY; m++){
        fprintf(stderr,"%d\n",m);
        delete []fixedbodyFileName[m];
    delete []freebodyFileName[m];
    }
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

  //-- Initialize error message
  char ERROR_MutualExclusive[300];
  sprintf(ERROR_MutualExclusive,"\nERROR: \"%s\" & \"%s\" options are "
    "mutually exclusive on the command line.\n",
    numNodes.GetFlag(),numNodesPerObst.GetFlag());

#ifdef USE_VCLIP
   cdtype= VCLIP;
#endif

#ifdef USE_RAPID
   cdtype = RAPID;
#endif

#ifdef USE_CSTK
    cdtype = CSTK;
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
        } else if ( numNodes.AckCmdLine(&i, argc, argv) ){
        } else if ( numEdges.AckCmdLine(&i, argc, argv) ) {
        } else if ( numNodesPerObst.AckCmdLine(&i, argc, argv) ) {
        } else if ( numShells.AckCmdLine(&i, argc, argv) ) {
        } else if ( lineSegment.AckCmdLine(&i, argc, argv) ) {
        } else if ( usingClearance.AckCmdLine(&i, argc, argv) ) {
    } else if ( addPartialEdge.AckCmdLine(&i, argc, argv) ) {
        } else if ( collPair.AckCmdLine(&i, argc, argv) ) {
        } else if ( freePair.AckCmdLine(&i, argc, argv) ) {
	} else if ( calcClearance.AckCmdLine(&i, argc, argv) ) {
        } else if ( proportionSurface.AckCmdLine(&i, argc, argv) ) {
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
    if(cfgstr) {
        numofJoints = 3;
        cfgstr >> cfgName;
        if(cfgstr) {
        cfgstr >> numofJoints;
        if(numofJoints < 0 || numofJoints > 1000) {
            cerr << "Error in Input.c, wrong input for numofJoints !" << endl;
            exit(2);
        }
        }
        if (!(strncmp(cfgName,"Cfg_free_rigid",14))) {
            Cfg::CfgHelper = new Cfg_free();
        }else if (!(strncmp(cfgName,"Cfg_fixed_PRR",13))) {
            Cfg::CfgHelper = new Cfg_fixed_PRR();
        }else if (!(strncmp(cfgName,"Cfg_free_serial",15))) {
            Cfg::CfgHelper = new Cfg_free_serial(numofJoints);
            // to be done later, parameter for Cfg_free_serial.
        }
    } // default is Cfg_free().


    //-- Do some clean up and final checking

    if ( !defaultFile.IsActivated() &&
     !( envFile.IsActivated() && mapFile.IsActivated() )){
        throw BadUsage();
    }

    if ( inmapFile.IsActivated() ){
        VerifyFileExists(inmapFile.GetValue());
    }

    if ( numNodesPerObst.IsActivated() && numNodes.IsActivated() ) {
    cout << ERROR_MutualExclusive;
    throw BadUsage();
    }

    descDir.VerifyValidDirName();

    //-- Verify INPUT file exists
    VerifyFileExists(envFile.GetValue());


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
        _os << "\n  "; numNodes.PrintUsage(_os);
        _os << "\n  "; numEdges.PrintUsage(_os);
        _os << "\n  "; numNodesPerObst.PrintUsage(_os);
        _os << "\n  "; proportionSurface.PrintUsage(_os);
        _os << "\n  "; numShells.PrintUsage(_os);
        _os << "\n  "; collPair.PrintUsage(_os);
        _os << "\n  "; freePair.PrintUsage(_os);
	_os << "\n  "; calcClearance.PrintUsage(_os);
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

  _os <<"\n"<<setw(FW)<<"numNodes"<<"\t"<<setw(F)<< numNodes.GetValue();
  _os <<"\n"<<setw(FW)<<"numEdges"<<"\t"<<setw(F)<< numEdges.GetValue();
  _os <<"\n"<<setw(FW)<<"numNodesPerObst"<<"\t"<<setw(F)<<numNodesPerObst.GetValue();
  _os <<"\n"<<setw(FW)<<"proportionSurface"<<"\t"<<setw(F)<<proportionSurface.GetValue
();
  _os <<"\n"<<setw(FW)<<"numShells"<<"\t"<<setw(F)<< numShells.GetValue();
  _os <<"\n"<<setw(FW)<<"collPair"<<"\t"<<collPair.GetValue();
  _os <<"\n"<<setw(FW)<<"freePair"<<"\t"<<freePair.GetValue();
  _os <<"\n"<<setw(FW)<<"calcClearance"<<"\t"<<calcClearance.GetValue();
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
   cout << setw(FW) << "number of edges" << " (" << numEdges.GetFlag() << "): " <<
            numEdges.GetDefault() << endl << endl;
   cout << setw(FW) << "number of shells" << " (" << numShells.GetFlag() << ") : " <<
            numShells.GetDefault() << endl << endl;
   cout << setw(FW) << "number of nodes" << " (" << numNodes.GetFlag() << ") : " <<
            numNodes.GetDefault() << endl << endl;
   cout << setw(FW) << "number of nodes per obstacle" << " (" << numNodesPerObst.GetFlag() <<
           ") : " << numNodesPerObst.GetDefault() << endl << endl;
   cout << setw(FW) << "line segment" << " (" << lineSegment.GetFlag() << ") : " <<
            lineSegment.GetDefault() << endl << endl;
   cout << setw(FW) << "using clearance" << " (" << usingClearance.GetFlag() << ") : " <<
            usingClearance.GetDefault() << endl << endl;
   cout << setw(FW) << "add partial edge" << " (" << addPartialEdge.GetFlag() << ") : " <<
            addPartialEdge.GetDefault() << endl << endl;
   cout << setw(FW) << "proportion surface" << " (" << proportionSurface.GetFlag() << ") : " <<
            proportionSurface.GetDefault() << endl << endl;
   cout << setw(FW) << "position resolution" << " (" << posres.GetFlag() << ") : " <<
            posres.GetDefault() << endl << endl;
   cout << setw(FW) << "orientation resolution" << " (" << orires.GetFlag() << ") : " <<
            orires.GetDefault() << endl << endl;
   cout << setw(FW) << "bounding box scale" << " (" << bbox_scale.GetFlag() << ") : " <<
            bbox_scale.GetDefault() << endl << endl;
   cout << setw(FW) << "collision pair" << " (" << collPair.GetFlag() << ") : " <<
            collPair.GetValue() << endl << endl;
   cout << setw(FW) << "free pair" << " (" << freePair.GetFlag() << ") : " <<
            freePair.GetValue() << endl << endl;
   cout << setw(FW) << "calculate clearance" << " (" << calcClearance.GetFlag() << ") : " << 
            calcClearance.GetDefault() << endl << endl;
   cout << setw(FW) << "Cfg" << " (" << CFGstrings[0]->GetFlag() << ") : " << Cfg_default_string << endl << endl;

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
   // this defulat is already set in ConnectMapNodes::DefaultInit()
   cout << setw(FW) << endl << endl << "Local Planners" << " (" << LPstrings[0]->GetFlag() <<
      ") : default set id = " << cn.cnInfo.lpsetid;
   lp.planners.DisplayLPSet(cn.cnInfo.lpsetid); // LP set was already added

   // Distance Metric
   DistanceMetric dm;
   // this defulat is already set in ConnectMapNodes::DefaultInit()
   cout << setw(FW) << endl << endl << "Distance Metric" << " (" << DMstrings[0]->GetFlag() <<
      ") : default set id = " << cn.cnInfo.dmsetid;
   dm.distanceMetrics.DisplayDMSet(cn.cnInfo.dmsetid); // DM set was already added

   // Collision Detection
   CollisionDetection cd;
   // this defulat is already set in ConnectMapNodes::DefaultInit()
   cout << setw(FW) << endl << endl << "Collision Detection" << " (" << CDstrings[0]->GetFlag() <<
      ") : default set id = " << cn.cnInfo.cdsetid;
   cd.UserInit(this, &gn, &cn);
   cd.collisionCheckers.DisplayCDSet(cn.cnInfo.cdsetid);

   cout << endl << flush;
}


bool
Input::VerifyFileExists(char *_fname){

  ifstream is(_fname);

  char ch;
  if (!is.get(ch)) {
     cout << "\nERROR: Can't open \"" << _fname << "\"" << endl;
     exit(1);
  }

  is.close();
  return true;
}

//===================================================================
//  Input class
//  Read
//===================================================================
void Input::Read() {

  VerifyFileExists(envFile.GetValue());

  ifstream is(envFile.GetValue());
  Read(is);
  is.close();
};

#define COMMENT_DELIMITER '#'
#define LINEMAX 256
template <class T> bool readfield (istream &_is, T *fred) {

  char c;
  char ThrowAwayLine[LINEMAX];

  while ( _is.get(c) )
    if (c == '#')
        _is.getline(ThrowAwayLine,LINEMAX,'\n');
    else if (c != '\n') {
        _is.putback(c);
        if (_is >> *fred ) return true;
        else               return false;
    }
  return false;
}

void Input::Read(istream & _is) {
    int  i;
    char string[32];
    char tmpFilename[FILENAME_LENGTH*2];

    _is >> multibodyCount;      // # of MultiBodys'

    for (int m=0; m<multibodyCount; m++){
        //---------------------------------------------------------------
        // Read tag
        //---------------------------------------------------------------
        readfield(_is, &string);              // Tag, "MultiBody"
        readfield(_is, &string);              // Tag, "Active/Passive"

        readfield(_is, &BodyCount[m]);   // Tag, "FreeBody" or "FixedBody"
        for (i=0; i<BodyCount[m]; i++){
            readfield(_is, &string);    // Tag for fixed body (FixedBody or FreeBody)

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
               VerifyFileExists(fixedbodyFileName[m][FixedBodyCount[m]-1]);
               strcpy(tmpFilename,"");

               //if (i==0){  // for the very first body
           // Now for every fixed body 10/15/99

               fixedbodyPosition[m][FixedBodyCount[m]-1] = Vector3D(_is);
               Vector3D angles = Vector3D(_is);
               fixedbodyOrientation[m][FixedBodyCount[m]-1] = Orientation(Orientation::FixedXYZ,
                     angles[2]*TWOPI/360.0, angles[1]*TWOPI/360.0, angles[0]*TWOPI/360.0);

               //}
            }
            else{
               isFree[m][i] = 1;
               FreeBodyCount[m]++;

               _is >> BodyIndex[m][i];    // free body index
               BodyIndex[m][i] = FreeBodyCount[m] - 1;
               _is >> tmpFilename;
               strcpy(freebodyFileName[m][FreeBodyCount[m]-1],
            descDir.GetValue());
               strcat(freebodyFileName[m][FreeBodyCount[m]-1],
            tmpFilename);
               VerifyFileExists(freebodyFileName[m][FreeBodyCount[m]-1]);
               strcpy(tmpFilename,"");

               if (i==0){  // for the very first body
           // this whole 'if' statements should be got rid of later. not in use.
           // the only purpose here is to be able to use old environment file. Guang 10/15/99
                   freebodyPosition[m] = Vector3D(_is);

               Vector3D angles = Vector3D(_is);
               freebodyOrientation[m] = Orientation(Orientation::FixedXYZ,
               angles[2]*TWOPI/360.0, angles[1]*TWOPI/360.0, angles[0]*TWOPI/360.0);
               }
            }

        }

        readfield(_is, &string);               // Tag, "Connection"
        _is >> connectionCount[m];   // # of connections

        /////////////////////////////////////////////////////////////////
        // Read in connectionship data
        /////////////////////////////////////////////////////////////////
        for (i=0; i<connectionCount[m]; i++){
                _is >> previousBodyIndex[m][i];              // first body
                _is >> nextBodyIndex[m][i];                  // second body

                readfield(_is, &string);        // Tag, "Actuated/NonActuated"

                transformPosition[m][i] = Vector3D(_is);
                Vector3D angles = Vector3D(_is);
                transformOrientation[m][i] = Orientation(Orientation::FixedXYZ,
           angles[2]*TWOPI/360.0, angles[1]*TWOPI/360.0, angles[0]*TWOPI/360.0);

                _is >> dhparameters[m][i].alpha;          // DH parameter, alpha
                _is >> dhparameters[m][i].a;              // DH parameter, a
                _is >> dhparameters[m][i].d;              // DH parameter, d
                _is >> dhparameters[m][i].theta;          // DH parameter, theta

                readfield(_is, &string); // Tag, "Revolute" or "Prismatic"
                if (!strncmp(string, "Revolute", 9))
                   connectionType[m][i] = 0;              // Revolute type
                else
                   connectionType[m][i] = 1;              // Prismatic type

        positionToDHFrame[m][i] = Vector3D(_is);
        angles = Vector3D(_is);
        orientationToDHFrame[m][i] = Orientation(Orientation::FixedXYZ,
                   angles[2]*TWOPI/360.0, angles[1]*TWOPI/360.0, angles[0]*TWOPI/360.0);
        }

   }
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

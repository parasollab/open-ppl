////////////////////////////////////////////////////
//
//  MyInput.c
//
//  derived class of Input
//
/////////////////////////////////////////////////////

#include "MyInput.h"
#include "util.h"

MyInput::MyInput():
  Input(),
  nodeValidationFlag("-validateNodes","complete"),
  edgeValidationFlag("-validateEdges","complete")
{
  nodeValidationFlag.PutDesc("STRING ",
			     "\n\t\t\t Type of node validation: default complete"
			     "\n\t\t\t   none         all nodes are added to roadmap"
			     "\n\t\t\t   approximate  only approx. valid nodes are added to roadmap"
			     "\n\t\t\t   complete     only valid nodes are added to roadmap");
  edgeValidationFlag.PutDesc("STRING FLOAT",
			     "\n\t\t\t Type of edge validation: default complete"
			     "\n\t\t\t   none              all edges are added to roadmap"
			     "\n\t\t\t   approximate FLOAT only approx. valid edges are added to roadmap"
			     "\n\t\t\t                     (FLOAT is level of resolution validated)"
			     "\n\t\t\t   complete          only valid edges are added to roadmap");
}


void 
MyInput::ReadCommandLine(int argc, char** argv) {

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
	} else if ( nodeValidationFlag.AckCmdLine(&i, argc, argv) ) {
	} else if ( edgeValidationFlag.AckCmdLine(&i, argc, argv) ) {

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

}


void 
MyInput::PrintUsage(ostream& _os, char *executablename){
  Input::PrintUsage(_os, executablename);

  cout.setf(ios::left,ios::adjustfield);

    _os << "\n  "; nodeValidationFlag.PrintUsage(_os);
    _os << "\n  "; edgeValidationFlag.PrintUsage(_os);

  cout.setf(ios::right,ios::adjustfield);

  _os << "\n\n";
}


void 
MyInput::PrintDefaults(){
  Input::PrintDefaults();
  int FW  = 30;

  cout << endl;
  cout << setw(FW) << "node validation type" << " (" << nodeValidationFlag.GetFlag() << ") : " 
       << nodeValidationFlag.GetValue() << endl << endl;
  cout << setw(FW) << "edge validation type" << " (" << edgeValidationFlag.GetFlag() << ") : "
       << edgeValidationFlag.GetValue() << endl << endl;
}


void 
MyInput::PrintValues(ostream& _os){
  Input::PrintValues(_os);

  int FW  = 20;

  _os << "\n" << setw(FW) << "validateNodes" << "\t" << nodeValidationFlag.GetValue();
  _os << "\n" << setw(FW) << "validateEdges" << "\t" << edgeValidationFlag.GetValue();

  _os << "\n\n";
}

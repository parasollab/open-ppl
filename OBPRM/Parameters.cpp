// $Id$
/////////////////////////////////////////////////////////////////////
//  Parameters.c
//
//  11/27/00 Lucia K. Dale
/////////////////////////////////////////////////////////////////////

#include "Defines.h"
///Modified for VC
#if defined(_WIN32)
#include <strstrea.h>
#else
#include <strstream.h>
#endif

#include "Parameters.h"

//----------------------------------------
//  string (mostly) parameter ( n fields acknowledged from argv )
//----------------------------------------
n_str_param ::n_str_param()
                      :str_param <char*>() {numStrings=0;};
n_str_param ::n_str_param(char *_flag)
                      :str_param <char*>(_flag) {numStrings=0;};
n_str_param ::n_str_param(char *_flag,char* _initialValue)
                      :str_param<char*>(_flag,_initialValue){numStrings=0;};
n_str_param ::n_str_param(const n_str_param& tmp) {
  if (tmp.tvalue!=NULL) {
    //strcpy(tvalue, tmp.tvalue);
    tvalue = strdup(tmp.tvalue);
  }
  if (tmp.tdefault!=NULL) {
    //strcpy(tdefault, tmp.tdefault);
    tdefault = strdup(tmp.tdefault);
  }
  if (tmp.flag!=NULL) {
    strcpy(flag, tmp.flag);
    //flag = strdup(tmp.flag);
  }
  activated = tmp.activated;
  if (tmp.typedesc!=NULL) {
    //strcpy(typedesc, tmp.typedesc);
    typedesc = strdup(tmp.typedesc);
  }
  if (tmp.desc!=NULL) {
    //strcpy(desc, tmp.desc);
    desc = strdup(tmp.desc);
  }
  numStrings = tmp.GetNumStrings();
};

///
int n_str_param::
GetNumStrings() const {
	return numStrings;
};

///
void n_str_param::
PutNumStrings(int _n){
	numStrings = _n;
};
		
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
       int currNumStrings = 0;
       while (!stop) {
          // if arguments exhausted
          if (++(*i) == argc || !strncmp(argv[*i],"-",1) ) {
             stop = true;
          } else if (numStrings>0){
	     if (currNumStrings<numStrings) currNumStrings++;
	     else                           stop = true;
          }

	  if (!stop) { //-- concatenate to string
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

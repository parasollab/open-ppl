////////////////////////////////////////////////////
//
//  MyInput.h
//
//  derived class of Input.h
//
/////////////////////////////////////////////////////
#ifndef MyInput_h
#define MyInput_h


#include "Input.h"

class MyInput : public Input {

public:
   MyInput();
   ~MyInput() {}

   virtual void ReadCommandLine(int argc, char** argv);

   void PrintUsage(ostream& _os, char *executablename);
   void PrintDefaults();
   void PrintValues(ostream& _os);

   n_str_param nodeValidationFlag;
   n_str_param edgeValidationFlag;
};

#endif

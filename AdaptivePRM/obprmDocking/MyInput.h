////////////////////////////////////////////////////
//
//  MyInput.h
//
//  derived class of Input.h
//
/////////////////////////////////////////////////////
#ifndef MyInput_h
#define MyInput_h


#include"Input.h"
#include"util.h"

class MyInput : public Input {

public:
   MyInput() : Input() {}
   ~MyInput() {}

   virtual void ReadCommandLine(int argc, char** argv);
   void ReadPotentialParameters(istream &is);
   void ReadPotentialParameters();
};

#endif

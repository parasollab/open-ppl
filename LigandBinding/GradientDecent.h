////////////////////////////////////////////////
//  GradientDecent.h
//
//  Author: Guang 09/13/2000
//
/////////////////////////////////////////////////
#include"Cfg.h"

class GradientDecent {

public:

    GradientDecent() {};
    ~GradientDecent() {};

    static bool findingLocalMin(Environment *env, vector<Cfg> &trace);
};
   

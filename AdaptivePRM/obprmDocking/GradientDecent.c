////////////////////////////////////////////////
//  GradientDecent.c
//
//  Author: Guang 09/13/2000
//
/////////////////////////////////////////////////
#include"GradientDecent.h"
#include"Environment.h"
#include"BioPotentials.h"

bool findLocalMinPotential(Environment *env, vector<Cfg> &trace,
int neb, int depth) {

   Cfg c = trace[trace.size()-1];
   Cfg minC = c;
   double minP = BioPotentials::GetPotential(minC,env);
cout << "minp In is " << minP << endl;

   double posRes = env->GetPositionRes();
   double oriRes = env->GetOrientationRes();
   bool decreasing = false;
   for(int i=0; i<neb; ++i) {
      Cfg incr = Cfg::GetRandomCfg(0.25, 0.25); // posRes*5, oriRes*5); // 2: heuristic value.
      Cfg tmp = c + incr;
      double tmpPotential = BioPotentials::GetPotential(tmp, env);
      if(tmpPotential < minP) {
           minP = tmpPotential;
           minC = tmp;
	   decreasing = true;
      }
   }
   if(decreasing) {
cout << "minP = " << minP << endl;
      trace.push_back(minC);
      if(depth > 0)
         return findLocalMinPotential(env, trace, neb, depth-1);
      else
         return false;
   } else {
      return true;
   }
}



bool GradientDecent::findingLocalMin(Environment *env, 
		vector<Cfg> &trace) {

    Cfg start = trace[trace.size()-1];
    double pot = BioPotentials::GetPotential(start, env);
    int numNeb = 10;
    int searchDepth = 20;

    return findLocalMinPotential(env, trace, numNeb, searchDepth);

}

//void DecendingEdge();
   

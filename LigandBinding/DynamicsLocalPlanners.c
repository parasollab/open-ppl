// $Id$
/////////////////////////////////////////////////////////////////////
//
//  DynamicsLocalPlanners.c
//
//  General Description
//	A derived class of class LocalPlanners. Mainly to be used
//      in protein folding, molecular docking as other computational
//	biology stuff(and others which need correct dynamics.) 
//
//	The correct dynamics is dispensable for good simulation and 
//	it can be coded somehow into LocalPlanners.
//
//
//  Created
//	06/10/2000 Guang Song 
//
//  Last Modified By:
//      xx/xx/xx  <Name>
//	06/28/2000 Guang Song: add Monte Carlo Integrator.
/////////////////////////////////////////////////////////////////////


#include "DynamicsLocalPlanners.h"
#include "BioPotentials.h"

double CalWeight(double deltaE) {
    static double KT = 2.494353; // KT = R*T = 8.31 * 300 /1000 (kJ mol^-1)
    if(1) {  // Monte Carlo Integrator
       if(deltaE > 0) 
	   return deltaE/KT;
       else 
	   return 0.00000001;
    } else {
       if(deltaE > 12)
           return deltaE/KT;
       else if(deltaE < -12)
           return 0.000001;
       else
           return log(1.0+exp(deltaE/KT));
   }
}


bool DynamicsLocalPlanners::IsConnected_straightline_simple(Environment *env,
     CollisionDetection *cd,DistanceMetric *dm, Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info) {

    double forwardEdgeWeight = 0.0, backwardEdgeWeight = 0.0;
    int n_ticks;
    Cfg incr=_c1.FindIncrement(_c2,&n_ticks,info->positionRes, info->orientationRes);

    Cfg tick=_c1;
    double prevPotential = BioPotentials::GetPotential(tick,env);
    double currPotential = 0;
    for(int i = 0; i < n_ticks ; i++){
        tick.Increment(incr);

        info->cd_cntr ++;
        if(info->checkCollision){
            if((currPotential = BioPotentials::GetPotential(tick,env)) > 
		BioPotentials::ConnectionThreshold()) // considered in Collision
                return false;
            else {
                forwardEdgeWeight  += CalWeight(currPotential-prevPotential);
                backwardEdgeWeight += CalWeight(prevPotential-currPotential);
                prevPotential = currPotential;
            }
        }
        if(info->savePath || info->saveFailedPath){
            info->path.push_back(tick);
        }
    }
    if(info->checkCollision){
        //info->edge = pair<WEIGHT,WEIGHT>(forwardEdgeWeight, backwardEdgeWeight);
        info->edge.first.Weight() += forwardEdgeWeight;
        info->edge.second.Weight() += backwardEdgeWeight;
    }
    return true;

};


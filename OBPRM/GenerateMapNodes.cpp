// $Id$
/////////////////////////////////////////////////////////////////////
//
//  GenerateMapNodes.cpp
//
//  General Description
//     This set of classes supports a "Map Node Generation Algobase".
//     This file contains the definitions of the prototypes
//     declared in "GenerateMapNodes.h".
//
//  Created
//      8/27/98  Lucia K. Dale
//
//  Last Modified
//      8/21/99  Lucia K. Dale  added GaussPRM
//      8/31/99  Lucia K. Dale  invoked method name change
//
/////////////////////////////////////////////////////////////////////
#include <vector.h>

#include "GenerateMapNodes.h"

#include "Roadmap.h"
#include "GMSPolyhedron.h"
#include "MultiBody.h"
#include "Environment.h"
#include "DistanceMetrics.h"
#include "BasicMAPRM.h"

#include "Clock_Class.h"
#include "util.h"

#define EXPANSION_FACTOR 100

///Static Data Member init
const int GenerateMapNodes::MAX_CONVERGE = 20;  

/////////////////////////////////////////////////////////////////////
//
//  METHODS for class GenerateMapNodes
//
/////////////////////////////////////////////////////////////////////
//==================================
// GenerateMapNodes class Methods: Constructors and Destructor
//==================================

GenerateMapNodes::
GenerateMapNodes()
{
    DefaultInit();
};

GenerateMapNodes::
~GenerateMapNodes() {
};

//==================================
// GenerateMapNodes class Methods: Map Node Generation Functions
//==================================

void
GenerateMapNodes::
DefaultInit()
{
    //-----------------------------------------------
    // The default algorithms are specified
    //-----------------------------------------------
    gnInfo.gnsetid = BASICPRM;
#if defined USE_CSTK
    gnInfo.cdsetid = CSTK;
#elif defined USE_RAPID
    gnInfo.cdsetid = RAPID;
#elif defined USE_PQP
    gnInfo.cdsetid = PQP;
#elif defined USE_VCLIP
    gnInfo.cdsetid = VCLIP;
#else
    #ifdef NO_CD_USE
       gnInfo.cdsetid = -1;
    #else
      #error You have to specify at least one collision detection library.
    #endif
#endif
    gnInfo.dmsetid = S_EUCLID9;
};


void
GenerateMapNodes::
UserInit(Input * input, Environment *_env)
{
    generators.PutDefaults(_env);
    ValidateParameters(input);
	
    //-----------------------------------------------
    // User Initialize Node Generator
    //-----------------------------------------------
    generators.MakeGNSet("BasicPRM");         // enum BASICPRM
    generators.MakeGNSet("BasicOBPRM");       // enum BASICOBPRM
	
    if ( input->numGNs == 0 ) { //use default GN sets
        generators.MakeGNSet("BasicPRM"); //? why again?
    }
    else {
        gnInfo.gnsetid = GN_USER1;
        for (int i = 0; i < input->numGNs; i++) {
            generators.MakeGNSet(input->GNstrings[i]->GetValue());
        }
    }
	
    gnInfo.numShells = input->numShells.GetValue();
    gnInfo.proportionSurface = input->proportionSurface.GetValue();
    gnInfo.collPair = input->collPair;
    gnInfo.freePair = input->freePair;
    gnInfo.calcClearance = input->calcClearance.GetValue();
	gnInfo.calcPenetration = !(input->calcPenetration.GetValue()==0);
    gnInfo.addNodes2Map = true;
    gnInfo.tag = InfoCfg::NULL_INFO;
};


//===================================================================
// (driver)
// Generate nodes according to all GN's in the set
//
// ie, public method implementations
//
//===================================================================
void
GenerateMapNodes::
GenerateNodes(Roadmap *_rm, CollisionDetection *cd,DistanceMetric *dm,SID _gnsetid, GNInfo &info) {
	
	vector<GN> gnset = generators.GetGNSet(_gnsetid);
	
	// clear generated nodes space
	info.nodes.erase(info.nodes.begin(),info.nodes.end());
	
	for (int gn=0; gn < gnset.size(); ++gn) {
		
#ifndef QUIET
		Clock_Class clock;
		clock.StartClock(gnset[gn].GetName());
		cout<<"\n  "; clock.PrintName(); cout << " " << flush;
#endif
		
		GNF gnfcn = gnset[gn].GetGenerator();
		gnfcn(_rm->environment, cd,dm, gnset[gn], info);
		
#ifndef QUIET
		clock.StopClock();
		cout << clock.GetClock_SEC() << " sec  \n" << flush;
#endif
		
	}
	
	if (info.calcClearance) {
		// go through info.nodes and calculate their clearances...
		for (int i=0; i < info.nodes.size(); i++) {
			info.nodes[i].info.clearance = 
				info.nodes[i].ApproxCSpaceClearance
				(_rm->environment, cd, info.cdsetid, info.cdInfo, dm, 
				 info.dmsetid, info.calcClearance, info.calcPenetration);
		}
	} 
	
	// if that's what the user wants
	if (info.addNodes2Map) {
		
		// tag all nodes generated as user indicates
		//for (int i=0;i<info.nodes.size();++i)
	        //		info.nodes[i].info.tag = info.tag;
		
		
		// then add generated nodes
		_rm->m_pRoadmap->AddVertex(info.nodes); //add node to graph
	}
	
};


//===================================================================
// Basic PRM
//===================================================================
void
GenerateMapNodes::
BasicPRM(Environment *_env, CollisionDetection* cd, DistanceMetric *,GN& _gn, GNInfo &_info){
	
#ifndef QUIET
	cout << "(numNodes=" << _gn.numNodes.GetValue() << ") ";
#endif
	
	
	// PRM style node generation -- generate in expanded bounding box
	vector<Cfg> path;
	//for(int i=0; i< _gn.numNodes.GetValue(); ++i) {
	//  Cfg tmp = Cfg::GetFreeRandomCfg(_env,cd,_info.cdsetid,_info.cdInfo);
	//  path.push_back(tmp);
	//}
	Cfg::GetNFreeRandomCfgs(path, _env,cd,_info.cdsetid,
        				_info.cdInfo, _gn.numNodes.GetValue());
	_info.nodes.insert(_info.nodes.end(), path.begin(), path.end());
	
#if INTERMEDIATE_FILES
	//in util.h
	WritePathConfigurations("prm.path", path, _env);
#endif
};


//===================================================================
// Basic PRM  with Gaussian Filter
//
//   for i = 1 to n
//     randomly generate cfg1
//     randomly generate cfg2 distance of "d" away from cfg1
//     if one of (cfg1,cfg2) is in collision and the other is not
//           add the free one to the roadmap
//     endif
//   endfor
//
//===================================================================
void
GenerateMapNodes::
GaussPRM(
		 Environment *_env, CollisionDetection* cd, DistanceMetric *,
		 GN& _gn, GNInfo &_info){
	
#ifndef QUIET
	cout << "(numNodes=" << _gn.numNodes.GetValue() << ") ";
#endif
	
#if INTERMEDIATE_FILES
	vector<Cfg> path; path.reserve(_gn.numNodes.GetValue());
#endif
	
	if (_gn.gauss_d.GetValue() == 0) {  //if no Gauss_d value given, calculate from env
		_gn.gauss_d.PutValue(_env->Getminmax_BodyAxisRange());
	}
	
	// generate in bounding box
	for (int i=0,newNodes=0; i < _gn.numNodes.GetValue() || newNodes<1 ; i++) {
		
		// cfg1 & cfg2 are generated to be inside bbox
		Cfg cfg1 = Cfg::GetRandomCfg(_env);
		Cfg cfg2 = Cfg::GetRandomCfg(_env);
		cfg2 = Cfg::c1_towards_c2(cfg1,cfg2,_gn.gauss_d.GetValue());
		
		// because cfg2 is modified it must be checked again
		if (cfg2.InBoundingBox(_env)){
			
			bool cfg1_free = !cfg1.isCollision(_env,cd,_info.cdsetid,_info.cdInfo);
			cfg1.info.obst = _info.cdInfo.colliding_obst_index;
			
			bool cfg2_free = !cfg2.isCollision(_env,cd,_info.cdsetid,_info.cdInfo);
			cfg2.info.obst = _info.cdInfo.colliding_obst_index;
			
			
			if (cfg1_free && !cfg2_free) {
				
				_info.nodes.push_back(Cfg(cfg1));   newNodes++;
#if INTERMEDIATE_FILES
				path.push_back(cfg1);
#endif
				
			} else if (!cfg1_free && cfg2_free) {
				
				_info.nodes.push_back(Cfg(cfg2));   newNodes++;
#if INTERMEDIATE_FILES
				path.push_back(cfg2);
#endif
				
			} // endif push nodes
			
		} // endif BB
	} // endfor
	
#if INTERMEDIATE_FILES
	WritePathConfigurations("GaussPRM.path", path, _env);
#endif

};


//===================================================================
// BasicOBPRM
//===================================================================
void
GenerateMapNodes::
BasicOBPRM(Environment *_env, CollisionDetection* cd, DistanceMetric * dm, GN& _gn, GNInfo &_info){
	
#ifndef QUIET
    cout << "(numNodes=" << _gn.numNodes.GetValue() << ", "<<flush;
    cout << "numShells=" << _gn.numShells.GetValue() << ") "<<flush;
#endif
	
#if INTERMEDIATE_FILES
    vector<Cfg> surface;
#endif
	
	_info.numShells = _gn.numShells.GetValue();
	
	vector<Cfg>  preshells, shells, tmp, obstSurface;
	int numMultiBody = _env->GetMultiBodyCount();
	int numExternalBody = _env->GetExternalBodyCount();
	//added by Xinyu Tang
	
	int robot        = _env->GetRobotIndex();
	
	Cfg InsideNode, OutsideNode, low, high, mid;
	

	//Modified by Xinyu Tang 04/01/2002
	int N = _gn.numNodes.GetValue()
		/ (numExternalBody-1)  // -1 for the robot
         / _gn.numShells.GetValue();
	
	if (N < 1) N = max(_gn.numNodes.GetValue(),_gn.numShells.GetValue());
	
	for (int obstacle = 0 ; obstacle < numExternalBody ; obstacle++){
		if (obstacle != robot){  // && obstacle is "Passive" not "Active" robot
			
			for(int n = 0 ; n < N; n++){
				
				// Generate Inside cfg
			  Cfg InsideNode;
			  if(!GenerateInsideCfg(_env, cd, robot, obstacle, &InsideNode, _info)) {
			    cout << "\nError: cannot overlap COMs of robot & obstacle\n";
			    continue;
			  }
			  if(!InsideNode.isCollision(_env,cd,robot,obstacle,_info.cdsetid,_info.cdInfo)){
			    cout << "\nError: Seed not in collision w/"
			      " obstacle[index="<<obstacle<<"]\n" << flush;
			    continue;
			  }
				
				// Generate Random direction
			  Cfg incrCfg=Cfg::GetRandomRay(
							EXPANSION_FACTOR * _env->GetPositionRes()
							);
				
				// Generate outside cfg
			  Cfg OutsideNode = GenerateOutsideCfg(_env,cd,robot,obstacle,
							       InsideNode,incrCfg,_info);
			  if(OutsideNode.AlmostEqual(InsideNode)) continue; // can not find outside node.
				
				// Generate surface cfgs
			  tmp = GenerateSurfaceCfg(_env,cd,dm,_info,
						   robot,obstacle, InsideNode,OutsideNode);
				
				// Choose as many as nshells
			  preshells = Shells(tmp, _gn.numShells.GetValue());
			  shells = InsideBoundingBox(_env, preshells);
			  preshells.erase(preshells.begin(), preshells.end());
				
				// Collect the cfgs for this obstacle
			  obstSurface.insert(obstSurface.end(),
					     shells.begin(),shells.end());
				
			} // endfor: n
			
			// Collect the generated surface nodes
			for (int i=0;i<obstSurface.size();i++){
				obstSurface[i].info.obst = obstacle;
				_info.nodes.push_back(obstSurface[i]);
			}
			
#if INTERMEDIATE_FILES
			surface.insert(surface.end(),
				obstSurface.begin(),obstSurface.end());
#endif
			
			obstSurface.erase   (obstSurface.begin(),obstSurface.end());
			
		} // endif (obstacle != robot)
		else
		  if(numExternalBody == 1) { //if robot is the only object
		    //		  if(numMultiBody == 1) {
		    vector<Cfg> CobstNodes = GenCfgsFromCObst(_env, cd, dm, obstacle, _gn.numNodes.GetValue(), _info);
		    int i;
		    for(i=0; i<CobstNodes.size(); ++i)
		      CobstNodes[i].info.obst = obstacle;
		    _info.nodes.push_back(CobstNodes[i]);
#if INTERMEDIATE_FILES
		    surface.insert(surface.end(),CobstNodes.begin(), CobstNodes.end());
#endif
		  }
			
	} // endfor: obstacle
	
#if INTERMEDIATE_FILES
    WritePathConfigurations("surface.path", surface, _env);
#endif
	
};


//===================================================================
// OBPRM
//===================================================================
void
GenerateMapNodes::
OBPRM(Environment *_env, CollisionDetection *cd ,DistanceMetric * dm,GN& _gn, GNInfo &_info){
	
#ifndef QUIET
	cout << "(numNodes="          << _gn.numNodes.GetValue() << ", ";
	cout << "\tproportionSurface="<< _gn.proportionSurface.GetValue() << ", ";
	cout << "\nnumShells="        << _gn.numShells.GetValue()   << ", ";
	cout << "collPair="           << _gn.collPair.GetValue()    << ", ";
	cout << "freePair="           << _gn.freePair.GetValue()   << ", ";
	cout << "clearanceFactor="    << _gn.clearanceFactor.GetValue()   << ") ";
#endif
    double clearanceFactor;
    clearanceFactor  = _gn.clearanceFactor.GetValue();
    _info.numShells = _gn.numShells.GetValue();
    _info.collPair.PutValue(_gn.collPair.GetValue());
    _info.freePair.PutValue(_gn.freePair.GetValue());
	
    pair<int,int> seedSelect,freeSelect;
	
    ValidatePairs("seedSelect", _gn.collPair, &seedSelect);
    ValidatePairs("freeSelect", _gn.freePair, &freeSelect);
	
    vector<Cfg> tmp, preshells, shells;
	
    int numMultiBody = _env->GetMultiBodyCount();
    int robot        = _env->GetRobotIndex();
    int numExternalBody = _env->GetExternalBodyCount();
    //added by Xinyu Tang
	
    double P = _gn.proportionSurface.GetValue();
	
    // Subtract # of robots (ie, numMultiBody-1)
//      int NSEED = (int)(P * _gn.numNodes.GetValue()/(numMultiBody-1)/_gn.numShells.GetValue());
//      int NFREE = (int)((1.0-P) * _gn.numNodes.GetValue()/(numMultiBody-1));

    // modified by Xinyu Tang 04/01/2002
    // Subtract # of robots (ie, numExternalBody-1)
    int NSEED = (int)(P * _gn.numNodes.GetValue()/(numExternalBody-1)/_gn.numShells.GetValue());
    int NFREE = (int)((1.0-P) * _gn.numNodes.GetValue()/(numExternalBody-1));
	
    if (NSEED < 1) NSEED = 1;
	
    vector<Cfg> obstSurface, obstFree, surface;
    for(int obstacle = 0 ; obstacle < numExternalBody ; obstacle++){
		
        if(obstacle != robot){
			
			// Generate Surface Cfgs using Binary Search Procedure
			obstSurface = GenSurfaceCfgs4Obst(_env,cd,dm, obstacle, NSEED, _info,clearanceFactor);
			
			// Generate Free Cfgs using Ad Hoc procedure
			obstFree = GenFreeCfgs4Obst(_env,cd, obstacle, NFREE, _info);
			
            // Collect free & surface nodes for return
            int i;
            for (i=0;i<obstSurface.size();i++){
				obstSurface[i].info.obst = obstacle;
                _info.nodes.push_back(obstSurface[i]);
			}
            for (    i=0;i<obstFree.size();i++){
				obstFree[i].info.obst = obstacle;
                _info.nodes.push_back(obstFree[i]);
			}
			
#if INTERMEDIATE_FILES
			surface.insert(surface.end(),
                obstSurface.begin(),obstSurface.end());
			surface.insert(surface.end(),
                obstFree.begin(),obstFree.end());
#endif
			
            obstSurface.erase(obstSurface.begin(),obstSurface.end());
            obstFree.erase(obstFree.begin(), obstFree.end());
			
        } // if(obstacle != robot)
		else 
		  //			if(numMultiBody == 1) { //if robot is the only object
			if(numExternalBody == 1) { //if robot is the only object
				vector<Cfg> CobstNodes = GenCfgsFromCObst(_env, cd, dm, obstacle, _gn.numNodes.GetValue(), _info);
				for(int i=0; i<CobstNodes.size(); ++i){
					CobstNodes[i].info.obst = obstacle;
					_info.nodes.push_back(CobstNodes[i]);
				}
#if INTERMEDIATE_FILES
				surface.insert(surface.end(),CobstNodes.begin(), CobstNodes.end());
#endif
			}
			
			
    } // for(obstacle)
	
#if INTERMEDIATE_FILES
	WritePathConfigurations("surface.path", surface, _env);
#endif
};


//===================================================================
// Basic MAPRM
// written by Brent, June 2000  -- FUNCTION IS IN DEVELOPMENT
//
// Do NOT count on this function staying the way it is
//
// Medial Axis version - take 1
// Points NOT in collision and push them to medial axis
// Points in collision are moved out of via random dir, then
//    pushed towards MA.
//
// Originally just works with VCLIP and Cfg_free robots.
// cfg_free robots are in (x, y, z, roll, pitch, yaw) format
//===================================================================
void
GenerateMapNodes::
BasicMAPRM(Environment *_env, CollisionDetection* cd, 
           DistanceMetric *dm,GN& _gn, GNInfo &_info)
{
	CBasicMAPRM::BasicMAPRM(_env,cd,dm,_gn,_info);
} // end BasicMAPRM

//===================================================================
// CSpaceMAPRM
//
// Generates random configurations and pushes them to the medial
// axis of the C-Space. It considers both free nodes and nodes in
// collision.
//===================================================================
void
GenerateMapNodes::
CSpaceMAPRM(Environment *_env, CollisionDetection* cd, DistanceMetric *dm,
          GN& _gn, GNInfo &_info){

   #ifndef QUIET
     cout << "(numNodes=" << _gn.numNodes.GetValue() << ") ";
   #endif

   #if INTERMEDIATE_FILES
     vector<Cfg> path; path.reserve(_gn.numNodes.GetValue());
   #endif

   // MAPRM style node generation using clearances in the CSpace
   for (int i=0; i < _gn.numNodes.GetValue(); i++) {
      Cfg cfg = Cfg::GetMedialAxisCfg(_env,cd,_info.cdsetid,_info.cdInfo,dm,_info.dmsetid,_info.calcClearance);

      if ( !cfg.isCollision(_env,cd,_info.cdsetid,_info.cdInfo) ) {
         _info.nodes.push_back(Cfg(cfg));
         #if INTERMEDIATE_FILES
       path.push_back(cfg);
         #endif
      } else { 
        cout << "cfg in collision!\n" << flush; 
      }
   }

   #if INTERMEDIATE_FILES
   //in util.h
     WritePathConfigurations("csmaprm.path", path, _env);
   #endif
};


//===================================================================
// Validate Parameters
//===================================================================
bool
GenerateMapNodes::
ValidateParameters(Input *_input){
	if ( ValidatePairs(NULL,_input->collPair,NULL) )
		if ( ValidatePairs(NULL,_input->freePair,NULL) )
			return true;
		return false;
}


//===================================================================
// protected method implementations
//===================================================================


//===================================================================
// GenSurfaceCfgs4Obst
//===================================================================
vector<Cfg>
GenerateMapNodes::
GenSurfaceCfgs4Obst(Environment * env,CollisionDetection* cd,DistanceMetric * dm, 
					int obstacle, int nCfgs, GNInfo &info){
	return GenSurfaceCfgs4Obst(env,cd,dm,obstacle,nCfgs,info,1.0);
}

vector<Cfg>
GenerateMapNodes::
GenSurfaceCfgs4Obst(Environment * env,CollisionDetection* cd,DistanceMetric * dm, 
					int obstacle, int nCfgs, GNInfo &info,double clearanceFactor){
	
    pair<int,int> seedSelect;
    ValidatePairs("seedSelect", info.collPair, &seedSelect);
	
    if(seedSelect.first == N_rT && seedSelect.second == N_rT)
        return Cfg::GenSurfaceCfgs4ObstNORMAL(
		env, cd, obstacle, nCfgs, info.cdsetid, info.cdInfo);
    else
        return GenSurfaceCfgs4ObstVERTEX(env, cd, dm, obstacle, nCfgs, info, clearanceFactor);
}


//===================================================================
// GenSurfaceCfgs4ObstVERTEX
//      generate nodes by overlapping two vertices.
//===================================================================
vector<Cfg>
GenerateMapNodes::
GenSurfaceCfgs4ObstVERTEX(Environment * env,CollisionDetection* cd,DistanceMetric * dm, 
						  int obstacle, int nCfgs, GNInfo &info){
	
    return GenSurfaceCfgs4ObstVERTEX(env,cd,dm,obstacle,nCfgs,info,1.0);
}

vector<Cfg>
GenerateMapNodes::
GenSurfaceCfgs4ObstVERTEX(Environment * env,CollisionDetection* cd,DistanceMetric * dm, 
						  int obstacle, int nCfgs, GNInfo &info,double clearanceFactor){
	
    pair<int,int> seedSelect;
    ValidatePairs("seedSelect", info.collPair, &seedSelect);
	
    vector<Cfg> obstSeeds;
    GenerateSeeds(env,cd,info,obstacle,nCfgs,seedSelect.first,seedSelect.second,&obstSeeds);
	
    int robot = env->GetRobotIndex();
    vector<Cfg> tmp, preshells, shells, surface;
    for(int i = 0 ; i < obstSeeds.size() ; i++){
		
		Cfg incrCfg = Cfg::GetRandomRay(EXPANSION_FACTOR*env->GetPositionRes());
		
		Cfg OutsideNode = GenerateOutsideCfg(env,cd,robot,obstacle,obstSeeds[i],incrCfg,info);
		if(OutsideNode.AlmostEqual(obstSeeds[i])) continue; // can not find outside node.
		
		tmp = GenerateSurfaceCfg(env,cd,dm,info,robot,obstacle,obstSeeds[i],OutsideNode,clearanceFactor);
		
        // Choose as many as nshells
        preshells = Shells(tmp, info.numShells);
        shells = InsideBoundingBox(env, preshells);
        preshells.erase(preshells.begin(), preshells.end());
		
        // Collect the cfgs for this obstacle
        surface.insert(surface.end(),shells.begin(),shells.end());
    }
    return surface;
};


//===================================================================
// GenCfgsFromCObst
//      generate nodes by collecting free nodes and emitting rays from nodes in collision.
//===================================================================
vector<Cfg>
GenerateMapNodes::
GenCfgsFromCObst(Environment * env,CollisionDetection* cd,DistanceMetric * dm, 
				 int obstacle, int nCfgs, GNInfo &info){
	
    return GenCfgsFromCObst(env,cd,dm,obstacle,nCfgs,info,1.0);
}

vector<Cfg>
GenerateMapNodes::
GenCfgsFromCObst(Environment * env,CollisionDetection* cd,DistanceMetric * dm, 
				 int obstacle, int nCfgs, GNInfo &info, double clearanceFactor){
	
    int robot = env->GetRobotIndex();
    vector<Cfg> surface, obstSeeds;
    Vector3D voidA, voidB;
    Cfg gen;
    int i;
    for(i=0; i<nCfgs; ++i) {
		///random orientation....?
		Cfg::GenerateOverlapCfg(env, robot, voidA, voidB, &gen);  // voidA, voidB is not used.

		///check collision
		if(gen.isCollision(env,cd, info.cdsetid,info.cdInfo))
			obstSeeds.push_back(gen);
        else
			surface.push_back(gen);
    }
	
    vector<Cfg> tmp, preshells, shells;
    for(i = 0 ; i < obstSeeds.size() ; i++){
		
        Cfg incrCfg = Cfg::GetRandomRay(EXPANSION_FACTOR*env->GetPositionRes());
		
        Cfg OutsideNode = GenerateOutsideCfg(env,cd,robot,obstacle,obstSeeds[i],incrCfg,info);
		if(OutsideNode.AlmostEqual(obstSeeds[i])) continue; // can not find outside node.
		
        tmp = GenerateSurfaceCfg(env,cd,dm,info,robot,obstacle,obstSeeds[i],OutsideNode,clearanceFactor);
		
        // Choose as many as nshells
        preshells = Shells(tmp, info.numShells);
        shells = InsideBoundingBox(env, preshells);
        preshells.erase(preshells.begin(), preshells.end());
		
        // Collect the cfgs for this obstacle
        surface.insert(surface.end(),shells.begin(),shells.end());
    }
    return surface;
};

//===================================================================
// TranslateOptionCode
//===================================================================
int
GenerateMapNodes::
TranslateOptionCode(char *mnemonic, n_str_param param){
	
	//-- Implemented options are:
	
	vector<str_param <char*> >MM; MM.reserve(8);
	MM.push_back(str_param<char*>("cM","(for \"center of mass\")") );
	MM.push_back(str_param<char*>("rV","(for \"random vertex\")") );
	MM.push_back(str_param<char*>("rT","(for \"point in random triangle\")") );
	MM.push_back(str_param<char*>("rE","(for \"random extreme vertex\")") );
	MM.push_back(str_param<char*>("rW","(for \"point in random weighted triangle\")") );
	MM.push_back(str_param<char*>("cM_rV","(for \"cg/random vertex\")") );
	MM.push_back(str_param<char*>("rV_rT","(for \"random vertex/point in random triangle\")") );
	MM.push_back(str_param<char*>("rV_rW","(for \"random vertex/point in random weighted triangle\")") );
	MM.push_back(str_param<char*>("N_rT","(for \"normal of a random triangle\")") );
	MM.push_back(str_param<char*>("all","(for \"all of the basics\")") );
	
	//-- NOT YET Implemented options are:
	
	vector<str_param<char*> >NI; NI.reserve(1);
	NI.push_back(str_param<char*>("other","(for \"other \")") );
	
	const char *DISCLAIMER = "\n  ** Code for mnemonic not yet implemented **";
	
	int i;
	
	if (strlen(mnemonic)>0) {
		
		//-- Exhaustive search of tbl for field values
		for (i=0;i<MM.size();++i)
			if (!strcmp(MM[i].GetFlag(),mnemonic)) return i;
			
			//-- Exhaustive search of "not implemented" tbl for field values
			for (i=0;i<NI.size();++i){
				if (!strcmp(NI[i].GetFlag(),mnemonic)) {
					cout << "\n\nSORRY, mnemonic \""
						<< mnemonic << "\" is not yet implemented ";
					cout << "\n  Implemented mnemonics are:";
					for (i=0;i<MM.size();++i)
						cout << "\n\t" <<setw(5)<< MM[i].GetFlag() << "     " <<MM[i].GetValue();
					// return MM.size()+i;
					return INVALID_OPTION;
				}
			}
	}
	
	//-- if value is not recognized, let'em know it
	
	cout << "\n\nERROR: invalid mnemonic specified \""
        << param.GetFlag() << " " << param.GetValue() << "\"\n";
	cout << "\n  You must specify 2 of the following recognized mnemonics:";
	for (i=0;i<MM.size();++i)
        cout << "\n\t" <<setw(5)<< MM[i].GetFlag() << "     " <<MM[i].GetValue();
	for (i=0;i<NI.size();++i)
        cout << "\n\t" <<setw(5)<< NI[i].GetFlag() << "   **" <<NI[i].GetValue();
	cout << DISCLAIMER;
	cout << "\n\n  If you specify the flag option as \""<<
        param.GetFlag() << " cM rV\","
		"\n\tcM will be used as the option for the robot "
		"\n\tand rV will be used as the option for the obstacle.\n\n";
	return INVALID_OPTION;
	
};


//===================================================================
// ValidatePairs
//===================================================================
bool
GenerateMapNodes::
ValidatePairs(char *msg, n_str_param params, pair<int,int> * results){
	
	int k;
	char *p;
	bool success = false;
	
	//-- separate out the 2 fields
	
    //-- field 1
	
	char str1[20]; strcpy(str1,params.GetValue());
	for (k=0,p=str1; *p != ' ' && p<str1+strlen(str1);++p,++k)
        str1[k]=*p;
	str1[k]='\0';
	
    //-- field 2
	
	char str2[20]; strcpy(str2,params.GetValue());
	for (k=0,p=strchr(str2,' ')+1; *p != ' ' && p<str2+strlen(str2);++p,++k)
        str2[k]=*p;
	str2[k]='\0';
	
	//-- Valid field values will translate to integer options codes
	
	int code1 = TranslateOptionCode(str1,params);
	int code2 = TranslateOptionCode(str2,params);
	if (code1!=INVALID_OPTION && code2!=INVALID_OPTION){
        //-- If storage was provided, store field values
        if (results){
			results->first = code1;
			results->second = code2;
        }
		
        //-- Indicate success
        success = true;
		return true;
	}
	
	//-- Indicate failure
	if (!success){
        cout <<"\nERROR: Bad " << msg <<"pair values\n\n";
        exit(-1);
	}
	return false;
};


//====================================================================
//  GenerateSeeds
//===================================================================
void
GenerateMapNodes::
GenerateSeeds(Environment * env,CollisionDetection *cd, GNInfo &_gnInfo,
              int obst, int nseeds,
              int selectRobot, int selectObstacle,
              vector<Cfg>* seeds)
{
    int rob = env->GetRobotIndex();
	
    vector<Vector3D> ptsRobot, ptsObstacle;
    ptsRobot = PointsOnMultiBody(env->GetMultiBody(rob), nseeds, selectRobot);
	
    ptsObstacle = PointsOnMultiBody(env->GetMultiBody(obst), nseeds, selectObstacle);
	
    Cfg cfg;
    for(int i = 0 ; i < nseeds ; i++){
        if(Cfg::GenerateOverlapCfg(env, rob, ptsRobot[i], ptsObstacle[i], &cfg)){
            // check if it is possible to generate a Cfg with this pose.
            if(cfg.isCollision(env,cd, _gnInfo.cdsetid,_gnInfo.cdInfo)) {
                seeds->push_back(cfg);
            }
        }
    }
    ptsRobot.erase(ptsRobot.begin(),ptsRobot.end());
    ptsObstacle.erase(ptsObstacle.begin(),ptsObstacle.end());
}


//===================================================================
// GenerateFreeCfgs4Obst
//===================================================================
vector<Cfg>
GenerateMapNodes::
GenFreeCfgs4Obst(Environment * env, CollisionDetection *cd, int obstacle, int nCfgs, GNInfo &info){
	
    pair<int,int> freeSelect;
    ValidatePairs("freeSelect", info.freePair, &freeSelect);
	
    int robot = env->GetRobotIndex();
    vector<Vector3D> ptsRobot, ptsObstacle;
	
    ptsRobot = PointsOnMultiBody(env->GetMultiBody(robot), nCfgs, freeSelect.first);
    ptsObstacle = PointsOnMultiBody(env->GetMultiBody(obstacle), nCfgs, freeSelect.second);
	
#if INTERMEDIATE_FILES
	vector<Cfg> att; att.reserve(nCfgs);
#endif
	
    Cfg cfg;
    vector<Cfg> free; free.reserve(nCfgs);
    for(int i = 0 ; i < nCfgs ; i++){
        if( Cfg::GenerateOverlapCfg(env, robot, ptsRobot[i], ptsObstacle[i], &cfg) ) {
			// check if it is possible to generate a Cfg with this pose.
			//if(!cfg.isCollision(env, info.cdsetid,info.cdInfo) && CfgInsideBB(env, cfg)) {
			if(!cfg.isCollision(env,cd, info.cdsetid,info.cdInfo)) {
				free.push_back(cfg);
			}
        }
#if INTERMEDIATE_FILES
		att.push_back(cfg);
#endif
    }
    ptsRobot.erase(ptsRobot.begin(),ptsRobot.end());
    ptsObstacle.erase(ptsObstacle.begin(),ptsObstacle.end());
	
#if INTERMEDIATE_FILES
	WritePathConfigurations("att.path", att, env);
#endif
	
    return free;
};


//===================================================================
// GenerateSurfaceCfg
//===================================================================
vector<Cfg>
GenerateMapNodes::
GenerateSurfaceCfg(Environment *env,CollisionDetection *cd, DistanceMetric * dm,GNInfo& info,
                   int rob, int obst, Cfg insideCfg, Cfg outsideCfg){
	
    return GenerateSurfaceCfg(env,cd,dm,info,rob,obst,insideCfg,outsideCfg,1.0);
	
}

vector<Cfg>
GenerateMapNodes::
GenerateSurfaceCfg(Environment *env,CollisionDetection *cd, DistanceMetric * dm,GNInfo& info,
                   int rob, int obst, Cfg insideCfg, Cfg outsideCfg, double clearanceFactor){
	
    const double PositionRes = env->GetPositionRes();
    vector<Cfg> surface; surface.reserve(MAX_CONVERGE);
    vector<Cfg>     tmp;     tmp.reserve(MAX_CONVERGE);
	
    Cfg low, high, mid;
    double delta;
    int cnt;
	
    low = insideCfg; high = outsideCfg;
    mid = Cfg::WeightedSum(low, high, 0.5);
    delta = dm->Distance(env, low, high, info.dmsetid);
    cnt = 0;
	
    // Do the Binary Search
    tmp.push_back(high);
    while((delta >= clearanceFactor*PositionRes) && (cnt < MAX_CONVERGE)){
        if(mid.isCollision(env,cd , rob, obst, info.cdsetid,info.cdInfo) ) {
            low = mid;
        } else {
            high = mid;
            tmp.push_back(high);
        }
        mid = Cfg::WeightedSum(low, high, 0.5);
        delta = dm->Distance(env, low, high, info.dmsetid);
        cnt++;
    }
	
    // if converged save the cfgs that don't collide with the environment
    if(cnt < MAX_CONVERGE) {
        if(!high.isCollision(env,cd, info.cdsetid,info.cdInfo)) {
            surface = FirstFreeCfgs(env, cd,tmp, info);
        }
    }
    return surface;
}


//===================================================================
// Spread
//===================================================================
void
GenerateMapNodes::
Spread(Cfg pivot, double tStep, double rStep, int nspread,
       vector<Cfg>* spread){
	
    for(int j = 0 ; j < nspread ; j++){
		spread->push_back(pivot + Cfg::GetRandomCfg(tStep, rStep));
    }
}


//===================================================================
// FirstFreeCfgs
//===================================================================
vector<Cfg>
GenerateMapNodes::
FirstFreeCfgs(Environment *env,CollisionDetection *cd, vector<Cfg> cfgs, GNInfo &info, int n){
	
    int size = cfgs.size();
    n = min(n,size);
	
    vector<Cfg> free;
    free.reserve(size);
    int i = 0; int cnt = 0;
    for (i = 0, cnt = 0; i < size && cnt < n; i++){
		if(!cfgs[i].isCollision(env,cd, info.cdsetid,info.cdInfo)){
            free.push_back(cfgs[i]);
            cnt++;
        }
    }
    return free;
}


//===================================================================
// FirstFreeCfgs
//===================================================================
vector<Cfg>
GenerateMapNodes::
FirstFreeCfgs(Environment *env,CollisionDetection *cd, vector<Cfg> cfgs, GNInfo &info){
    int size = cfgs.size();
    return FirstFreeCfgs(env, cd, cfgs, info, size);
}


//===================================================================
// InsideBoundingBox
//===================================================================
vector<Cfg>
GenerateMapNodes::
InsideBoundingBox(Environment *env, vector<Cfg> cfgs){
	
    vector<Cfg> ncfgs;
	
    for(int i = 0 ; i < cfgs.size() ; i++)
		if(cfgs[i].InBoundingBox(env))
            ncfgs.push_back(cfgs[i]);
		return ncfgs;
};


//===================================================================
// ChooseRandomVertexOnBody
//===================================================================
Vector3D
GenerateMapNodes::
ChooseRandomVertexOnBody(Body *body, bool isFreeBody)
{
    GMSPolyhedron polyhedron;
	
    // for robot, choose body frame; for obstacle, choose world frame
    if(isFreeBody) polyhedron = body->GetPolyhedron();
    else           polyhedron = body->GetWorldPolyhedron();
	
    // We choose a vertex of the part at random
    return polyhedron.vertexList[(int)(drand48()*polyhedron.numVertices)];
};


//===================================================================
// ExtremeVertex
//===================================================================
Vector3D
GenerateMapNodes::
ExtremeVertex(Body * body, bool isFreeBody)
{
    GMSPolyhedron polyhedron;
    // for robot, choose body frame; for obstacle, choose world frame
    if(isFreeBody) polyhedron = body->GetPolyhedron();
    else           polyhedron = body->GetWorldPolyhedron();
	
    int indexVert[6];
    for(int j = 0 ; j < 6 ; j++){
        indexVert[j] = 0;
    }
	
    for(int i = 1 ; i < polyhedron.numVertices ; i++){
		
		//MAX X
        if(polyhedron.vertexList[i][0] < polyhedron.vertexList[indexVert[0]][0])
            indexVert[0] = i;

		//MIN X
        if(polyhedron.vertexList[i][0] > polyhedron.vertexList[indexVert[1]][0])
            indexVert[1] = i;
		
		//MAX Y
        if(polyhedron.vertexList[i][1] < polyhedron.vertexList[indexVert[2]][1])
            indexVert[2] = i;
		
		//MIN Y
        if(polyhedron.vertexList[i][1] > polyhedron.vertexList[indexVert[3]][1])
            indexVert[3] = i;
		
		//MAX Z
        if(polyhedron.vertexList[i][2] < polyhedron.vertexList[indexVert[4]][2])
            indexVert[4] = i;
		
		//<MIN Z
        if(polyhedron.vertexList[i][2] > polyhedron.vertexList[indexVert[5]][2])
            indexVert[5] = i;
    }
	
    // Choose an extreme random vertex at random
    int index = rand() % 6;
    return polyhedron.vertexList[index];
};


//===================================================================
// ChoosePointOnTriangle
//===================================================================
Vector3D
GenerateMapNodes::
ChoosePointOnTriangle(Vector3D p, Vector3D q, Vector3D r){
	
    Vector3D u, v;
    u = q - p;
    v = r - p;
	
    double s = drand48(); double t = drand48();
    while(s + t > 1){
        t = drand48();
    }
    return (p + u*s + v*t);
};


//===================================================================
// ChooseRandomTriangleOnBody
//===================================================================
Vector3D
GenerateMapNodes::
ChooseRandomTriangleOnBody(Body *body, bool isFreeBody)
{
    GMSPolyhedron polyhedron;
    // for robot, choose body frame; for obstacle, choose world frame
    if(isFreeBody)
		polyhedron = body->GetPolyhedron();
    else
		polyhedron = body->GetWorldPolyhedron();
	
    // We choose a triangle of the body at random
	
    GMSPolygon *poly = &polyhedron.polygonList[(int)(drand48()*polyhedron.numPolygons)];
    //GMSPolygon p2=poly;
    Vector3D p, q, r;
    p = polyhedron.vertexList[poly->vertexList[0]];
    q = polyhedron.vertexList[poly->vertexList[1]];
    r = polyhedron.vertexList[poly->vertexList[2]];
	
    Vector3D u;
    u = ChoosePointOnTriangle(p, q, r);
    return u;
};


//===================================================================
//  ChooseRandomWeightedTriangleOnBody
//===================================================================
Vector3D
GenerateMapNodes::
ChooseRandomWeightedTriangleOnBody(Body *body, bool isFreeBody)
{
    GMSPolyhedron polyhedron;
    // for robot, choose body frame; for obstacle, choose world frame
    if(isFreeBody){
        polyhedron = body->GetPolyhedron();
    }
    else{
        polyhedron = body->GetWorldPolyhedron();
    }
	
    double area;
    area = body->GetPolyhedron().area;
	
    double targetArea = area * drand48();
	
    int index, i;
    double sum;
    index = 0; i = 1; sum = body->GetPolyhedron().polygonList[0].area;
    while(targetArea > sum){
        sum += body->GetPolyhedron().polygonList[i].area;
        index++;
        i++;
    }
	
    // We choose the triangle of the body with that index
    GMSPolygon *poly = &polyhedron.polygonList[index];
	
    // We choose a random point in that triangle
    Vector3D p, q, r;
    p = polyhedron.vertexList[poly->vertexList[0]];
    q = polyhedron.vertexList[poly->vertexList[1]];
    r = polyhedron.vertexList[poly->vertexList[2]];
	
    Vector3D u;
    u = ChoosePointOnTriangle(p, q, r);
    return u;
};


//===================================================================
// PointOnBody
//===================================================================
Vector3D
GenerateMapNodes::
PointOnBody(Body * body, int select, bool isFreeBody)
{
    Vector3D pt;
    int opt;
	
    switch( (PairOptions)select ){
	case cM:
		pt = body->GetCenterOfMass();
		break;
		
	case rV:
		pt = ChooseRandomVertexOnBody(body, isFreeBody);
		break;
		
	case rT:
		pt = ChooseRandomTriangleOnBody(body, isFreeBody);
		break;
		
	case rE:
		pt = ExtremeVertex(body, isFreeBody);
		break;
		
    case rW:
		pt = ChooseRandomWeightedTriangleOnBody(body, isFreeBody);
		break;
		
    case cM_rV:
        opt = rand() % 2;
        if(opt == 0){
            pt = body->GetCenterOfMass();
        }
        else{
			pt = ChooseRandomVertexOnBody(body, isFreeBody);
        }
		break;
		
    case rV_rT:
        opt = rand() % 2;
        if(opt == 0){
			pt = ChooseRandomVertexOnBody(body, isFreeBody);
		}
        else{
			pt = ChooseRandomTriangleOnBody(body, isFreeBody);
        }
        break;
		
    case rV_rW:
        opt = rand() % 2;
		if(opt == 0){
			pt = ChooseRandomVertexOnBody(body, isFreeBody);
		}
		else{
			pt = ChooseRandomWeightedTriangleOnBody(body, isFreeBody);
        }
        break;
		
    case all:
        opt = rand() % 5;
		if(opt == 0){
			pt = body->GetCenterOfMass();
		}
		else if(opt == 1){
			pt = ChooseRandomVertexOnBody(body, isFreeBody);
		}
		else if(opt == 2){
			pt = ChooseRandomTriangleOnBody(body, isFreeBody);
		}
		else if(opt == 3){
			pt = ExtremeVertex(body, isFreeBody);
        }
        else{
			pt = ChooseRandomWeightedTriangleOnBody(body, isFreeBody);
		}
        break;
		
	default:
		cout << "\n Unknown Option for PointOnBody \n";
		exit(0);
		break;
    }
    return pt;
}


//===================================================================
// PointsOnMultiBody
//===================================================================
vector<Vector3D>
GenerateMapNodes::
PointsOnMultiBody(MultiBody * mbody, int npts, int select)
{
    int nFree = mbody->GetFreeBodyCount();
	
    vector<Vector3D> pts;
    pts.reserve(npts);
	
    // for robot(which have freebodys, sample points on last link only.
    // for obstacle, sample first link. (in fact, only ONE link?! )
    if(nFree)
        for(int j = 0 ; j < npts ; j++){
            pts.push_back(PointOnBody(mbody->GetFreeBody(nFree-1),select,1));
        }
		else
			for(int j = 0 ; j < npts ; j++){
				pts.push_back(PointOnBody(mbody->GetFixedBody(0), select, 0));
			}
			
			return pts;
}


//===================================================================
// Shells
//===================================================================
vector<Cfg>
GenerateMapNodes::
Shells(vector<Cfg> cfgs, int nshells){
	
    int size = cfgs.size();
    int limit = min(nshells, size);
    vector<Cfg> shells;
    shells.reserve(nshells);
    if(limit > 0){
        int step = size/limit;
        for(int i = 0 ; i < limit ; i++){
            int k = (cfgs.size() - 1) - (i*step);
            shells.push_back(cfgs[k]);
        }
    }
    return shells;
}


//===================================================================
// GenerateInsideCfg
//===================================================================
bool
GenerateMapNodes::
GenerateInsideCfg(Environment *_env, CollisionDetection* _cd,
				  int rob, int obst, Cfg *insideNode, GNInfo &_info){
	
    bool tmp = Cfg::GenerateOverlapCfg(_env, rob,
		_env->GetMultiBody(rob)->GetCenterOfMass(),
		_env->GetMultiBody(obst)->GetCenterOfMass(),
		insideNode);
	
    // check the cfg obtained by center of mass overlapping if valid
    if (!insideNode->isCollision(_env, _cd, rob, obst, _info.cdsetid,
		_info.cdInfo)) {
		
		// if center of mass does not work in getting the cfg in collision,
		// use random vertex of an obstacle (J Kim)
		Vector3D vP;
		
		// code copied from GenerateMapNodes::PointsOnMultiBody()
		vP = PointOnBody(_env->GetMultiBody(obst)->GetFixedBody(0), rV, false);
		
		// get inside cfg again by using vP instead of center of mass
		bool tmp = Cfg::GenerateOverlapCfg(_env, rob,
			_env->GetMultiBody(rob)->GetCenterOfMass(),
			vP,
			insideNode);
		tmp = true;
    }
    return tmp;
}


//===================================================================
// GenerateOutsideCfg
//===================================================================
Cfg
GenerateMapNodes::
GenerateOutsideCfg(Environment *env,CollisionDetection * cd, int rob, int obst,
                   Cfg InsideNode, Cfg incrCfg, GNInfo &info){
	
    int count = 0;
    Cfg OutsideNode = InsideNode + incrCfg;
    while(OutsideNode.isCollision(env,cd, rob, obst, info.cdsetid,info.cdInfo) ) {
        OutsideNode = OutsideNode + incrCfg;
		if(count++ > 500)
			return InsideNode;
    }
    return OutsideNode;
}



/////////////////////////////////////////////////////////////////////
//
//  METHODS for class GN
//
/////////////////////////////////////////////////////////////////////

GN::
GN():

//                                default  MIN  MAX
numNodes         ("nodes",            10,  1,   5000000),
numShells        ("shells",            3,  1,   50),
proportionSurface("pctSurf",         1.0,  0,   1.0),
collPair         ("collPair","cM rT "),
freePair         ("freePair","cM rV "),
clearanceFactor  ("clearFact",       1.0,  0,   1.0),
gauss_d          ("d",                 0,  0,   5000000)

{
	
    numNodes.PutDesc         ("INTEGER","");
    numShells.PutDesc        ("INTEGER","");
    proportionSurface.PutDesc("FLOAT  ","");
    collPair.PutDesc         ("STRING STRING",
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
    freePair.PutDesc         ("STRING STRING","\n\t\t\tSame as above"
        );
	
    // limit the number of strings "grabbed" to 2
	collPair.PutNumStrings(2);
	freePair.PutNumStrings(2);
	
	
    clearanceFactor.PutDesc  ("FLOAT  ","");
    gauss_d.PutDesc          ("FLOAT  ","");
	
	
	strcpy(name,"");
	generator = 0;
	gnid = INVALID_EID;
};

GN::
~GN() {
};

void
GN::
PrintUsage_All(ostream& _os){
	
    PrintUsage_BasicPRM(_os);
    PrintUsage_BasicOBPRM(_os);
    PrintUsage_OBPRM(_os);
    PrintUsage_GaussPRM(_os);
    PrintUsage_BasicMAPRM(_os);
    PrintUsage_CSpaceMAPRM(_os);
	
}

void
GN::
PrintUsage_BasicPRM(ostream& _os){
	
    cout.setf(ios::left,ios::adjustfield);
	
	_os << "\nBasicPRM ";
	_os << "\n\t"; numNodes.PrintUsage(_os);
	
    cout.setf(ios::right,ios::adjustfield);
	
};
void
GN::
PrintUsage_BasicOBPRM(ostream& _os){
	
    cout.setf(ios::left,ios::adjustfield);
	
	_os << "\nBasicOBPRM ";
	_os << "\n\t"; numNodes.PrintUsage(_os);
	_os << "\n\t"; numShells.PrintUsage(_os);
	
    cout.setf(ios::right,ios::adjustfield);
	
};

void
GN::
PrintUsage_OBPRM(ostream& _os){
	
    cout.setf(ios::left,ios::adjustfield);
	
	_os << "\nOBPRM ";
	_os << "\n\t"; numNodes.PrintUsage(_os);
	_os << "\n\t"; numShells.PrintUsage(_os);
	_os << "\n\t"; proportionSurface.PrintUsage(_os);
	_os << "\n\t"; numShells.PrintUsage(_os);
	_os << "\n\t"; collPair.PrintUsage(_os);
	_os << "\n\t"; freePair.PrintUsage(_os);
	_os << "\n\t"; clearanceFactor.PrintUsage(_os);
	
    cout.setf(ios::right,ios::adjustfield);
	
};

void
GN::
PrintUsage_GaussPRM(ostream& _os){
	
    cout.setf(ios::left,ios::adjustfield);
	
	_os << "\nGaussPRM ";
	_os << "\n\t"; numNodes.PrintUsage(_os);
	_os << "\n\t"; gauss_d.PrintUsage(_os);
	
    cout.setf(ios::right,ios::adjustfield);
	
};
void
GN::
PrintUsage_BasicMAPRM(ostream& _os){
	
    cout.setf(ios::left,ios::adjustfield);
	
	_os << "\nBasicMAPRM ";
	_os << "\n\t"; numNodes.PrintUsage(_os);
	
    cout.setf(ios::right,ios::adjustfield);
	
};
void
GN::
PrintUsage_CSpaceMAPRM(ostream& _os){

    cout.setf(ios::left,ios::adjustfield);

        _os << "\nCSpaceMAPRM ";
        _os << "\n\t"; numNodes.PrintUsage(_os);

    cout.setf(ios::right,ios::adjustfield);

};

bool
GN::
operator==(const GN& _gn) const
{
	if ( strcmp(name,_gn.name) != 0 ) {
		return false;
	} else if ( !strcmp(name,"BasicPRM") ) {
		return true;
		
		//return ( numNodes.GetValue() == _gn.numNodes.GetValue );
		
	} else if ( !strcmp(name,"BasicOBPRM") ) {
		return true;
		
		//return ( (numNodes.GetValue() == _gn.numNodes.GetValue()) &&
		//         (numShells.GetValue() == _gn.numShells.GetValue()) );           
		
	} else if ( !strcmp(name,"OBPRM") ) {
		return true;
		
		//return ( (numNodes.GetValue() == _gn.numNodes.GetValue()) &&
		//       (numShells.GetValue() == _gn.numShells.GetValue()) &&
		//       (proportionSurface.GetValue() == _gn.proportionSurface.GetValue()) &&
		//       (collPair.GetValue() == _gn.collPair.GetValue()) &&
		//       (freePair.GetValue() == _gn.freePair.GetValue()) &&
		//       (clearanceFactor.GetValue() == _gn.clearanceFactor.GetValue()) );
		
	} else if ( !strcmp(name,"GaussPRM") ) {
		return true;
		
		//return ( (numNodes.GetValue() == _gn.numNodes.GetValue()) &&
		//         (gauss_d.GetValue() == _gn.gauss_d.GetValue()) );
		
	} else if ( !strcmp(name,"BasicMAPRM") ) {
		return true;
		
		//return ( numNodes.GetValue() == _gn.numNodes.GetValue() );

        } else if ( !strcmp(name,"CSpaceMAPRM") ) {
                return true;

                //return ( numNodes.GetValue() == _gn.numNodes.GetValue() );
		
	} else { // unrecognized...
		return false;
	}
};


char*
GN::
GetName() const {
	return const_cast<char*>(name);
};

GNF
GN::
GetGenerator(){
	return generator;
};


EID
GN::
GetID() const {
	return gnid;
};

ostream& operator<< (ostream& _os, const GN& gn) {
    _os<< gn.GetName();
	
    /*
    if ( !strstr(gn.GetName(),"BasicPRM") ){
	_os<< ", numNodes = " << gn.numNodes.GetValue();
    }
    if ( !strstr(gn.GetName(),"GaussPRM") ){
	_os<< ", numNodes = " << gn.numNodes.GetValue();
	_os<< ", d = " << gn.gauss_d.GetValue();
    }
    if ( !strstr(gn.GetName(),"BasicOBPRM") ){
	_os<< ", numNodes = " << gn.numNodes.GetValue();
	_os<< ", numShells = " << gn.numShells.GetValue();
    }
    if ( !strstr(gn.GetName(),"OBPRM") ){
	_os<< ", numNodes = " << gn.numNodes.GetValue();
	_os<< ", numShells = " << gn.numShells.GetValue();
	_os<< ", proportionSurface = " << gn.proportionSurface.GetValue();
	_os<< ", collPair = " << gn.collPair.GetValue();
	_os<< ", freePair = " << gn.freePair.GetValue();
	_os<< ", clearanceFactor = " << gn.clearanceFactor.GetValue();
    }
    if ( !strstr(gn.GetName(),"BasicMAPRM") ){
	_os<< ", numNodes = " << gn.numNodes.GetValue();
    }
    if ( !strstr(gn.GetName(),"CSpaceMAPRM") ){
        _os<< ", numNodes = " << gn.numNodes.GetValue();
    }
    */
	
    return _os;
};



/////////////////////////////////////////////////////////////////////
//
//  METHODS for class GNSets
//
/////////////////////////////////////////////////////////////////////

//==================================
// GNSets class Methods: Constructors and Destructor
//==================================
// GNSets();
// ~GNSets();

GNSets::
GNSets(){
};

GNSets::
~GNSets(){
};

//===================================================================
// GNSets class Methods: Adding GNs, Making & Modifying GN sets
//===================================================================
// EID AddGN(const char* _gninfo);
// int AddGNToSet(const SID _sid, const EID _gnid);
// int DeleteGNFromSet(const SID _sid, const EID _gnid);
// SID MakeGNSet(const char* gnlist);  // make an ordered set of gns,
// SID MakeGNSet(istream& _myistream); //  - add gn to universe if not there
// SID MakeGNSet(const EID _eid);
// SID MakeGNSet(const vector<EID> _eidvector);
// int DeleteGNSet(const SID _sid);

void
GNSets::
PutDefaults(Environment *_env) {
};

int
GNSets::
AddGN(const char* _gninfo) {
	SID sid = MakeGNSet(_gninfo);
	SetIDs--;
	return DeleteOSet(sid);        // delete the set, but not elements
};


int
GNSets::
AddGNToSet(const SID _sid, const EID _gnid) {
	return AddElementToOSet(_sid,_gnid);
};

int
GNSets::
DeleteGNFromSet(const SID _sid, const EID _gnid) {
	return DeleteElementFromOSet(_sid,_gnid);
};

SID
GNSets::
MakeGNSet(const char* _gnlist){
	
#ifdef _WIN32
	istrstream  is((char*) _gnlist);
#else
	istrstream  is(_gnlist);
#endif
	if (!is) {
		cout << endl << "In MakeGNSet: can't open instring: " << _gnlist ;
		return INVALID_SID;
	}
	return MakeGNSet(is);
};

SID
GNSets::
MakeGNSet(const EID _eid) {
	return MakeOSet(_eid);
}

SID
GNSets::
MakeGNSet(const vector<EID> _eidvector) {
	return MakeOSet(_eidvector);
}

int
GNSets::
DeleteGNSet(const SID _sid) {
	return DeleteOSet(_sid);
}

/** GetFieldRange is a macro used only by MakeGNSet.
* It returns the beginning and ending indices of the 
* current Generation Method only if the method is valid. 
*
* Tets if this cmdFields starts with BasicPRM, GaussPRM
* , BasicOBPRM, OBPRM, or BasicMAPRM. If not,
* i++. start is start of field range and stop is the 
* end of field range...
*/
#define GetFieldRange()                              \
	int start=i+1;                                       \
	while ( (strcmp(cmdFields[i+1],"BasicPRM")           \
	&& strcmp(cmdFields[i+1],"GaussPRM")                 \
	&& strcmp(cmdFields[i+1],"BasicOBPRM")               \
	&& strcmp(cmdFields[i+1],"OBPRM")                    \
	&& strcmp(cmdFields[i+1],"BasicMAPRM")               \
        && strcmp(cmdFields[i+1],"CSpaceMAPRM") )&&i+1<argc  \
	){i++;}                                              \
	int stop=i+1;                                        \
	
SID
GNSets::
MakeGNSet(istream& _myistream) {

	vector<EID> gnvec;  // vector of gnids for this set
	
	// BEGIN - Ugly
	//lkd the follow lines before the "for" stmt are ugly
	// and disgusting as are all the cast's put on the 
	// parameters sent to AckCmdLine.  But it all works! (-:
	int argc=0;
	char *argv[50];
	char cmdFields[50][100]; 
	while ( _myistream >> cmdFields[argc] ){
		argv[argc]= (char *)(&cmdFields[argc]); ++argc;
	};
	// END  - Ugly
	
	for (int i=0;i<argc; ++i) {
		
		GN gn1;
		strcpy(gn1.name,cmdFields[i]);
		
		if (!strcmp(cmdFields[i],"BasicPRM")) {
			gn1.generator = &GenerateMapNodes::BasicPRM;
			GetFieldRange();
			
			for (int j=start;j<stop; ++j) {
				if(gn1.numNodes.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else {
					cout << "\nERROR MakeGNSet: Don\'t understand \""
						<< cmdFields[j]<<"\"\n\n";
					gn1.PrintUsage_BasicPRM(cout);
					cout << endl;
					exit (-1);
				} //endif
			} //endfor j
		} else if (!strcmp(cmdFields[i],"GaussPRM")) {
			gn1.generator = &GenerateMapNodes::GaussPRM;
			GetFieldRange();
			
			for (int j=start;j<stop; ++j) {
				if(gn1.numNodes.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else if (gn1.gauss_d.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else {
					cout << "\nERROR MakeGNSet: Don\'t understand \""
						<< cmdFields[j]<<"\"\n\n";
					gn1.PrintUsage_GaussPRM(cout);
					cout << endl;
					exit (-1);
				} //endif
			} //endfor j
			
		} else if (!strcmp(cmdFields[i],"BasicOBPRM")) {
			gn1.generator = &GenerateMapNodes::BasicOBPRM;
			GetFieldRange();
			
			for (int j=start;j<stop; ++j) {
				if        (gn1.numNodes.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else if (gn1.numShells.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else {
					cout << "\nERROR MakeGNSet: Don\'t understand \""
						<< cmdFields[j]<<"\"\n\n";
					gn1.PrintUsage_BasicOBPRM(cout);
					cout << endl;
					exit (-1);
				} //endif
			} //endfor j
			
			
		} else if (!strcmp(cmdFields[i],"OBPRM")) {
			gn1.generator = &GenerateMapNodes::OBPRM;
			GetFieldRange();
			
			for (int j=start;j<stop; ++j) {
				if (gn1.numNodes.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else if (gn1.numShells.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else if (gn1.proportionSurface.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else if (gn1.freePair.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else if (gn1.collPair.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else if (gn1.clearanceFactor.AckCmdLine(&j,(int)stop,(char**)(&argv))){
				} else {
					cout << "\nERROR MakeGNSet: Don\'t understand \""
						<< cmdFields[j]<<"\"\n\n";
					gn1.PrintUsage_OBPRM(cout);
					cout << endl;
					exit (-1);
				} //endif
			} //endfor j
		} else if (!strcmp(cmdFields[i],"BasicMAPRM")) {
			gn1.generator = &GenerateMapNodes::BasicMAPRM;
			GetFieldRange();
			if( CBasicMAPRM::ReadParameters(start,stop,(char**)(&argv),gn1.numNodes)==false ){
				cout << "\nERROR MakeGNSet: Don\'t understand \""
				<< cmdFields[start]<<"\"\n\n";
				gn1.PrintUsage_BasicMAPRM(cout);
				cout << endl;
				exit (-1);
			}//end if
                } else if (!strcmp(cmdFields[i],"CSpaceMAPRM")) {
                        gn1.generator = &GenerateMapNodes::CSpaceMAPRM;
                        GetFieldRange();

                        for (int j=start;j<stop; ++j) {
                                if        (gn1.numNodes.AckCmdLine(&j,(int)stop,(char**)(&argv))){
                                } else {
	                                cout << "\nERROR MakeGNSet: Don\'t understand \""
                                                << cmdFields[j]<<"\"\n\n";
                                        gn1.PrintUsage_CSpaceMAPRM(cout);
                                        cout << endl;
                                        exit (-1);
                                } //endif
                        } //endfor j

		} else {
			cout << "\n\nERROR MakeGNSet: Don\'t understand \""
				<< cmdFields[i]<<"\"\n";
			gn1.PrintUsage_All(cout);
			cout << endl;
			exit (-1);
		}//endif
		
		
		gn1.gnid = AddElementToUniverse(gn1);
		if ( ChangeElementInfo(gn1.gnid,gn1) != OK ) {
			cout << endl << "\nIn MakeSet: couldn't change element info";
			exit(-1);
		}
		gnvec.push_back( gn1.gnid );
		
		
  }//endfor
  
  return MakeOSet(gnvec);
};

//===================================================================
// GNSets class Methods: Getting Data & Statistics
//===================================================================
// GN GetGN(const EID _gnid) const;
// vector<GN> GetGNs() const;
// vector<GN> GetGNSet(const SID _sid) const;
// vector<pair<SID,vector<GN> > > GetGNSets() const;

GN
GNSets::
GetGN(const EID _gnid) const {
	return GetElement(_gnid);
};

vector<GN>
GNSets::
GetGNs() const {
	vector<GN> elts2;
	vector<pair<EID,GN> > elts1 = GetElements();
	for (int i=0; i < elts1.size(); i++)
		elts2.push_back( elts1[i].second );
	return elts2;
};

vector<GN>
GNSets::
GetGNSet(const SID _sid) const {
	vector<GN> elts2;
	vector<pair<EID,GN> > elts1 = GetOSet(_sid);
	for (int i=0; i < elts1.size(); i++)
		elts2.push_back( elts1[i].second );
	return elts2;
};


vector<pair<SID,vector<GN> > >
GNSets::
GetGNSets() const {
	
	vector<pair<SID,vector<GN> > > s2;
	vector<GN> thesegns;
	
	vector<pair<SID,vector<pair<EID,GN> > > > s1 = GetOSets();
	
	for (int i=0; i < s1.size(); i++)  {
		thesegns.erase(thesegns.begin(),thesegns.end());
		for (int j=0; j < s1[i].second.size(); j++ )
			thesegns.push_back (s1[i].second[j].second);
		s2.push_back( pair<SID,vector<GN> > (s1[i].first,thesegns) );
	}
	return s2;
};


void
GNSets::
DisplayGNs() const{
	DisplayElements();
};

void
GNSets::
DisplayGN(const EID _gnid) const{
	DisplayElement(_gnid);
};

void
GNSets::
DisplayGNSets() const{
	DisplayOSets();
};

void
GNSets::
DisplayGNSet(const SID _sid) const{
	DisplayOSet(_sid);
};

void
GNSets::
WriteGNs(const char* _fname) const {
	
	ofstream  myofstream(_fname);
	if (!myofstream) {
		cout << endl << "In WriteGNS: can't open outfile: " << _fname ;
	}
	WriteGNs(myofstream);
	myofstream.close();
};

void
GNSets::
WriteGNs(ostream& _myostream) const {
	
	vector<GN> gns = GetGNs();
	
	_myostream << endl << "#####GNSTART#####";
	_myostream << endl << gns.size();  // number of gns
	
	//format: GN_NAME (a string) GN_PARMS (double, int, etc)
	for (int i = 0; i < gns.size() ; i++) {
		_myostream << endl;
		_myostream << gns[i].name << " ";
		if ( !strcmp(gns[i].name,"GaussPRM") ) {
			_myostream << gns[i].gauss_d.GetValue();
		}
	}
	_myostream << endl << "#####GNSTOP#####";
};

void
GNSets::
ReadGNs(const char* _fname) {
	
	ifstream  myifstream(_fname);
	if (!myifstream) {
		cout << endl << "In ReadGNs: can't open infile: " << _fname ;
		return;
	}
	ReadGNs(myifstream);
	myifstream.close();
};

void
GNSets::
ReadGNs(istream& _myistream) {
	
	char tagstring[100];
	char gndesc[100];
	int  numGNs;
	
	_myistream >> tagstring;
	if ( !strstr(tagstring,"GNSTART") ) {
		cout << endl << "In ReadGNs: didn't read GNSTART tag right";
		return;
	}
	
	_myistream >> numGNs;
	_myistream.getline(gndesc,100,'\n');  // throw out rest of this line
	for (int i = 0; i < numGNs; i++) {
        _myistream.getline(gndesc,100,'\n');
        AddGN(gndesc);
	}
	
	_myistream >> tagstring;
	if ( !strstr(tagstring,"GNSTOP") ) {
		cout << endl << "In ReadGNs: didn't read GNSTOP tag right";
		return;
	}
};




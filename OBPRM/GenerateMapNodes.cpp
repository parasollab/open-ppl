// $Id$
/////////////////////////////////////////////////////////////////////
//
//  GenerateMapNodes.c
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

#include "GenerateMapNodes.h"

#include "Roadmap.h"
#include "Clock_Class.h"
#include <vector.h>

#define EXPANSION_FACTOR 100

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
#ifdef USE_CSTK
    gnInfo.cdsetid = CSTK;
#else
    gnInfo.cdsetid = RAPID;
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
        generators.MakeGNSet("BasicPRM");
    }
    else {
        gnInfo.gnsetid = GN_USER1;
        for (int i = 0; i < input->numGNs; i++) {
            generators.MakeGNSet(input->GNstrings[i]->GetValue());
        }
    }

    gnInfo.numNodes = input->numNodes.GetValue();
    gnInfo.numNodesPerObst = input->numNodesPerObst.GetValue();
    gnInfo.numShells = input->numShells.GetValue();
    gnInfo.proportionSurface = input->proportionSurface.GetValue();
    gnInfo.collPair = input->collPair;
    gnInfo.freePair = input->freePair;

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

  // add generated nodes
  _rm->roadmap.AddVertex(info.nodes);

};


//===================================================================
// Basic PRM
//===================================================================
void
GenerateMapNodes::
BasicPRM(Environment *_env, CollisionDetection* cd, DistanceMetric *,GN& _gn, GNInfo &_info){

   #ifndef QUIET
     cout << "(numNodes=" << _info.numNodes << ") ";
   #endif

   #if INTERMEDIATE_FILES
     vector<Cfg> path; path.reserve(_info.numNodes);
   #endif

   // PRM style node generation -- generate in expanded bounding box
   for (int i=0; i < _info.numNodes; i++) {
      Cfg cfg = Cfg::GetRandomCfg(_env);

      if ( !cfg.isCollision(_env,cd,_info.cdsetid) ) {
         _info.nodes.push_back(Cfg(cfg));
         #if INTERMEDIATE_FILES
	   path.push_back(cfg);
         #endif
      }
   }

   #if INTERMEDIATE_FILES
     WritePathTranformationMatrices("prm.path", path, _env);
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
     cout << "(numNodes=" << _info.numNodes << ") ";
   #endif

   #if INTERMEDIATE_FILES
     vector<Cfg> path; path.reserve(_info.numNodes);
   #endif

   // generate in bounding box
   for (int i=0; i < _info.numNodes; i++) {

      // cfg1 & cfg2 are generated to be inside bbox
      Cfg cfg1 = Cfg::GetRandomCfg(_env);
      Cfg cfg2 = Cfg::GetRandomCfg(_env);
      cfg2 = Cfg::c1_towards_c2(cfg1,cfg2,_gn.Get_Gauss_d());

      // because cfg2 is modified it must be checked again
      if (cfg2.InBoundingBox(_env)){

        bool cfg1_free = !cfg1.isCollision(_env,cd,_info.cdsetid);
        bool cfg2_free = !cfg2.isCollision(_env,cd,_info.cdsetid);

        if (cfg1_free && !cfg2_free) {
         _info.nodes.push_back(Cfg(cfg1));

         #if INTERMEDIATE_FILES
           path.push_back(cfg1);
         #endif
        } else if (!cfg1_free && cfg2_free) {
         _info.nodes.push_back(Cfg(cfg2));
         #if INTERMEDIATE_FILES
           path.push_back(cfg2);
         #endif
        } // endif push nodes
      } // endif BB
   } // endfor

   #if INTERMEDIATE_FILES
     WritePathTranformationMatrices("GaussPRM.path", path, _env);
   #endif

};


//===================================================================
// BasicOBPRM
//===================================================================
void
GenerateMapNodes::
BasicOBPRM(Environment *_env, CollisionDetection* cd, DistanceMetric * dm, GN& _gn, GNInfo &_info){

  #ifndef QUIET
    cout << "(numNodes=" << _info.numNodes << ", "<<flush;
    cout << "numShells=" << _info.numShells << ") "<<flush;
  #endif

  #if INTERMEDIATE_FILES
    vector<Cfg> surface;
  #endif

  vector<Cfg>  preshells, shells, tmp, obstSurface;
  int numMultiBody = _env->GetMultiBodyCount();
  int robot        = _env->GetRobotIndex();

  Cfg InsideNode, OutsideNode, low, high, mid;

  int N = _info.numNodes
                / (numMultiBody-1) 	// -1 for the robot
		/ _info.numShells;

  if (N<1) {
	cout << "\n  Not asking to generate any nodes per obstacle (N="
		<< N << ")\n\n" << flush;
	return;
  }

  for (int obstacle = 0 ; obstacle < numMultiBody ; obstacle++){
    if (obstacle != robot){  // && obstacle is "Passive" not "Active" robot

        for(int n = 0 ; n < N; n++){

		// Generate Inside cfg
		Cfg InsideNode;
		if(!GenerateInsideCfg(_env, robot, obstacle, &InsideNode)) {
                   cout << "\nError: cannot overlap COMs of robot & obstacle\n";
                   continue;
                }
		if(!InsideNode.isCollision(_env,cd,robot,obstacle,_info.cdsetid)){
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
                preshells = Shells(tmp, _info.numShells);
                shells = InsideBB(_env, preshells);
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
    else  // obst = robot : Guang 10/13/99
    if(numMultiBody == 1) {
            vector<Cfg> CobstNodes = GenCfgsFromCObst(_env, cd, dm, obstacle, _info.numNodes, _info);
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
    WritePathTranformationMatrices("surface.path", surface, _env);
  #endif

};


//===================================================================
// OBPRM
//===================================================================
void
GenerateMapNodes::
OBPRM(Environment *_env, CollisionDetection *cd ,DistanceMetric * dm,GN& _gn, GNInfo &_info){

    #ifndef QUIET
      cout << "(numNodes="          << _info.numNodes   << ", ";
      cout << "\tproportionSurface="<< _info.proportionSurface << ", ";
      cout << "\nnumShells="        << _info.numShells   << ", ";
      cout << "collPair="           << _info.collPair.GetValue()    << ", ";
      cout << "freePair="           << _info.freePair.GetValue()   << ") ";
    #endif

    pair<int,int> seedSelect,freeSelect;

    ValidatePairs("seedSelect", _info.collPair, &seedSelect);
    ValidatePairs("freeSelect", _info.freePair, &freeSelect);

    vector<Cfg> tmp, preshells, shells;

    int numMultiBody = _env->GetMultiBodyCount();
    int robot        = _env->GetRobotIndex();

    double P = _info.proportionSurface;

    // Subtract # of robots (ie, numMultiBody-1)
    int NSEED = (int)(P * _info.numNodes/(numMultiBody-1)/_info.numShells);
    int NFREE = (int)((1.0-P) * _info.numNodes/(numMultiBody-1));

    if (NSEED<1 && NFREE<1) {
        cout << "\n  Not asking to generate any nodes (NSEED="
                << NSEED << ", NFREE=" << NFREE << ")\n\n" << flush;
        return;
    }

    vector<Cfg> obstSurface, obstFree, surface;
    for(int obstacle = 0 ; obstacle < numMultiBody ; obstacle++){

        if(obstacle != robot){

	    // Generate Surface Cfgs using Binary Search Procedure
	    obstSurface = GenSurfaceCfgs4Obst(_env,cd,dm, obstacle, NSEED, _info);

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
	else  // obst = robot : Guang 10/13/99
	if(numMultiBody == 1) {
	    vector<Cfg> CobstNodes = GenCfgsFromCObst(_env, cd, dm, obstacle, _info.numNodes, _info);
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
      WritePathTranformationMatrices("surface.path", surface, _env);
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
GenSurfaceCfgs4Obst(Environment * env,CollisionDetection* cd,DistanceMetric * dm, int obstacle, int nCfgs, GNInfo &info){

    pair<int,int> seedSelect;
    ValidatePairs("seedSelect", info.collPair, &seedSelect);

    if(seedSelect.first == N_rT && seedSelect.second == N_rT)
        return Cfg::GenSurfaceCfgs4ObstNORMAL(env, cd, obstacle, nCfgs, info.cdsetid);
    else
        return GenSurfaceCfgs4ObstVERTEX(env, cd, dm, obstacle, nCfgs, info);
}


//===================================================================
// GenSurfaceCfgs4ObstVERTEX
//      generate nodes by overlapping two vertices.
//===================================================================
vector<Cfg>
GenerateMapNodes::
GenSurfaceCfgs4ObstVERTEX(Environment * env,CollisionDetection* cd,DistanceMetric * dm, int obstacle, int nCfgs, GNInfo &info){

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

	tmp = GenerateSurfaceCfg(env,cd,dm,info,robot,obstacle,obstSeeds[i],OutsideNode);

        // Choose as many as nshells
        preshells = Shells(tmp, info.numShells);
        shells = InsideBB(env, preshells);
        preshells.erase(preshells.begin(), preshells.end());

        // Collect the cfgs for this obstacle
        surface.insert(surface.end(),shells.begin(),shells.end());
    }
    return surface;
};


//===================================================================
// GenCfgsFromCObst
//      generate nodes by collecting free nodes and emitting rays from nodes in collision.
//	Guang Song 10/13/99
//===================================================================
vector<Cfg>
GenerateMapNodes::
GenCfgsFromCObst(Environment * env,CollisionDetection* cd,DistanceMetric * dm, int obstacle, int nCfgs, GNInfo &info){

    int robot = env->GetRobotIndex();
    vector<Cfg> surface, obstSeeds;
    Vector3D voidA, voidB;
    Cfg gen;
    int i;
    for(i=0; i<nCfgs; ++i) {
	Cfg::GenerateOverlapCfg(env, robot, voidA, voidB, &gen);  // voidA, voidB is not used.
	if(gen.isCollision(env,cd, info.cdsetid))
           obstSeeds.push_back(gen);
        else
	   surface.push_back(gen);
    }

    vector<Cfg> tmp, preshells, shells;
    for(i = 0 ; i < obstSeeds.size() ; i++){

        Cfg incrCfg = Cfg::GetRandomRay(EXPANSION_FACTOR*env->GetPositionRes());

        Cfg OutsideNode = GenerateOutsideCfg(env,cd,robot,obstacle,obstSeeds[i],incrCfg,info);
	if(OutsideNode.AlmostEqual(obstSeeds[i])) continue; // can not find outside node.

        tmp = GenerateSurfaceCfg(env,cd,dm,info,robot,obstacle,obstSeeds[i],OutsideNode);

        // Choose as many as nshells
        preshells = Shells(tmp, info.numShells);
        shells = InsideBB(env, preshells);
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
//  9/3/98  Daniel Vallejo
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
            if(cfg.isCollision(env,cd, _gnInfo.cdsetid)) {
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
           //if(!cfg.isCollision(env, info.cdsetid) && CfgInsideBB(env, cfg)) {
           if(!cfg.isCollision(env,cd, info.cdsetid)) {
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
      WritePathTranformationMatrices("att.path", att, env);
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
    while((delta >= PositionRes) && (cnt < MAX_CONVERGE)){
        if(mid.isCollision(env,cd , rob, obst, info.cdsetid) ) {
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
        if(!high.isCollision(env,cd, info.cdsetid)) {
            surface = FreeCfgs(env, cd,tmp, info);
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


//-----------------------------------------
// Used by "FarthestFromStart" for sort
//-----------------------------------------
bool
GenerateMapNodes::
DIST_Compare (const VID_DISTANCE_TYPE &_cc1, const VID_DISTANCE_TYPE &_cc2) {
        return (_cc1.second > _cc2.second ) ;
};


//===================================================================
// FarthestFromStart
//===================================================================

void
GenerateMapNodes::
FarthestFromStart(Environment * env, DistanceMetric * dm,GNInfo &info,
	Cfg start, vector<Cfg> spread, vector<Cfg>* farNode){


    vector<pair <int, double> > ds;


    for(int i = 0 ; i < spread.size() ; i++){
        ds.push_back( VID_DISTANCE_TYPE(
			i,
			dm->Distance(env, start, spread[i], info.dmsetid))
                    );
    }
    sort(ds.begin(), ds.end(), ptr_fun(DIST_Compare));

    for(int j = 0 ; j < spread.size() ; j++){


		farNode->push_back(spread[ds[j].first]);

    }
}


//===================================================================
// FirstNFreeCfgs
//===================================================================
void
GenerateMapNodes::
FirstNFreeCfgs(Environment *env,CollisionDetection *cd, GNInfo &info,
	int n, vector<Cfg> cfgs, vector<Cfg>* free){

    int size = cfgs.size();
    n = min(n,size);

    int i = 0; int cnt = 0;
    while(i < size && cnt < n){
	if(!cfgs[i].isCollision(env,cd, info.cdsetid)){
            free->push_back(cfgs[i]);
            cnt++;
        }
        i++;
    }
}


//==============================================================
// GenNewPivots
//==============================================================
void
GenerateMapNodes::
GenNewPivots(Environment *env,CollisionDetection *cd, DistanceMetric * dm,GNInfo &info,
	Cfg start, vector<Cfg> pivots,
	double tStep, double rStep, int nspread, int nFar,
        vector<Cfg>* newPivots){

    int num = pivots.size();
    vector<Cfg> spread; spread.reserve(num * nspread);
    for(int i = 0 ; i < num ; i++){
        Spread(pivots[i], tStep, rStep, nspread, &spread);
    }
    vector<Cfg> farNode; farNode.reserve(spread.size());
    FarthestFromStart(env,dm, info, start, spread, &farNode);
    FirstNFreeCfgs(env,cd, info, nFar, farNode, newPivots);
}


//==============================================================
// SpreadCfg
//==============================================================
void
GenerateMapNodes::
SpreadCfg(Environment *env,CollisionDetection *cd,DistanceMetric * dm, GNInfo &info,
        Cfg start,
        double tStep,
        double rStep,
        int nspread,
        int nFar,
        int nIterations){

    //-- default values for everything except "start" configuration

            if(tStep <= 0){
                tStep = 0.1;
            }
            if(rStep <= 0){
                rStep = 0.1;
            }
            if(nspread <= 0){
                nspread = 100;
            }
            if(nFar <= 0){
                nFar = 20;
            }
            if(nIterations <= 0){
                nIterations = 5;
            }

    vector<Cfg> spread; spread.reserve(nspread);
    vector<Cfg> farNode;       farNode.reserve(nspread);
    vector<Cfg> pivots; pivots.reserve(nFar);

    #if INTERMEDIATE_FILES
      vector<Cfg> free; free.reserve(nFar*nIterations + 1);
      free.push_back(start);
    #endif

    info.nodes.push_back(start);
    pivots.push_back(start);

    vector<Cfg> tmp;
    for(int j = 0 ; j < nIterations ; j++){

	if(pivots.size() == 0) break;
	tmp.reserve(nFar);
        GenNewPivots(env,cd,dm,info,
		start, pivots, tStep, rStep, nspread, nFar, &tmp);

	pivots.erase(pivots.begin(),pivots.end());
	pivots.reserve(nFar);

	// Collect all the generated nodes
        for(int i=0; i<tmp.size(); i++)
	  info.nodes.push_back(Cfg(tmp[i]));
	pivots.insert(pivots.end(),tmp.begin(),tmp.end());
	#if INTERMEDIATE_FILES
	  free.insert(free.end(),tmp.begin(),tmp.end());
	#endif

	tmp.erase(tmp.begin(),tmp.end());

    } // endfor(j)

#if INTERMEDIATE_FILES
    WritePathTranformationMatrices("ss.path", free, env);
#endif

};


//===================================================================
// FreeCfgs
//===================================================================
vector<Cfg>
GenerateMapNodes::
FreeCfgs(Environment *env,CollisionDetection *cd, vector<Cfg> cfgs, GNInfo &info){
    int size = cfgs.size();
    vector<Cfg> free;
    free.reserve(size);
    for(int i = 0 ; i < size ; i++){
	if(!cfgs[i].isCollision(env,cd, info.cdsetid)) {
            free.push_back(cfgs[i]);
        }
    }
    return free;
}


//===================================================================
// InsideBB
//===================================================================
vector<Cfg>
GenerateMapNodes::
InsideBB(Environment *env, vector<Cfg> cfgs){

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

        if(polyhedron.vertexList[i][0] < polyhedron.vertexList[indexVert[0]][0])
            indexVert[0] = i;

        if(polyhedron.vertexList[i][0] > polyhedron.vertexList[indexVert[1]][0])
            indexVert[1] = i;

        if(polyhedron.vertexList[i][1] < polyhedron.vertexList[indexVert[2]][1])
            indexVert[2] = i;

        if(polyhedron.vertexList[i][1] > polyhedron.vertexList[indexVert[3]][1])
            indexVert[3] = i;

        if(polyhedron.vertexList[i][2] < polyhedron.vertexList[indexVert[4]][2])
            indexVert[4] = i;

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
//  2/2/98  Daniel Vallejo
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
GenerateInsideCfg(Environment *_env, int rob, int obst, Cfg * insideNode){

    _env->GetMultiBody(obst)->ComputeCenterOfMass();
    bool tmp = Cfg::GenerateOverlapCfg(_env, rob,
                _env->GetMultiBody(rob)->GetCenterOfMass(),
                _env->GetMultiBody(obst)->GetCenterOfMass(),
                insideNode);
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
    while(OutsideNode.isCollision(env,cd, rob, obst, info.cdsetid) ) {
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
GN() {
  strcpy(name,"");
  Gauss_d = 0;
  generator = 0;
  gnid = INVALID_EID;
};

GN::
~GN() {
};

bool
GN::
operator==(const GN& _gn) const
{
  if ( strcmp(name,_gn.name) != 0 ) {
     return false;
  } else if ( !strcmp(name,"BasicPRM") ) {
     return true;
  } else if ( !strcmp(name,"BasicOBPRM") ) {
     return true;
  } else if ( !strcmp(name,"OBPRM") ) {
     return true;
  } else if ( !strcmp(name,"GaussPRM") ) {
     return ( Gauss_d == _gn.Gauss_d );
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

double
GN::
Get_Gauss_d() const {
  if ( !strcmp(name,"GaussPRM") ) {
    return Gauss_d;
  } else {
    return -1;
  }
};

ostream& operator<< (ostream& _os, const GN& gn) {
    _os<< gn.GetName();
    if ( !strstr(gn.GetName(),"GaussPRM") ){
           _os<< ", d = " << gn.Get_Gauss_d();
    }
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
  DEFAULT_Gauss_d = _env->Getminmax_BodyAxisRange();
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


SID
GNSets::
MakeGNSet(istream& _myistream) {
  char gnname[100];
  vector<EID> gnvec;  // vector of gnids for this set

  while ( _myistream >> gnname ) { // while gns to process...
    if (!strcmp(gnname,"BasicPRM")) {           // BasicPRM
       GN gn1;
       strcpy(gn1.name,gnname);
       gn1.generator = &GenerateMapNodes::BasicPRM;
       gn1.gnid = AddElementToUniverse(gn1);
       if ( ChangeElementInfo(gn1.gnid,gn1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       gnvec.push_back( gn1.gnid );

    } else if (!strcmp(gnname,"GaussPRM")) {
       GN gn1; double Gauss_d;
       strcpy(gn1.name,gnname);
       gn1.generator = &GenerateMapNodes::GaussPRM;
       gn1.Gauss_d = 0;
       if( _myistream >> Gauss_d) { // get d value
          if ( Gauss_d < 0 ) {
            cout << endl << "INVALID: d = " << Gauss_d;
            exit(-1);
          } else {
            gn1.Gauss_d = Gauss_d;
            gn1.gnid = AddElementToUniverse(gn1);
            if( ChangeElementInfo(gn1.gnid,gn1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
            gnvec.push_back( gn1.gnid );
          }
       }
       _myistream.clear(); // clear failure to read parameters

       if(gn1.Gauss_d == 0) {  //if no Gauss_d value given, use default
            gn1.Gauss_d = DEFAULT_Gauss_d;
            gn1.gnid = AddElementToUniverse(gn1);
            if( ChangeElementInfo(gn1.gnid,gn1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
            gnvec.push_back( gn1.gnid );
       }


    } else if (!strcmp(gnname,"BasicOBPRM")) { 	// BasicOBPRM
       GN gn1;
       strcpy(gn1.name,gnname);
       gn1.generator = &GenerateMapNodes::BasicOBPRM;
       gn1.gnid = AddElementToUniverse(gn1);
       if ( ChangeElementInfo(gn1.gnid,gn1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       gnvec.push_back( gn1.gnid );

    } else if (!strcmp(gnname,"OBPRM")) {  // OBPRM
       GN gn1;
       strcpy(gn1.name,gnname);
       gn1.generator = &GenerateMapNodes::OBPRM;
       gn1.gnid = AddElementToUniverse(gn1);
       if ( ChangeElementInfo(gn1.gnid,gn1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       gnvec.push_back( gn1.gnid );

    } else {
       cout << "INVALID: map node generator name = " << gnname;
       exit(-1);
    }
  } // end while

  return MakeOSet(gnvec);
}

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
             _myostream << gns[i].Gauss_d;
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




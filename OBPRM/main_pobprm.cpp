///////////////////////////////////////////////////////////////////
//
// main_pobprm : parallized obprm main 
// main_pobprm.cpp
// 
// created by Jyh-Ming Lien
// 2001 Spring
///////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream.h>
#include <assert.h>

///////////////////////////////////////////////////////////////////
// Include pOBPRM headers
#include "main_pobprm.h"

///////////////////////////////////////////////////////////////////
// Include OBPRM headers
#include "MultiBody.h"
#include "Clock_Class.h"
#include "Stat_Class.h"
#include "SwitchDefines.h"

///////////////////////////////////////////////////////////////////
//
//  Parallel Environment class
//
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
//  Parallel Environment class: MultiBody
///////////////////////////////////////////////////////////////////
int pEnvironment::GetMultiBodyCount()
{
    //The "real" number of multi body defined in env file
    //because robot was duplicated for each processor
    int numMB = Environment::GetMultiBodyCount();
    if(m_RobotDuplicated)
        numMB-=(CPOBPRM_MPComponents::m_NumberOfProcessor-1);
    return numMB;
}

MultiBody * pEnvironment::GetMultiBody(int _index){
    if( _index==GetRobotIndex() ){
        //get robot..
        int PID=m_get_myid();
        if( PID!=0 )  _index=GetMultiBodyCount()+PID-1;
    }
    return Environment::GetMultiBody(_index);
}

////////////////////////////////////////////////////////////////////
//    Parallel Environment class : Helper functions
///////////////////////////////////////////////////////////////////
void pEnvironment::Get(Input * input)
{
    Environment::Get(input);

    //This could be parallelized
    m_RobotDuplicated = false;
    DulpicateRobot(input);
    m_RobotDuplicated = true;
}

void pEnvironment::DulpicateRobot(Input * input)
{
    int pNum = CPOBPRM_MPComponents::m_NumberOfProcessor;
    for (int iP = 1; iP<pNum; iP++) 
    {
        //dulpilcate for each processor
        MultiBody * mb = new MultiBody(this);
        mb->Get(input, Environment::GetRobotIndex());
        AddMultiBody(mb);
    }
}

///////////////////////////////////////////////////////////////////
//
//  Parallel Node Generation class
//
///////////////////////////////////////////////////////////////////

//Constructor
pGenerateMapNodes::pGenerateMapNodes( CPOBPRM_MPComponents & rMPC )
:GenerateMapNodes(), m_rMPC(rMPC){}

///over ride base class's GenerateNodes
void pGenerateMapNodes::UserInit(Input * input, Environment *_env)
{
    if( CheckStatus()==false ) return;
    GenerateMapNodes::UserInit(input, _env);

    //re-assign number of nodes will be generated
    vector< pair<EID,GN> > GNs=generators.GetElements();
    int totalProc = CPOBPRM_MPComponents::m_NumberOfProcessor;

    for(int iE=0; iE<GNs.size(); iE++ )
    {
        int totalNumber = (int)GNs[iE].second.numNodes.GetValue();
        int numNodePerProc = totalNumber/totalProc;
        //assign remain number to first k processors
        int remain = totalNumber-totalProc*numNodePerProc;
        if( m_rMPC.GetPID()< remain ) numNodePerProc++;
        //1 is minima accpetable value..... :<
        if( numNodePerProc==0 ) numNodePerProc=1;
        GNs[iE].second.numNodes.PutValue(numNodePerProc);
        generators.ChangeElementInfo(GNs[iE].first,GNs[iE].second);
    }
}

///over ride base class's GenerateNodes
void pGenerateMapNodes::GenerateNodes
(Roadmap* _rm, CollisionDetection* _cd,DistanceMetric* _dm, SID sid, GNInfo& gninfo)
{
    if( CheckStatus()==false ) return;
    GenerateMapNodes::GenerateNodes(_rm,_cd,_dm, sid, gninfo);
}

//Check current status
bool pGenerateMapNodes::CheckStatus(){
    int PID = m_rMPC.GetPID();
    return !( PID<0 || PID>CPOBPRM_MPComponents::m_NumberOfProcessor );
}

///////////////////////////////////////////////////////////////////
//
//  Parallel Node Connection class
//
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
//  Parallel Node Connection class : Constructor
///////////////////////////////////////////////////////////////////
pConnectMapNodes::pConnectMapNodes( CPOBPRM_MPComponents & rMPC )
:ConnectMapNodes(), m_rMPC(rMPC){}

void pConnectMapNodes::UserInit(Input * input, Environment * env){
    if( CheckStatus()==false ) return; //something is wrong...
    ConnectMapNodes::UserInit(input, env);
    //override "old" connectors
    ReplaceConnector();
}

///over ride base class's GenerateNodes
void pConnectMapNodes::ConnectNodes
(Roadmap * rdmp, CollisionDetection *cd,LocalPlanners* lp, DistanceMetric * dm, SID _cnsetid, CNInfo &info)
{
    if( CheckStatus()==false ) return; //something is wrong...

    //now call ConnectMapNodes::ConnectNodes
    ConnectMapNodes::ConnectNodes(rdmp, cd, lp, dm, _cnsetid, info);
}

///////////////////////////////////////////////////////////////////
//  Parallel Node Connection class : pOBPRM Connection Methods
///////////////////////////////////////////////////////////////////

void pConnectMapNodes::ConnectNodes_Random
(Roadmap * _rm, CollisionDetection* cd, LocalPlanners* lp, 
 DistanceMetric * dm, CN& _cn, CNInfo& info)
{
    int totalProc = CPOBPRM_MPComponents::m_NumberOfProcessor;
    int totalNumber = (int)_cn.GetNumEdges();
    int numNodePerProc = totalNumber/totalProc;
    //assign remain number to first k processors
    int remain = totalNumber-totalProc*numNodePerProc;
    if( m_get_myid()< remain ) numNodePerProc++;
    _cn.SetNumEdges(numNodePerProc);

    ConnectMapNodes::ConnectNodes_Random(_rm, cd, lp, dm, _cn, info);
}

void pConnectMapNodes::ConnectNodes_Closest
(Roadmap * _rm, CollisionDetection* cd, LocalPlanners* lp, 
 DistanceMetric * dm, CN& _cn, CNInfo& info)
{
    int PID=m_get_myid();

    vector<Cfg> allVectices = _rm->m_pRoadmap->GetVerticesData();
    int totalNumber = (int)allVectices.size();
    const int k = min(_cn.GetKClosest(),totalNumber);

    vector< pair<int,VID> > allCCs = _rm->m_pRoadmap->GetCCStats();
    int totalProc = CPOBPRM_MPComponents::m_NumberOfProcessor;
    int numNodePerProc = totalNumber/totalProc;
    //assign remain number to first k processors
    int remain = totalNumber-totalProc*numNodePerProc;
    //find responsible cfgs (index and numNodePerProc)
    int index=0;
    if( PID< remain ){
        index=(numNodePerProc+1)*PID;
        numNodePerProc++;
    }
    else{
        index=numNodePerProc*PID+remain;
    }

    if(PID!=0 ) index--;
    vector< Cfg > cfgs;
    cfgs.insert
    (cfgs.begin(), allVectices.begin()+index, allVectices.begin()+index+numNodePerProc);

    //call override ConnectNodes_Closest
    ConnectMapNodes::ConnectNodes_Closest(_rm, cd, lp, dm, _cn, info, cfgs, allVectices, k);
}

void pConnectMapNodes::ConnectNodes_ConnectCCs
(Roadmap * _rm, CollisionDetection* cd, LocalPlanners* lp, 
 DistanceMetric * dm, CN& _cn, CNInfo& info)
{
    int PID=m_get_myid();
    vector< pair<int,VID> > allCCs = _rm->m_pRoadmap->GetCCStats();
    int totalProc = CPOBPRM_MPComponents::m_NumberOfProcessor;
    int totalNumber = (int)allCCs.size();
    int numNodePerProc = totalNumber/totalProc;
    //assign remain number to first k processors
    int remain = totalNumber-totalProc*numNodePerProc;
    //find responsible cfgs (index and numNodePerProc)
    int index=0;
    if( PID< remain ){
        index=(numNodePerProc+1)*PID;
        numNodePerProc++;
    }
    else{
        index=numNodePerProc*PID+remain;
    }

    vector< pair<int,VID> > ccs;
    ccs.insert(ccs.begin(), allCCs.begin()+index, allCCs.begin()+index+numNodePerProc);
    allCCs.erase(allCCs.begin()+index+numNodePerProc, allCCs.end());
    ConnectMapNodes::ConnectNodes_ConnectCCs(_rm, cd, lp, dm, _cn, info, ccs,allCCs);
}

void pConnectMapNodes::NOT_IMPLEMENTED_FUNC
(Roadmap * _rm, CollisionDetection* cd, LocalPlanners* lp, 
 DistanceMetric * dm, CN& _cn, CNInfo& info)
{
    cerr<<"! pOBPRM Error: "<<_cn.GetName()<<" is not implemented"<<endl;
}


///////////////////////////////////////////////////////////////////
//  Parallel Node Connection class : pOBPRM Help methods
///////////////////////////////////////////////////////////////////
void pConnectMapNodes::ReplaceConnector(){

    //Get set for modification...
    vector< pair<EID,CN> > cnset=connectors.GetElements();
    //These funcions are overloaded in ConnectMapNodes, so can't compare them directly.
    CNF CM_CLOSEST = ConnectMapNodes::ConnectNodes_Closest;
    CNF CM_CC = ConnectMapNodes::ConnectNodes_ConnectCCs;

    //replace "old" node connection method with parallelized method
    for(int cn=0; cn < cnset.size(); ++cn) {
        EID eid   = cnset[cn].first;
        CN & rCN  = cnset[cn].second;
        CNF cnfcn = rCN.GetConnector();

        if( cnfcn==ConnectMapNodes::ConnectNodes_Random )
            rCN.GetConnector() = pConnectMapNodes::ConnectNodes_Random;
        else if( cnfcn==CM_CLOSEST )
            rCN.GetConnector() = pConnectMapNodes::ConnectNodes_Closest;
        else if( cnfcn==CM_CC )
            rCN.GetConnector() = pConnectMapNodes::ConnectNodes_ConnectCCs;
        else //otherwise these function are not implemented
            rCN.GetConnector() = pConnectMapNodes::NOT_IMPLEMENTED_FUNC;

        connectors.ChangeElementInfo(eid,rCN);
    }
}

///////////////////////////////////////////////////////////////////
//  Parallel Node Connection class : Protected Methods
///////////////////////////////////////////////////////////////////
//Check current status
bool pConnectMapNodes::CheckStatus(){
    return !( m_rMPC.GetPID()<0 || m_rMPC.GetPID()>CPOBPRM_MPComponents::m_NumberOfProcessor );
}

///////////////////////////////////////////////////////////////////
//
//  Parallel Node Generation class
//
///////////////////////////////////////////////////////////////////

unsigned int CPOBPRM_MPComponents::m_NumberOfProcessor=0;

bool
CPOBPRM_MPComponents::doInit(unsigned int PID){
    if( m_pMap==NULL || m_pInput==NULL )
        return false;
    m_ProcessorID = PID;
    
    //Generate components
    m_ppGN = new pGenerateMapNodes(*this);
    m_ppCN = new pConnectMapNodes(*this);
    m_pLP = new LocalPlanners();
    m_pDM = new DistanceMetric();
    m_pCD = new CollisionDetection();

    //check memory
    if(m_ppGN==NULL||m_ppCN==NULL||m_pLP==NULL||m_pDM==NULL||m_pCD==NULL)
        return false;

    //init other components use user provided info
    m_pCD->UserInit (m_pInput, m_ppGN, m_ppCN);
    m_pLP->UserInit (m_pInput, m_ppCN );
    m_ppGN->UserInit(m_pInput, m_pMap->GetEnvironment() );
    m_ppCN->UserInit (m_pInput, m_pMap->GetEnvironment() );
    m_pDM->UserInit (m_pInput, m_ppGN, m_pLP);

    return true;
}

///////////////////////////////////////////////////////////////////
//
//  Main Function and other help sub functions
//
///////////////////////////////////////////////////////////////////
Stat_Class Stats;
#define ARGV_1 "number_of_processors"

void Init(Roadmap& ,Input & ,pvector<CPOBPRM_MPComponents> &);
void GenerateMapNodes(Roadmap& ,Input & ,pvector<CPOBPRM_MPComponents> &);
void ConnectMapNodes(Roadmap& ,Input & ,pvector<CPOBPRM_MPComponents> &);
void OutputMap(Roadmap& ,Input & ,pvector<CPOBPRM_MPComponents> &);

int main( int argc, char ** argv ){

    if( argc<2 ){
        cerr<<"! pOBPRM Error: Not Enough Parameters"<<endl;
        return 1;
    }

    ////////////////////////////////////////////////////////////
    //
    // (1) Set number of processors are going to be used
    // (2) Create Motion Planning Components
    // (3) Create Componets shared by all processors (Input and Roadmap)
    //
    ////////////////////////////////////////////////////////////
    
    //create MPC list
    unsigned int cNumProc = atoi(argv[1]); //number of processors
    //set environment variable
    char variable[50];
    sprintf(variable,"MP_NUMBER_OF_THREADS=%d",  cNumProc);
    putenv(variable);
    CPOBPRM_MPComponents::m_NumberOfProcessor = cNumProc;
    //create components
    pvector<CPOBPRM_MPComponents> MPC(cNumProc);

    //create input
    Input input;
    char * new_argv_1 = new char[strlen(argv[0])+strlen(ARGV_1)+2];
    sprintf(new_argv_1, "%s %s",argv[0],ARGV_1);
    argv[1]=new_argv_1;
    input.ReadCommandLine(--argc,++argv);

    //create roadmap graph
    Roadmap rmap;

    ////////////////////////////////////////////////////////////
    //
    // (1) Init Componets in parallel
    // (2) Create Map Nodes in parallel
    // (3) Connect Map Nodes in parallel
    // (4) Write Map to file
    //
    ////////////////////////////////////////////////////////////

    cout<<"- pOBPRM Msg : Init Motion Planner Components"<<endl;
    Init(rmap, input, MPC);
    cout<<"- pOBPRM Msg : Generate Road Map Nodes"<<endl;
    GenerateMapNodes(rmap, input, MPC);
    cout<<"- pOBPRM Msg : Connect Road Map Nodes"<<endl;
    ConnectMapNodes(rmap, input, MPC);
    cout<<"- pOBPRM Msg : Write Road Map Graph to file"<<endl;
    OutputMap(rmap, input, MPC);
    cout<<"- pOBPRM Msg : Done"<<endl;

    return 0; //every thing is fine
}

void Init(Roadmap& rmap, Input & input, pvector<CPOBPRM_MPComponents> & MPC){

    //replace roadmap graph
    pRoadmapGraph<Cfg, WEIGHT> * prmapGraph = new pRoadmapGraph<Cfg, WEIGHT>;
    delete rmap.m_pRoadmap;
    rmap.m_pRoadmap = prmapGraph;

    //create pEnv and assign it to roadmap
    pEnvironment * pEnv = new pEnvironment();
    rmap.InitRoadmap(&input, NULL, NULL, NULL, NULL, pEnv);
    rmap.InitEnvironment(&input);

    //init CPOBPRM_MPComponents in Parallel
    pvector<CPOBPRM_MPComponents>::prange_type MPCpr = MPC.get_prange();
    p_Init<pvector<CPOBPRM_MPComponents>::prange_type::s_range_type> pInit(input,rmap);
    __p_for_all1(&MPCpr,&pInit);
}

void GenerateMapNodes(Roadmap& rmap,Input & input, pvector<CPOBPRM_MPComponents> & MPC )
{
    //read from file
    if ( input.inmapFile.IsActivated() ){
        rmap.ReadRoadmapGRAPHONLY(input.inmapFile.GetValue());
        return;
    }
    
    //otherwise genertated bu myself
    pvector<CPOBPRM_MPComponents>::prange_type MPCpr = MPC.get_prange();
    p_GNodes<pvector<CPOBPRM_MPComponents>::prange_type::s_range_type> pGNode;
    //setup clocks
    Clock_Class NodeGenClock;
    NodeGenClock.StartClock("- pOBPRM Msg : Node Generate Time (rusage)");
    //start to generate
    __p_for_all1(&MPCpr,&pGNode);
    //print time info
    NodeGenClock.StopClock(); NodeGenClock.PrintName(); 
    cout<<" : "<<NodeGenClock.GetClock_SEC()/CPOBPRM_MPComponents::m_NumberOfProcessor<<" sec."<<endl;
    //output info
    cout<< "- pOBPRM Msg : "<<rmap.m_pRoadmap->GetVertexCount()
        << " Map Nodes"<<endl;
}

void ConnectMapNodes(Roadmap& rmap,Input & input, pvector<CPOBPRM_MPComponents> & MPC )
{
    pvector<CPOBPRM_MPComponents>::prange_type MPCpr = MPC.get_prange();
    p_CNodes<pvector<CPOBPRM_MPComponents>::prange_type::s_range_type> pCNode;
    //setup clocks
    Clock_Class NodeConClock;
    NodeConClock.StartClock("- pOBPRM Msg : Node Connect Time (rusage)");
    //start to generate
    __p_for_all1(&MPCpr,&pCNode);
    //print time info
    NodeConClock.StopClock(); NodeConClock.PrintName(); 
    cout<<" : "<<NodeConClock.GetClock_SEC()/CPOBPRM_MPComponents::m_NumberOfProcessor<<" sec."<<endl;
    //output info
    cout<< "- pOBPRM Msg : "<<rmap.m_pRoadmap->GetCCcount() 
        << " Connected Components"<<endl;
}

void OutputMap(Roadmap& rmap,Input & input, pvector<CPOBPRM_MPComponents> & MPC)
{
    rmap.WriteRoadmap(&input,MPC[0].GetCD(),MPC[0].GetDM(),MPC[0].GetLP());
}

#ifndef _MAIN_POBPRM_H_
#define _MAIN_POBPRM_H_

///////////////////////////////////////////////////////////////////
//
// main_pobprm : parallized obprm main
// main_pobprm.h
//
// created by Jyh-Ming Lien
// 2001 Spring
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// Include OBPRM headers
#include "OBPRM.h"
#include "Roadmap.h"
#include "Input.h"
#include "CollisionDetection.h"
#include "DistanceMetrics.h"
#include "Environment.h"
#include "GenerateMapNodes.h"

///////////////////////////////////////////////////////////////////
// Include Stapl headers
#include <p_algorithm.h>
#include <p_vector.h>
#include <stapl_lock.h>
using namespace stl;
using namespace stapl;

class CPOBPRM_MPComponents; //defined in later part of this file.

///////////////////////////////////////////////////////////////////
//
//  Parallel Environment class
//
///////////////////////////////////////////////////////////////////
class pEnvironment : public Environment
{
public:

    ///////////////////////////////////////////////////////////////////
    //  MultiBody
    ///////////////////////////////////////////////////////////////////
    /*@name MultiBody Access Methods*/
    //@{
        ///Return the number of MultiBody's in the environment
        inline virtual int GetMultiBodyCount();
        virtual MultiBody * GetMultiBody(int _index);
    //@}

    ////////////////////////////////////////////////////////////////////
    //    Helper functions
    ///////////////////////////////////////////////////////////////////
    /*@name Helper functions*/
    //@{
        virtual void Get(Input * _input);
        virtual void DulpicateRobot(Input * _input);
        void Output();
    //@}

private:
    bool m_RobotDuplicated; ///<Set as true while robot was duplicated
};

///////////////////////////////////////////////////////////////////
//
//  Parallel Roadmap class
//
///////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
class pRoadmapGraph : public RoadmapGraph<VERTEX,WEIGHT>
{
public:
    ////////////////////////////////////////////////////////////////////
    //    Constructors and Destructor
    ///////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{
        pRoadmapGraph();
        pRoadmapGraph(int _sz);  // 'reserve' space for verts (default for edges)
        pRoadmapGraph(int _sz, int _edgelistsz); // 'reserve' space for verts & edges
    //@}    
    
    ////////////////////////////////////////////////////////////////////
    //    Adding & Deleting Vertices & Edges
    ///////////////////////////////////////////////////////////////////
    /**@name Adding & Deleting Vertices & Edges*/
    //@{
        virtual VID  AddVertex(VERTEX&); 
        virtual VID  AddVertex(vector<VERTEX>&);
        virtual int  AddEdges( vector<EdgeInfo<WEIGHT> >& );
        virtual int  AddEdge(VID, VID, WEIGHT);
        virtual int  AddEdge(VERTEX&, VERTEX&, WEIGHT);
        virtual int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>);
        virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>);
        //flush vertex to roadmap
        virtual void FlushVertex();
    //@}


    
    virtual WeightedMultiDiGraph<VERTEX,WEIGHT> BFS(VID) const;

private:
    Stapl_Lock<short> m_Lock;
    vector< vector<Cfg> > m_Vertex;
};

///////////////////////////////////////////////////////////////////
//
//  Parallel Node Generation class
//
///////////////////////////////////////////////////////////////////
class pGenerateMapNodes : public GenerateMapNodes
{
public:
    //Constructor
    pGenerateMapNodes( CPOBPRM_MPComponents & rMPC );

    ///over ride base class's GenerateNodes
    virtual void GenerateNodes
    (Roadmap* _rm, CollisionDetection*,DistanceMetric*, SID, GNInfo&);

    ///over ride base class's GenerateNodes
    virtual void UserInit(Input * input, Environment *_env);

protected:
    virtual inline bool CheckStatus();
    CPOBPRM_MPComponents & m_rMPC;
};

///////////////////////////////////////////////////////////////////
//
//  Parallel Node Connection class
//
///////////////////////////////////////////////////////////////////

class pConnectMapNodes : public ConnectMapNodes
{
public:
    //Constructor
    pConnectMapNodes( CPOBPRM_MPComponents & rMPC );

    ///over ride base class's ConnectNodes
    virtual void ConnectNodes(Roadmap * rdmp, CollisionDetection *cd,
                              LocalPlanners* lp, DistanceMetric * dm, 
                              SID _cnsetid, CNInfo &info);

    ///over ride base class's UserInit
    virtual void UserInit(Input * input, Environment * env);

///////////////////////////////////////////////////////////////////
//  pOBPRM Connection Methods
///////////////////////////////////////////////////////////////////
protected:

    static void ConnectNodes_Random
    (Roadmap * _rm, CollisionDetection* cd, LocalPlanners* lp, 
     DistanceMetric * dm, CN& _cn, CNInfo& info);

    static void ConnectNodes_Closest
    (Roadmap * _rm, CollisionDetection* cd, LocalPlanners* lp, 
     DistanceMetric * dm, CN& _cn, CNInfo& info);

    static void ConnectNodes_ConnectCCs
    (Roadmap * _rm, CollisionDetection* cd, LocalPlanners* lp, 
     DistanceMetric * dm, CN& _cn, CNInfo& info);

    static void NOT_IMPLEMENTED_FUNC
    (Roadmap * _rm, CollisionDetection* cd, LocalPlanners* lp, 
     DistanceMetric * dm, CN& _cn, CNInfo& info);
///////////////////////////////////////////////////////////////////
//  pOBPRM Help methods
///////////////////////////////////////////////////////////////////
protected:

    //replace default connectors by paralel connector
    virtual void ReplaceConnector();

///////////////////////////////////////////////////////////////////
//  Protected Methods
///////////////////////////////////////////////////////////////////
protected:
    virtual inline bool CheckStatus();
    CPOBPRM_MPComponents & m_rMPC;
};

///////////////////////////////////////////////////////////////////
//
//  CPOBPRM_MPComponents class
//
///////////////////////////////////////////////////////////////////

/**
* This class contains motion planning components.
* For example, node generator, node connector, and local
* planner. 
*/
class CPOBPRM_MPComponents{
public:
    
    static unsigned int m_NumberOfProcessor;

    CPOBPRM_MPComponents(){
        //set every thing to NULL
        memset(this,NULL,sizeof(CPOBPRM_MPComponents));
    }
    
    virtual ~CPOBPRM_MPComponents(){
        if( m_ppGN!=NULL ) delete m_ppGN;
        if( m_ppCN!=NULL ) delete m_ppCN;
        if( m_pLP!=NULL ) delete m_pLP;
        if( m_pDM!=NULL ) delete m_pDM;
        if( m_pCD!=NULL ) delete m_pCD;
        if( m_pMap!=NULL ){
            m_pMap->m_pRoadmap=NULL;
            delete m_pMap;
        }
    }
    
    virtual bool doInit(unsigned int PID);

///////////////////////////////////////////////////////////////////
//  Access methods
///////////////////////////////////////////////////////////////////
    pGenerateMapNodes * GetGN() const{ return  m_ppGN; }
    void SetGN( pGenerateMapNodes* ppGN) { m_ppGN=ppGN; }
    
    pConnectMapNodes * GetCN() const{ return  m_ppCN; }
    void SetGN( pConnectMapNodes * pCN) { m_ppCN=pCN; }
    
    LocalPlanners * GetLP() const{ return  m_pLP; }
    void SetLP(LocalPlanners * pLP) { m_pLP=pLP; }
    
    DistanceMetric * GetDM() const{ return  m_pDM; }
    void SetGN(DistanceMetric * pDM) { m_pDM=pDM; }
    
    CollisionDetection * GetCD() const{ return  m_pCD; }
    void SetGN(CollisionDetection * pCD) { m_pCD=pCD; }
    
    Roadmap * GetRoadmap() const{ return  m_pMap; }
    void SetRoadmap(Roadmap * pMap) { m_pMap=pMap; }
    
    Input* GetInput() const{ return  m_pInput; }
    void SetInput(Input * pInput) { m_pInput=pInput; }
    
    unsigned int GetPID(){ return m_ProcessorID; }
///////////////////////////////////////////////////////////////////
//  Private data and methods
///////////////////////////////////////////////////////////////////
private:
    pGenerateMapNodes   * m_ppGN;
    pConnectMapNodes    * m_ppCN;
    LocalPlanners       * m_pLP;
    DistanceMetric      * m_pDM;
    CollisionDetection  * m_pCD;

    //shared by all processors
    Input           * m_pInput;
    Roadmap         * m_pMap;

    unsigned int m_ProcessorID;
};

///////////////////////////////////////////////////////////////////
//
//  p_function classes
//
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
//init CPOBPRM_MPComponents in Parallel
template<class pRange>
class p_Init : public p_function {
public:
    p_Init(Input & input, Roadmap & map)
    :m_Input(input), m_Map(map){}
    
    void operator()(pRange& pr) {
        for (typename pRange::iterator i = pr.begin(); i != pr.end(); i++) {
            i->SetRoadmap(&m_Map); i->SetInput(&m_Input);
            bool bResult = i->doInit(m_get_myid());
            assert( bResult );
        }
    }
private:
    Input & m_Input;
    Roadmap & m_Map;
};

///////////////////////////////////////////////////////////////////
//Generate Nodes in Parallel
template<class pRange>
class p_GNodes : public p_function {
public:
    void operator()(pRange& pr) {
        for (typename pRange::iterator i = pr.begin(); i != pr.end(); i++) {
            //prepare components
            GenerateMapNodes *   pGN = i->GetGN();
            CollisionDetection * pCD = i->GetCD();
            DistanceMetric *     pDM = i->GetDM();
            Roadmap *           pMap = i->GetRoadmap();
            GNInfo & info=pGN->gnInfo;

            //call generate func
            pGN->GenerateNodes(pMap,pCD,pDM, info.gnsetid, info);
            ((pRoadmapGraph<Cfg,WEIGHT>*)pMap->m_pRoadmap)->FlushVertex();
        }
    }
};


///////////////////////////////////////////////////////////////////
//Connect Nodes in Parallel
template<class pRange>
class p_CNodes : public p_function {
public:
    void operator()(pRange& pr) {
        for (typename pRange::iterator i = pr.begin(); i != pr.end(); i++) {
            //prepare components
            GenerateMapNodes *   pGN = i->GetGN();
            ConnectMapNodes *    pCN = i->GetCN();
            LocalPlanners *      pLP = i->GetLP();
            CollisionDetection * pCD = i->GetCD();
            DistanceMetric *     pDM = i->GetDM();
            Roadmap * pMap = i->GetRoadmap();
            CNInfo & info=pCN->cnInfo;

            //call generate func
            pCN->ConnectNodes(pMap,pCD,pLP,pDM, info.cnsetid, info);
        }
    }
};

///////////////////////////////////////////////////////////////////
//
//  Parallel Roadmap class implementation
//
///////////////////////////////////////////////////////////////////

template<class VERTEX, class WEIGHT>
pRoadmapGraph<VERTEX,WEIGHT>::pRoadmapGraph(){ 
    m_Vertex.reserve(CPOBPRM_MPComponents::m_NumberOfProcessor); 
    for( int iV=0;iV<CPOBPRM_MPComponents::m_NumberOfProcessor;iV++ )
        m_Vertex.push_back(vector<Cfg>());
}

template<class VERTEX, class WEIGHT>
pRoadmapGraph<VERTEX,WEIGHT>::pRoadmapGraph(int _sz) 
:RoadmapGraph<VERTEX,WEIGHT>(_sz,DEFAULT_EDGES_PER_VERTEX){
        m_Vertex.reserve(CPOBPRM_MPComponents::m_NumberOfProcessor);
        for( int iV=0;iV<CPOBPRM_MPComponents::m_NumberOfProcessor;iV++ )
                m_Vertex.push_back(vector<Cfg>());
}

template<class VERTEX, class WEIGHT>
pRoadmapGraph<VERTEX,WEIGHT>::pRoadmapGraph(int _sz, int _edgelistsz)
:RoadmapGraph<VERTEX,WEIGHT>(_sz,_edgelistsz){
        m_Vertex.reserve(CPOBPRM_MPComponents::m_NumberOfProcessor);
        for( int iV=0;iV<CPOBPRM_MPComponents::m_NumberOfProcessor;iV++ )
                m_Vertex.push_back(vector<Cfg>());
}

// require that VERTEX data (configuration) is unique
template<class VERTEX, class WEIGHT>
VID pRoadmapGraph<VERTEX,WEIGHT>::AddVertex(VERTEX& _v1) {
    int PID = m_get_myid();
    m_Vertex[PID].push_back(_v1); 
    return PID;
}

// require that VERTEX data (configuration) is unique
template<class VERTEX, class WEIGHT>
VID pRoadmapGraph<VERTEX,WEIGHT>::AddVertex(vector<VERTEX>& _v) {
    int PID = m_get_myid();
    m_Vertex[PID].insert(m_Vertex[PID].begin(), _v.begin(),_v.end());
    return PID;
}

template<class VERTEX, class WEIGHT>
int pRoadmapGraph<VERTEX,WEIGHT>::AddEdges( vector<EdgeInfo<WEIGHT> >& _e) {
    int returnV;
    m_Lock.aquire_lock();
    returnV = RoadmapGraph<VERTEX,WEIGHT>::AddEdges(_e);
    m_Lock.release_lock();
    return returnV;
}

template<class VERTEX, class WEIGHT>
int pRoadmapGraph<VERTEX,WEIGHT>::AddEdge(VID vid1, VID vid2, WEIGHT w){
    int returnV;
    m_Lock.aquire_lock();
    if( !IsEdge(vid1, vid2) )
        returnV = RoadmapGraph<VERTEX,WEIGHT>::AddEdge(vid1, vid2, w);
    m_Lock.release_lock();
    return returnV;
}

template<class VERTEX, class WEIGHT>
int  pRoadmapGraph<VERTEX,WEIGHT>::AddEdge(VERTEX& v1, VERTEX& v2, WEIGHT w){
    int returnV;
    m_Lock.aquire_lock();
    if( !IsEdge(v1, v2) )
        returnV = RoadmapGraph<VERTEX,WEIGHT>::AddEdge(v1, v2, w);
    m_Lock.release_lock();
    return returnV;
}

template<class VERTEX, class WEIGHT>
int  pRoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VID v1, VID v2, pair<WEIGHT,WEIGHT> w){
    int returnV;
    m_Lock.aquire_lock();
    if( !IsEdge(v1, v2) )
        returnV = RoadmapGraph<VERTEX,WEIGHT>::AddEdge(v1, v2, w);
    m_Lock.release_lock();
    return returnV;
}

template<class VERTEX, class WEIGHT>
int  pRoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& v1, VERTEX& v2, pair<WEIGHT,WEIGHT> w){
    int returnV;
    m_Lock.aquire_lock();
    if( !IsEdge(v1, v2) )
        returnV = RoadmapGraph<VERTEX,WEIGHT>::AddEdge(v1, v2, w);
    m_Lock.release_lock();
    return returnV;
}

template<class VERTEX, class WEIGHT> void
pRoadmapGraph<VERTEX,WEIGHT>::FlushVertex(){
    RoadmapGraph<VERTEX,WEIGHT>::AddVertex(m_Vertex[m_get_myid()]);
}

template<class VERTEX, class WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT> 
pRoadmapGraph<VERTEX,WEIGHT>::BFS(VID vid) const{
    WeightedMultiDiGraph<VERTEX,WEIGHT> returnV;
    m_Lock.aquire_lock();
    returnV = RoadmapGraph<VERTEX,WEIGHT>::BFS(vid);
    m_Lock.release_lock();
    return returnV;

}

#endif

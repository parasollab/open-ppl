// $Id$
/////////////////////////////////////////////////////////////////////
//  Graph.h
//
//  General Description
//      This set of template classes provides an implementation for 
//      the graph abstract data type. 
//
//      The user must provide the parameter type VERTEX, which is
//      the data that will be stored in each vertex of the graph.
//      It is assumed the VERTEX class has the following operators
//      defined for it: << >> == = 
// 
//      In addition, for weighted graphs, the user must supply
//      the parameter type WEIGHT, which is the data that will
//      be stored in each edge of the graph.
//
//      The classes in this hierarchy (so far) include:
//        o AbstractGraph 
//          o WeightedMultiDiGraph (derived from AbstractGraph)
//            o WeightedGraph (derived from WeightedMultiDiGraph)
//
//      Graphs are represented by adjacency lists. 
//      The unique vertex IDs are typedef'ed short ints (which
//      should be suffient for most graphs).
//
// *IMPORTANT* 
//      For convenience, options have been included for dealing either with 
//      vertex ids (VIDs) or vertex data (VERTEX). The ones dealing with 
//      VERTEX data are only 'safe' if VERTEX data is unique.
//
//  Created
//      7/18/98  Nancy Amato
//  Last Modified By:
//      8/26/99  An Ping
//      8/31/99  Lucia K. Dale
//      11/22/99  Nancy Amato (add GetEdgesVData)
//      11/26/99  Nancy Amato (Dijkstra SSSP;BFS cleanup;Weight class;cleanup)
/////////////////////////////////////////////////////////////////////
#ifndef Graph_h
#define Graph_h

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <iostream.h>
#include <fstream.h>
#include <iomanip.h>

#include <algo.h>          //g++ in SUN
#include <list.h>          //g++ in SUN

#ifndef __HP_aCC
   //#include <algorithm>  //c++ in parasol
   //#include <list>       //c++ in parasol
   #include <vector.h>
   #include <deque.h>	
   #include <stack.h>
	
   #ifndef VID
   typedef short VID;
   #endif

#else		
   #include <iterator>
   #include <vector>       //aCC in parasol
   #include <deque>        //aCC in parasol
   #include <stack>        //aCC in parasol

   #ifndef VID
   typedef int VID;
   #endif

#endif


#include "BasicDefns.h"

#ifndef EID
 typedef short EID;
#endif

#ifndef INVALID_VID
#define INVALID_VID    -999
#endif

template<class VERTEX, class WEIGHT> class WtEdgeType;
template<class VERTEX, class WEIGHT> class WtVertexType;

/////////////////////////////////////////////////////////////////////
//  class WtEdgeType<VERTEX,WEIGHT>
//
//  General Description
//      Has friend class WtVertexType<VERTEX,WEIGHT>
//
//      WtEdge is the generic structure for edges in weighted graphs.
//      Its data includes:
//        --  the unique_id of the *second* endpoint of the edge (the 
//            first endpoint is implicit due to the adjacency list rep) 
//        --  the edge WEIGHT
//
//  Created
//      7/18/98  Nancy Amato
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
class WtEdgeType {   
	friend class WtVertexType<VERTEX,WEIGHT>;
public:
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
       WtEdgeType();
       WtEdgeType(VID, WEIGHT);
      ~WtEdgeType();

  //===================================================================
  //  Other Methods
  //===================================================================
       // Display, Input, Output
  void DisplayEdge() const;
  void WriteEdge(ostream&) const;

  //===================================================================
  //  Data
  //===================================================================
  VID     vertex2id; //start vertex (v1) is implicit in adj list rep
  WEIGHT  weight;
protected:
private:
};

/////////////////////////////////////////////////////////////////////
//  class WtVertexType<VERTEX,WEIGHT>
//
//  General Description
//      Is friend of class WtEdgeType<VERTEX,WEIGHT>
//
//      WtVertexType is the generic structure for vertices in weighted graphs.
//      Its data includes an edgelist for the vertex, and its methods 
//      are responsible for adding/removing edges and weights. 
//
//  Created
//      7/18/98  Nancy Amato
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
class WtVertexType {  
public:
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
       WtVertexType();
       WtVertexType(VERTEX&, VID);       // don't reserve space for edges
       WtVertexType(VERTEX&, VID, int);  // 'reserve' space for edges
      ~WtVertexType();

  //===================================================================
  //  Other Methods
  //===================================================================
       //  Adding & Deleting Edges 
  void AddEdge(VID, WEIGHT); 
  int  DeleteXEdges(VID, int); 
  int  DeleteXEdges(VID, WEIGHT, int); 

       // Finding Edges
  bool IsEdge(VID) const ; 
  bool IsEdge(VID, const WtEdge**) const;
  bool IsEdge(VID, WEIGHT, const WtEdge** _ei) const;

       // Getting Data & Statistics 
  VERTEX  GetData() const; 
  int  GetVID() const;
  int  GetEdgeCount() const;
  WEIGHT  GetEdgeWeight(VID) const; 

       // Display, Input, Output
  void DisplayEdgelist() const;
  void WriteEdgelist(ostream&) const;

  //===================================================================
  //  Utility Stuff
  //===================================================================
   typedef  typename vector< WtEdge >::iterator EI;
   typedef  typename vector< WtEdge >::const_iterator CEI;
   typedef  typename vector< WtEdge >::reverse_iterator REI;
   typedef  typename vector< WtEdge >::const_reverse_iterator CREI;

   typedef  typename vector< pair<VID, WEIGHT> >::iterator PEI;
   typedef  typename vector< pair<VID, WEIGHT> >::const_iterator CPEI;  
   typedef  typename vector< pair<VID, WEIGHT> >::reverse_iterator RPEI;
   typedef  typename vector< pair<VID, WEIGHT> >::const_reverse_iterator
CRPEI;

       // Predicates
  WtEdge* my_find_EID1_eq(const EID _eid) const; 
  WtEdge* my_find_EID2_eq(const WtEdge* _start, const WtEdge* _end, const EID _eid) const; 
  WtEdge* my_find_EWT_eq(const WEIGHT _wt) const; 
  WtEdge* my_find_EIDWT_eq(const pair<VID,WEIGHT> _wtpair) const; 
  // NMA: predicates below don't work with sgi/CC, but do for sun/g++
  //class EID_eq;
  //class EWT_eq;
  //class EIDWT_eq;

  //===================================================================
  //  Data
  //===================================================================
  VERTEX data; // e.g., cfg, data here...
  VID    vid;          // this vertex's unique id (not nec. index)
  vector< WtEdge > edgelist;   // adj list rep of graph
  vector< pair<VID, WEIGHT> > preedgelist;

protected:
private:
};


/////////////////////////////////////////////////////////////////////
//  class AbstractGraph<VERTEX>
//
//  General Description
//      Abstract base class for all graphs.
//      It is an abstract class (contains pure virtual functions)
//      and serves as the interface to the derived classes
//      representing various types of graphs (e.g., undirected,
//      directed, unweighted, weighted, or multigraphs).
//
//  Created
//      7/18/98  Nancy Amato
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////
template<class VERTEX> 
class AbstractGraph {
public:
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
   AbstractGraph();
   AbstractGraph(int); // number of edges per vertex ('reserve' space)
   ~AbstractGraph();

  //===================================================================
  //  Other Methods
  //===================================================================
        // Adding & Finding & Deleting Vertices
   virtual VID  AddVertex(VERTEX&) = 0;
   virtual VID  AddVertex(vector<VERTEX>&) = 0;
   virtual int  DeleteVertex(VID) = 0; 
   virtual int  DeleteVertex(VERTEX&) = 0; 
   virtual void DeleteAllEdges(VID) = 0; // delete all incident edges 

        // Finding Vertices & Edges
   virtual bool IsVertex(VID) const = 0;
   virtual bool IsEdge(VID, VID) const = 0; 
   virtual bool IsVertex(VERTEX&) const = 0;
   virtual bool IsEdge(VERTEX&, VERTEX&) const  = 0; 

        // Getting Data & Statistics 
   int  GetVertexCount() const;
   int  GetEdgeCount() const;
   VID  GetNextVID() const;

        // Display, Input, Output 
   virtual void DisplayGraph() const = 0; 
   virtual void DisplayVertexAndEdgelist(VID) const = 0; 

//protected:
  //===================================================================
  //  Data
  //===================================================================
   VID vertIDs;  // used to give each vertex unique identifier     
   int numVerts;       
   int numEdges;       
   int reserveEdgesPerVertex; // used to 'reserve' space in edgelist

private:
};

/////////////////////////////////////////////////////////////////////
//  class WeightedMultiDiGraph<VERTEX,WEIGHT>
//
//  General Description
//      Derived from AbstractGraph<VERTEX,WEIGHT>.
//
//      The WeightedMultiDiGraph class:
//      -- is a *directed* weighted graph
//      -- allows multiple vertices with the same VERTEX data
//      -- allows multiple (v1id,v2id) edges with *different* WEIGHTs
//      -- allows multiple (v1id,v2id) edges with *same* WEIGHT
//
//      The graph is represented by an adjacency list structure.
//
//  Created
//      7/18/98  Nancy Amato
//  Last Modified By:
//      1/15/98  Nancy Amato
/////////////////////////////////////////////////////////////////////

template<class VERTEX, class WEIGHT>
class WeightedMultiDiGraph : public AbstractGraph<VERTEX> {
public:
  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

  //===================================================================
  //  Constructors and Destructor
  //===================================================================
   WeightedMultiDiGraph();          // don't reserve any space
   WeightedMultiDiGraph(int);       // 'reserve' space for vertices
   WeightedMultiDiGraph(int,int);   // 'reserve' space for verts & edges
   ~WeightedMultiDiGraph();

  //===================================================================
  //  Other Methods
  //===================================================================
        // Adding & Deleting Vertices
   virtual VID  AddVertex(VERTEX&);
   virtual VID  AddVertex(vector<VERTEX>&);
   int  DeleteVertex(VID);
   int  DeleteVertex(VERTEX&);
   int  EraseGraph(); 

        // Adding & Deleting Edges
   virtual int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>);
   virtual int  AddEdge(VID, VID, WEIGHT);
   virtual int  AddPath( vector<VID>&, WEIGHT); // all edges same weight
   virtual int  AddPath( vector< pair<VID,WEIGHT> >& ); 
   virtual int  AddPath( vector< pair<VID, pair<WEIGHT,WEIGHT> > >& );
   virtual int  DeleteEdge(VID, VID, int n=-1); // default, delete all
   virtual int  DeleteWtEdge(VID, VID, WEIGHT, int n=-1); // default, delete all
   void DeleteAllEdges(VID); 


   virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>);
   virtual int  AddEdge(VERTEX&, VERTEX&, WEIGHT);  
   virtual int  AddPath( vector<VERTEX>&, WEIGHT); // all edges same weight
   virtual int  AddPath( vector< pair<VERTEX,WEIGHT> >& ); 
   virtual int  AddPath( vector< pair<VERTEX, pair<WEIGHT,WEIGHT> > >& );
   virtual int  DeleteEdge(VERTEX&,VERTEX&, int n=-1); //default, delete all
   virtual int  DeleteWtEdge(VERTEX&,VERTEX&,WEIGHT, int n=-1); //default, delete all
   void DeleteAllEdges(VERTEX&);                       

        // Modifying Vertices
  void PutData(VID, VERTEX);

        // Finding Vertices & Edges
   bool IsVertex(VID) const;
   bool IsEdge(VID, VID) const;
   bool IsEdge(VID, VID, WEIGHT) const;

   bool IsVertex(VERTEX&) const;
   bool IsEdge(VERTEX&, VERTEX&) const;
   bool IsEdge(VERTEX&, VERTEX&, WEIGHT) const;

        // Getting Data & Statistics 
                // global information
   vector<VID> GetVerticesVID() const;
   vector<VID> GetVerticesVID(VID _vid,int n) const; // get n verts starting with VID _vid
   vector<VERTEX> GetVerticesData() const;
   vector<VERTEX> GetVerticesData(VID _vid,int n) const; // get n verts starting with VID _vid
   virtual vector< pair<pair<VID,VID>, WEIGHT> > GetEdges() const;
   virtual vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetEdgesVData() const;
                // vertex information
   inline VERTEX  GetData(VID) const;
   vector<VERTEX>  GetData(VID _v1id, VID _v2id) const;
   VID     GetVID(VERTEX&) const;
   WEIGHT  GetEdgeWeight(VID, VID) const;
   WEIGHT  GetEdgeWeight(VERTEX&, VERTEX&) const;

   virtual int GetVertexOutDegree(VID) const;
   //virtual int GetVertexInDegree(VID) const;
   //virtual int GetVertexDegree(VID) const;
   virtual vector<VID> GetSuccessors(VID) const;
   virtual vector<VID> GetSuccessors(VERTEX&) const;
   void GetPredecessorVector(); // NMA:::: THIS NEEDS TO GO!
   //virtual vector<VID> GetPredecessors(VID) const;
   //virtual vector<VID> GetPredecessors(VERTEX&) const;
   //virtual vector< pair< pair<VID,VID>, WEIGHT> > GetIncidentEdges(VID) const;
   //virtual vector< pair< pair<VID,VID>, WEIGHT> > GetIncomingEdges(VID) const;
   //virtual vector< pair< pair<VID,VID>, WEIGHT> > GetOutgoingEdges(VID) const;

       // Basic Graph Algorithms
   WeightedMultiDiGraph<VERTEX,WEIGHT> BFS(VID) const; 
   WeightedMultiDiGraph<VERTEX,WEIGHT> BFS(VERTEX&) const; 
   vector<VID> BFSVID(VID) const; 
   vector<VID> BFSVID(VERTEX&) const; 
   vector< pair<VERTEX,WEIGHT> > FindPathBFS(VID,VID) const;
   vector< pair<VERTEX,WEIGHT> > FindPathBFS(VERTEX&,VERTEX&) const;

   WeightedMultiDiGraph<VERTEX,WEIGHT> DijkstraSSSP(VID) const; //wts=pathlength
   vector< pair<VERTEX,WEIGHT> >  FindPathDijkstra(VID,VID) const; //wts=ewts
   vector< pair<VERTEX,WEIGHT> >  FindPathDijkstra(VERTEX&,VERTEX&) const; 

   vector< VID >  DFS () const; 
   deque < VID >  TopologicalSort ();
   deque < VID >  CycleDetect ();
   void DfsTpsCyc(int, int, vector<VID>&, deque<VID>&, deque<VID>&);

        // Display, Input, Output 
   void DisplayGraph() const; 
   void DisplayVertices() const; 
   void DisplayVertex(VID) const; 
   void DisplayVertexAndEdgelist(VID) const; 
   void WriteGraph(ostream& _myostream) const;
   void WriteGraph(const char*  _filename) const; 
   void ReadGraph(istream& _myistream);
   void ReadGraph(const char*  _filename);

protected:
  //===================================================================
  //  Utility Stuff for WeightedMultiDiGraphs
  //===================================================================
   typedef  typename vector< Vertex >::iterator VI;
   typedef  typename vector< Vertex >::const_iterator CVI;
   typedef  typename vector< Vertex >::reverse_iterator RVI;
   typedef  typename vector< Vertex >::const_reverse_iterator CRVI;
   typedef  typename vector< WtEdge >::iterator EI;
   typedef  typename vector< WtEdge >::const_iterator CEI;
   typedef  typename vector< WtEdge >::reverse_iterator REI;
   typedef  typename vector< WtEdge >::const_reverse_iterator CREI;


        // Adding & Deleting Vertices
   VID  AddVertex(VERTEX&,VID);
   virtual int AddEdge(VID, EI);
   virtual void DeleteAllEdgesToV(VID); 
   virtual void DeleteAllEdgesFromV(VID); 
   virtual void DeleteAllEdgesToV(VERTEX&); 
   virtual void DeleteAllEdgesFromV(VERTEX&); 

        // Finding Vertices & Edges 
   bool IsVertex(VID, const Vertex**) const;
   bool IsEdge(VID, VID, const Vertex**, const WtEdge**) const;
   bool IsEdge(VID, VID, WEIGHT, const Vertex**, const WtEdge**) const;
   bool IsVertex(VERTEX&, const Vertex**) const;
   bool IsEdge(VERTEX&, VERTEX&, const Vertex**, const WtEdge**) const;
   bool IsEdge(VERTEX&, VERTEX&, WEIGHT, const Vertex**, const WtEdge**) const;

        // Predicates, Comparisons, & Operations 
   Vertex* my_find_VID_eq(const VID _vid) const;
   Vertex* my_find_VDATA_eq(const VERTEX& _v) const;
   static bool VID_Compare (const Vertex& _v1, const Vertex& _v2); 
   class dkinfo;   // for Dijkstra's algorithm
   static bool dkinfo_Compare (const dkinfo& _d1, const dkinfo& _d2); 

   //NMA: the following predictates work with stl/sun/g++ but not stl/sgi/CC
   //class VID_eq; 
   //class VDATA_eq;
   //class VID_Compare;


  //===================================================================
  //  Data
  //===================================================================
	vector< Vertex >	v;	// vertices (with adj lists)

private:
};

/////////////////////////////////////////////////////////////////////
//  class WeightedGraph<VERTEX,WEIGHT>
//
//  General Description
//      Derived from WeightedMultiDiGraph<VERTEX,WEIGHT>.
//
//      The WeightedGraph class:
//      -- is an *undirected* weighted graph  (but can have different
//         weights on the 'forward' and 'back' edges).
//      -- allows multiple vertices with the same VERTEX data 
//      -- does not allow multiple (v1id,v2id) edges (even w/ different WEIGHTs)
//
//      The graph is represented by an adjacency list structure.
//
//  Created
//      7/18/98  Nancy Amato
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
class WeightedGraph : public WeightedMultiDiGraph<VERTEX,WEIGHT> {
public:

  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

  //===================================================================
  //  Constructors and Destructor
  //===================================================================
   WeightedGraph();
   WeightedGraph(int);       // 'reserve' space for vertices
   WeightedGraph(int,int);   // 'reserve' space for verts & edges
//   WeightedGraph(WeightedMultiDiGraph<VERTEX,WEIGHT>); // construct from base  
   ~WeightedGraph();

  //===================================================================
  //  Other Methods
  //===================================================================
        // Adding & Deleting Edges
   virtual int  AddEdge(VID, VID, WEIGHT);
   virtual int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>);
   virtual int  DeleteEdge(VID, VID, int _n=-1);
   virtual int  DeleteWtEdge(VID, VID, WEIGHT, int _n=-1);
   virtual int  ChangeEdgeWeight(VID, VID, WEIGHT);

   virtual int  AddEdge(VERTEX&, VERTEX&, WEIGHT);
   virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>);
   virtual int  DeleteEdge(VERTEX&, VERTEX&, int _n=-1);
   virtual int  DeleteWtEdge(VERTEX&, VERTEX&, WEIGHT, int _n=-1);
   virtual int  ChangeEdgeWeight(VERTEX&, VERTEX&, WEIGHT);

       // Getting Data & Statistics 
                // global information
   virtual vector< pair<pair<VID,VID>, WEIGHT> > GetEdges() const;
   virtual vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetEdgesVData() const;
                // vertex information
   virtual int     GetVertexDegree(VID) const;
   virtual vector< VID > GetAdjacentVertices(VID) const;
   virtual vector< pair<pair<VID,VID>,WEIGHT> > GetIncidentEdges(VID) const;
   virtual vector< pair<pair<VERTEX,VERTEX>,WEIGHT> > GetIncidentEdgesVData(VID) const;

       // Connected Components Utilities
   bool IsSameCC(VID,VID) const;
   bool IsSameCC(VERTEX&,VERTEX&) const;
   vector<VID> GetCC(VID) const;
   vector<VERTEX> GetCC(VERTEX&) const;
   vector< pair<pair<VID,VID>, WEIGHT> > GetCCEdges(VID) const;
   vector< pair<pair<VID,VID>, WEIGHT> > GetCCEdges(VERTEX&) const;
   vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetCCEdgesVData(VID) const;
   vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetCCEdgesVData(VERTEX&) const;
   vector< vector< pair<VERTEX,VERTEX> > > GetEdgesByCCVDataOnly() const; 
   vector< pair<int,VID> > GetCCStats() const;
   int GetCCcount() const;

        // Display, Input, Output 
   void DisplayCC(VID) const;
   void DisplayEdgesByCCVDataOnly() const; 
   void DisplayCCStats(int _numCCtoPrint = -1) const; // default print all 

        // Predicates, Comparisons, & Operations 
   static bool CCVID_Compare (const pair<int,VID>& _cc1, const pair<int,VID>& _cc2); 

  //===================================================================
  //  Utility Stuff for WeightedGraph
  //===================================================================
 
   typedef  typename vector< Vertex >::iterator VI;
   typedef  typename vector< Vertex >::const_iterator CVI;
   typedef  typename vector< Vertex >::reverse_iterator RVI;
   typedef  typename vector< Vertex >::const_reverse_iterator CRVI;
   typedef  typename vector< WtEdge >::iterator EI;
   typedef  typename vector< WtEdge >::const_iterator CEI;
   typedef  typename vector< WtEdge >::reverse_iterator REI;
   typedef  typename vector< WtEdge >::const_reverse_iterator CREI;


protected:
   virtual int  AddEdge(VID, EI);
private:
  //===================================================================
  //  Data
  //===================================================================
};

//=====================================================================
//=====================================================================
//  METHODS FOR GRAPH RELATED CLASSES
//=====================================================================
//=====================================================================

//=====================================================================
// METHODS FOR template AbstractGraph Class
//=====================================================================
  //==================================
  // AbstractGraph class Methods: Constructors and Destructor
  //==================================
template<class VERTEX> 
AbstractGraph<VERTEX>::
AbstractGraph() {
      vertIDs=numVerts=numEdges=0;
      reserveEdgesPerVertex = 0;
};

template<class VERTEX> 
AbstractGraph<VERTEX>::
AbstractGraph(int _reserveEdgesPerVertex) {
      vertIDs=numVerts=numEdges=0;
      reserveEdgesPerVertex = _reserveEdgesPerVertex;
};

template<class VERTEX> 
AbstractGraph<VERTEX>::
~AbstractGraph() {
};

  //==================================
  // AbstractGraph class Methods: Statistics -- num verts/edges, etc 
  //==================================

template<class VERTEX> 
int 
AbstractGraph<VERTEX>:: 
GetVertexCount() const {
      return numVerts;
};

template<class VERTEX> 
int 
AbstractGraph<VERTEX>:: 
GetEdgeCount() const {
      return numEdges;
};

template<class VERTEX>
VID
AbstractGraph<VERTEX>::
GetNextVID() const {
      return vertIDs;
};


//=====================================================================
// METHODS FOR template WeightedMultiDiGraph Class
//=====================================================================
  //==================================
  // WeightedMultiDiGraph class Methods: Constructors and Destructor
  //==================================

template<class VERTEX,class WEIGHT> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
WeightedMultiDiGraph(){
};

template<class VERTEX,class WEIGHT> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
WeightedMultiDiGraph(int _sz) {
  v.reserve(_sz);
};

template<class VERTEX,class WEIGHT> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
WeightedMultiDiGraph(int _sz, int _edgelistsz)
 : AbstractGraph<VERTEX> (_edgelistsz) 
{
  v.reserve(_sz);
};

template<class VERTEX, class WEIGHT> 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
~WeightedMultiDiGraph(){
};


  //==================================
  // WeightedMultiDiGraph class Methods: Adding & Deleting Vertices
  //==================================

template<class VERTEX, class WEIGHT> 
VID 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddVertex(VERTEX& _v) {
      VID vid = vertIDs++;
      Vertex newVertex(_v,vid,reserveEdgesPerVertex);
		v.push_back(newVertex);
      numVerts++;
      return (vid); // return vertex id (not nec. index)
};

template<class VERTEX, class WEIGHT>
VID
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddVertex(vector<VERTEX>& _v) {

     if (_v.size()>0) {
       VID vertex_id = AddVertex(_v[0]);
       for (int i=1;i<_v.size();++i)
       AddVertex(_v[i]);
       return (vertex_id); // return vertex id (not nec. index)
      }
      return INVALID_VID;

};

template<class VERTEX, class WEIGHT> 
VID 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddVertex(VERTEX& _v, VID _vid) {
      VID vid = _vid;
      Vertex newVertex(_v,vid,reserveEdgesPerVertex);
      v.push_back(newVertex);
      numVerts++;
      return (vid); // return vertex id (not nec. index)
};


template<class VERTEX, class WEIGHT> 
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DeleteVertex(VERTEX& _v1) {
     CVI cv1;
     VI v1;
     if ( IsVertex(_v1,&cv1) ) { 
         v1 = const_cast<VI>(cv1);
         DeleteAllEdgesToV(v1->vid);
         v.erase(v1);
         numVerts--;
         return OK;
     } else {
         cout << "\nDeleteVertex: vertex not in graph";
         return ERROR;
     };
};

template<class VERTEX, class WEIGHT>
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteVertex(VID _v1id) {
     CVI cv1;
     VI v1;
     if ( IsVertex(_v1id,&cv1) ) {
         v1 = const_cast<VI>(cv1);
         DeleteAllEdgesToV(_v1id);
         v.erase(v1);
         numVerts--;
         return OK;
     } else {
         cout << "\nDeleteVertex: vertex not in graph";
         return ERROR;
     };
};

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
EraseGraph() {
     while ( v.size() != 0 ) 
         v.pop_back();
     vertIDs = numVerts = numEdges = 0;
     return OK;
};


  //==================================
  // WeightedMultiDiGraph class Methods: Modifying Vertices
  //==================================

template<class VERTEX, class WEIGHT>
void
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
PutData(VID _vid, VERTEX _v){ 
     CVI cv1;
     VI v1;
     if ( IsVertex(_vid,&cv1) ) {
         v1 = const_cast<VI>(cv1);
         v1->data = _v;
     }
};


  //==================================
  // WeightedMultiDiGraph class Methods: Adding & Deleting Edges
  //==================================

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, EI _ei) {
    VI v1, v2;
    CVI cv1, cv2;
    VID v2id = _ei->vertex2id;
    WEIGHT weight = _ei->weight;

    if (IsVertex(_v1id,&cv1) && IsVertex(v2id,&cv2) ) {
	v1 = const_cast<VI> (cv1);
         v1->AddEdge(v2id,weight);
         numEdges++;
         return OK;
      } else {
         cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
         return ERROR;
     };
};


template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, VID _v2id, WEIGHT _weight) {
    CVI cv1;
    VI v1;
    if (IsVertex(_v1id,&cv1) && IsVertex(_v2id) ) {
         v1 = const_cast<VI>(cv1);
         v1->AddEdge(_v2id,_weight);
         numEdges++;
         return OK;
     } else {
         cout << endl << "AddEdge: v1 " << _v1id << " and/or v2 " << _v2id << "not in graph" ;
         return ERROR;
     };
};


template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, VID _v2id, pair<WEIGHT,WEIGHT> _wtpair ) {
    CVI cv1,cv2;
    VI v1,v2;
    if (IsVertex(_v1id,&cv1) && IsVertex(_v2id,&cv2) ) {
         v1 = const_cast<VI>(cv1);
         v2 = const_cast<VI>(cv2);
         v1->AddEdge(_v2id,_wtpair.first);
         v2->AddEdge(_v1id,_wtpair.second);
         numEdges += 2;
         return OK;
     } else {
         cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
         return ERROR;
     };
};

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    CVI cv1, cv2;
    VI v1, v2;
    if (IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
         v1 = const_cast<VI>(cv1);
         v2 = const_cast<VI>(cv2);
         v1->AddEdge(v2->vid,_weight);
         numEdges++;
         return OK;
      } else {
         cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
         return ERROR;
     };
};

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT> _wtpair) {
    CVI cv1, cv2;
    VI v1, v2;
    if ( IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
         v1 = const_cast<VI>(cv1);
         v2 = const_cast<VI>(cv2);
         v1->AddEdge(v2->vid,_wtpair.first);
         v2->AddEdge(v1->vid,_wtpair.second);
         numEdges += 2;
         return OK;
      } else {
         cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
         return ERROR;
     };
};

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector<VID>& _path, WEIGHT _wt) {
    int i;
    for (i = 0; i < _path.size(); i++){
       if (!IsVertex(_path[i])) return ERROR;
    }
    for (i = 0; i < _path.size() - 1; i++){
      AddEdge(_path[i],_path[i+1],_wt);
    }
    return OK;
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector<VERTEX>& _path, WEIGHT _wt) {
    if (!IsVertex(_path[0])) AddVertex(_path[0]);
    for (int i = 0; i < _path.size() - 1; i++){
      if (!IsVertex(_path[i+1])) AddVertex(_path[i+1]);
      AddEdge(_path[i],_path[i+1],_wt);
    }
    return OK;
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector< pair<VID,WEIGHT> >& _path) {
    int i;
    for (i = 0; i < _path.size(); i++){
       if (!IsVertex(_path[i].first)) return ERROR;
    }
    for (i = 0; i < _path.size() - 1; i++){
      AddEdge(_path[i].first,_path[i+1].first,_path[i].second);
    }
    return OK;
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector< pair<VERTEX,WEIGHT> >& _path) {
    if (!IsVertex(_path[0].first)) AddVertex(_path[0].first);
    for (int i = 0; i < _path.size() - 1; i++){
      if (!IsVertex(_path[i+1].first)) AddVertex(_path[i+1].first);
      AddEdge(_path[i].first,_path[i+1].first,_path[i].second);
    }
    return OK;
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector< pair<VID, pair<WEIGHT,WEIGHT> > >& _path) {
    int i;
    for (i = 0; i < _path.size(); i++){
       if (!IsVertex(_path[i].first)) return ERROR;
    }
    for (i = 0; i < _path.size() - 1; i++){
      AddEdge(_path[i].first,_path[i+1].first,_path[i].second);
    }
    return OK;
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector< pair<VERTEX, pair<WEIGHT,WEIGHT> > >& _path) {
    if (!IsVertex(_path[0].first)) AddVertex(_path[0].first);
    for (int i = 0; i < _path.size() - 1; i++){
      if (!IsVertex(_path[i+1].first)) AddVertex(_path[i+1].first);
      AddEdge(_path[i].first,_path[i+1].first,_path[i].second);
    }
    return OK;
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesToV(VID _v2id) {
     CVI v2;
     if ( IsVertex(_v2id,&v2) ) {
        for (VI vi = v.begin(); vi < v.end(); vi++) {
            numEdges -= vi->DeleteXEdges(_v2id,-1);
        }
     }
};

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesFromV(VID _v1id) {
     CVI cv1;
     VI v1;
     if ( IsVertex(_v1id,&cv1) ) {
        v1 = const_cast<VI>(cv1);
        numEdges -= v1->edgelist.size();
        v1->edgelist.erase( v1->edgelist.begin(), v1->edgelist.end() );
     }
};

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdges(VID _vid) {
     DeleteAllEdgesToV(_vid);
     DeleteAllEdgesFromV(_vid);
};

// default: delete all edges (v1,v2), otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteEdge(VID _v1id, VID _v2id, int _n) {
     VI v1;
     CVI cv1;
     if ( IsVertex(_v1id,&cv1) ) {
	v1 = const_cast<VI> (cv1);
         numEdges -= v1->DeleteXEdges(_v2id,_n);
         return OK;
     } else {
         return ERROR;
     };
};

// default: delete all edges (v1,v2) of specified weight, otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteWtEdge(VID _v1id, VID _v2id, WEIGHT _weight, int _n) {
     VI v1;
     CVI cv1;
     if ( IsVertex(_v1id,&cv1) ) {
	v1 = const_cast<VI> (cv1);
         numEdges -= v1->DeleteXEdges(_v2id,_weight,_n);
         return OK;
     } else {
         return ERROR;
     };
};

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesToV(VERTEX& _v2) {
     CVI v2;
     if ( IsVertex(_v2,&v2) ) {
        for (VI vi = v.begin(); vi < v.end(); vi++) {
            numEdges -= vi->DeleteXEdges(v2->vid,-1);
        }
     }
};

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesFromV(VERTEX& _v1) {
     CVI cv1;
     VI v1;
     if ( IsVertex(_v1,&cv1) ) {
        v1 = const_cast<VI>(cv1); 
        numEdges -= v1->edgelist.size();
        v1->edgelist.erase( v1->edgelist.begin(), v1->edgelist.end() );
     }
};

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdges(VERTEX& _v) {
     DeleteAllEdgesToV(_v);
     DeleteAllEdgesFromV(_v);
};


// default: delete all edges (v1,v2), otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteEdge(VERTEX& _v1, VERTEX& _v2, int _n) {
     VI v1,v2;
     CVI cv1, cv2;
     if ( IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
	v1= const_cast<VI> (cv1);
	v2= const_cast<VI> (cv2);
         numEdges -= v1->DeleteXEdges(v2->vid,_n);
         return OK;
     } else {
         return ERROR;
     };
};


// default: delete all edges (v1,v2) of specified weight, otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteWtEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight, int _n) {
     VI v1,v2;
     CVI cv1, cv2;
     if ( IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
	 v1= const_cast<VI> (cv1);
  	 v2= const_cast<VI> (cv2);
         numEdges -= v1->DeleteXEdges(v2->vid,_weight,_n);
         return OK;
     } else {
         return ERROR;
     };
};


  //==================================
  // WeightedMultiDiGraph class Methods: Finding Vertices & Edges
  //==================================
 
template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsVertex(VID _v1id) const {
     CVI v1;
     return ( IsVertex(_v1id,&v1) );
};

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsVertex(VID _v1id, const Vertex**  _v1ptr) const {

        CVI v1 = my_find_VID_eq(_v1id);
        if (v1 != v.end() ) {
            *_v1ptr = v1;
            return true;
        } else {
            return false;
        }
};

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsVertex(VERTEX& _v1) const {
     CVI v1;
     return ( IsVertex(_v1,&v1) );
};


template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsVertex(VERTEX& _v1, const Vertex**  _v1ptr) const {

     //CVI v1 = find_if(v.begin(), v.end(), VDATA_eq(_v1) );
     CVI v1 = my_find_VDATA_eq(_v1);
     if (v1 != v.end() ) {
         *_v1ptr = v1;
         return true;
     } else {
         return false;
     };
};

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id) const {
     CVI v1;
     CEI e12;
     return ( IsEdge(_v1id,_v2id,&v1,&e12) );
};

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id, WEIGHT _weight) const {
     CVI v1;
     CEI e12;
     return ( IsEdge(_v1id,_v2id,_weight,&v1,&e12) );
};

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id, const Vertex** _v1ptr, const WtEdge** _e12ptr) const {
     CVI v1;
     CEI e12;
     if ( IsVertex(_v1id,&v1) ) {
         if (v1->IsEdge(_v2id,&e12)) {
            *_v1ptr = v1;
            *_e12ptr = e12;
            return true;
         } else {
            return false;
         }
     } else {
         return false;
     };
};

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id, WEIGHT _weight, const Vertex** _v1ptr, const WtEdge** _e12ptr)  const {
     CVI v1;
     CEI e12;
     if ( IsVertex(_v1id,&v1) ) {
         if (v1->IsEdge(_v2id,_weight,&e12)) {
            *_v1ptr = v1;
            *_e12ptr = e12;
            return true;
         } else {
            return false;
         }
     } else {
         return false;
     };
};

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2) const {
     CVI v1;
     CEI e12;
     return ( IsEdge(_v1,_v2,&v1,&e12) );
};

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) const {
     CVI v1;
     CEI e12;
     return ( IsEdge(_v1,_v2,_weight,&v1,&e12) );
};


template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2, const Vertex** _v1ptr, const WtEdge** _e12ptr) const {
     CVI v1,v2;
     CEI e12;
     if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
         if (v1->IsEdge(v2->vid,&e12)) {
            *_v1ptr = v1;
            *_e12ptr = e12;
            return true;
         } else {
            return false;
         }
     } else {
         return false;
     };
};

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight, const Vertex** _v1ptr, const WtEdge** _e12ptr) const {
     CVI v1,v2;
     CEI e12;
     if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
         if (v1->IsEdge(v2->vid,_weight,&e12)) {
            *_v1ptr = v1;
            *_e12ptr = e12;
            return true;
         } else {
            return false;
         }
     } else {
         return false;
     };
};

  //==================================
  // WeightedMultiDiGraph class Methods: Getting Data & Statistics
  //==================================

template<class VERTEX, class WEIGHT>
vector<VID> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVerticesVID() const {
  vector<VID> verts;
  verts.reserve( v.size() );
  for (CVI vi = v.begin(); vi < v.end(); vi++) {
      verts.push_back(vi->vid);
  }
  sort( verts.begin(),verts.end() );
  return verts;
};

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVerticesVID(VID _vid, int _n) const {
  vector<VID> verts;
  verts.reserve( _n );
  CVI v1, v2;
  int i;
  if ( IsVertex(_vid,&v1) ) {
      for (i = 0, v2 = v1; i < _n && v2 < v.end(); i++, v2++) {
          verts.push_back(v2->vid);
      }
  } else {
      cout << "\nIn GetVerticesVID(VID,int): no vertex VID=" << _vid << " in graph\n";
  }
  sort( verts.begin(),verts.end() );
  return verts;
};

template<class VERTEX, class WEIGHT>
vector<VERTEX> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVerticesData() const {
  vector<VERTEX> verts;
  verts.reserve( v.size() );
  for (CVI vi = v.begin(); vi < v.end(); vi++) {
      verts.push_back(vi->data);
  }
  return verts;
};

template<class VERTEX, class WEIGHT>
vector<VERTEX>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVerticesData(VID _vid, int _n) const {
  vector<VERTEX> verts;
  verts.reserve( _n );
  CVI v1, v2;
  int i;
  if ( IsVertex(_vid,&v1) ) {
      for (i = 0, v2 = v1; i < _n && v2 < v.end(); i++, v2++) {
          verts.push_back(v2->data);
      }
  } else {
      cout << "\nIn GetVerticesData(VID,int): no vertex VID=" << _vid << " in graph\n";
  }
  return verts;
};


template<class VERTEX, class WEIGHT>
vector< pair< pair<VID,VID>, WEIGHT> >
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetEdges() const {
  vector< pair< pair<VID,VID>, WEIGHT> > edges;

  edges.reserve(numEdges);
  for (CVI vi = v.begin(); vi != v.end(); vi++) {
     for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
        pair<VID,VID> newedge(vi->vid, ei->vertex2id);
        pair<pair<VID,VID>,WEIGHT> newedgewt(newedge, ei->weight);
        edges.push_back( newedgewt );
     }
  }
  return edges;
};

template<class VERTEX, class WEIGHT>
vector< pair< pair<VERTEX,VERTEX>, WEIGHT> >
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetEdgesVData() const {
  vector< pair< pair<VERTEX,VERTEX>, WEIGHT> > edges;

  edges.reserve(numEdges);
  for (CVI vi = v.begin(); vi != v.end(); vi++) {
     for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
        VERTEX v2data = GetData(ei->vertex2id);
        pair<VERTEX,VERTEX> newedge(vi->data, v2data);
        pair<pair<VERTEX,VERTEX>,WEIGHT> newedgewt(newedge, ei->weight);
        edges.push_back( newedgewt );
     }
  }
  return edges;
};

template<class VERTEX, class WEIGHT>
VERTEX
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetData(VID _v1id) const {
  CVI v1;
  if ( IsVertex(_v1id,&v1) ) {
     return v1->data;
  } else {
     return VERTEX::InvalidData(); 
  }
};

template<class VERTEX, class WEIGHT>
vector<VERTEX>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetData(VID _v1id, VID _v2id) const {
  CVI v1, v2;
  vector<VERTEX> vset;

  if ( IsVertex(_v1id,&v1) && IsVertex(_v2id,&v2) ) {
     for (VID i = _v1id; i <= _v2id; i++) { 
       vset.push_back(v1->data);
       v1++;
     }
     return vset;
  } else {
     return vset; //in this case return an empty vector
  }
};


template<class VERTEX, class WEIGHT>
VID
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVID(VERTEX& _v1) const {
  CVI v1;
  if ( IsVertex(_v1,&v1) ) {
     return v1->vid;
  } else {
     return INVALID_VID;
  }
};

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVertexOutDegree(VID _v1) const {
  CVI v1;
  if ( IsVertex(_v1,&v1) ) {
     return v1->edgelist.size();
  } else {
     return ERROR;
  }
};
  
template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetSuccessors(VID _v1id) const {
     vector<VID> succ;
     CVI v1;
     if ( IsVertex(_v1id,&v1) ) {
         succ.reserve( v1->edgelist.size() );
         for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
            succ.push_back(ei->vertex2id);
         }
     } else {
         cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
     };
     return succ;
};

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetSuccessors(VERTEX& _v1) const {
  return GetSuccessors( GetVID(_v1) );
};

template<class VERTEX, class WEIGHT>
WEIGHT 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
GetEdgeWeight(VID _v1id, VID _v2id) const {
  CVI v1;
  CEI e12;
  if (IsEdge(_v1id,_v2id,&v1,&e12)) {
     return  e12->weight;
  } else {
     return WEIGHT::InvalidWeight();
  }
};

template<class VERTEX, class WEIGHT>
WEIGHT 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
GetEdgeWeight(VERTEX& _v1, VERTEX& _v2) const {

	return GetEdgeWeight( GetVID(_v1), GetVID(_v2) );
}


template<class VERTEX, class WEIGHT>
void
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
GetPredecessorVector() {
        VI v1, v2;
	CVI cv1, cv2;
        VID _v2id;

        for(v1 = v.begin(); v1 < v.end(); v1++) {
             for (EI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
                pair<VID,WEIGHT> newedge(v1->vid, ei->weight);
                _v2id = ei->vertex2id;
                if ( IsVertex(_v2id, &cv2) ) {
			v2 = const_cast<VI> (cv2);
                        v2->preedgelist.push_back( newedge );
                }
             }
        }
};

  //==================================
  // WeightedMultiDiGraph class Methods: Basic Graph Algorithms
  //==================================

//*************************************************************** 
//  BREADTH-FIRST-SEARCH ALGORITHMS
//*************************************************************** 

template<class VERTEX, class WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
BFS (VERTEX& _startV) const {
  CVI cv1;
  if ( IsVertex(_startV,&cv1) ) {
     return BFS(cv1->vid);
  } else {
     cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
     WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree;
     return bfstree; 
  }
};


template<class VERTEX, class WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
BFS (VID _startVid) const {
  WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree;
  list<VID> q; 
  CVI cv1,cv2;
  VI v1,v2;
  VID v1id, v2id; 

  if ( IsVertex(_startVid,&cv1) ) {
     q.push_back(_startVid);
     v1 = const_cast<VI>(cv1);
     bfstree.AddVertex(v1->data,_startVid); 
  } else {
     cout << "\nIn GraphBFS: root vid=" << _startVid << " not in graph";
     return bfstree; 
  }

  while ( !q.empty() ) {
     v1id = q.front();
     if ( IsVertex(v1id,&cv1) ) {
       for (CEI e = cv1->edgelist.begin(); e < cv1->edgelist.end(); e++) {
         v2id = e->vertex2id;
         if ( !bfstree.IsVertex(v2id) && IsVertex(v2id,&cv2) ) { 
            q.push_back(v2id);
            v2 = const_cast<VI>(cv2);
            bfstree.AddVertex(v2->data,v2id); 
            if ( bfstree.AddEdge(v1id,v2id,e->weight) != OK) {
                cout << "\nIn GraphBFS: OOPS! edge not added right...";
            }
         }
       }
     } else {   
       cout << "\nIn GraphBFS: OOPS! vertex=" << v1id << " not in graph";
     }
     q.pop_front();
  }
  bfstree.vertIDs = vertIDs; // set the same vert ID as in graph
  return bfstree;
};


template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
BFSVID (VERTEX& _startV) const {
   WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = BFS(_startV); 
   return bfstree.GetVerticesVID();
};

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
BFSVID (VID _startVID) const {
   WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = BFS(_startVID); 
   return bfstree.GetVerticesVID();
};

//==============================
//  FindPathBFS (for any graph)
//  -- returns BFS path between 2 specified vertices 
//==============================
template<class VERTEX, class WEIGHT>
vector< pair<VERTEX,WEIGHT> >
WeightedMultiDiGraph<VERTEX,WEIGHT>::
FindPathBFS (VERTEX& _startV, VERTEX& _endV) const {

  CVI cv1,cv2;
  if ( IsVertex(_startV,&cv1) && IsVertex(_endV,&cv2) ){
     return FindPathBFS(cv1->vid,cv2->vid);
  } else {
     cout << "\nIn FindPathBFS: start or goal vertex (";
     cout << _startV << ", " << _endV << ") not in graph";
     vector< pair<VERTEX,WEIGHT> > path;
     return path; 
  }
};


template<class VERTEX, class WEIGHT>
vector< pair<VERTEX,WEIGHT> > 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
FindPathBFS (VID _startVid, VID _endVid) const {
  WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree;
  vector< pair<VERTEX,WEIGHT> > path;
  list<VID> q;
  CVI cv1,cv2;
  VI v1,v2;
  VID v1id, v2id;


  if ( IsVertex(_startVid,&cv1) ) {
     path.reserve( v.size() );
     q.push_back(_startVid);
     v1 = const_cast<VI>(cv1);
     bfstree.AddVertex(v1->data,_startVid); 
  } else {
     cout << "\nIn FindPathBFS: start vertex (" << _startVid << ") not in graph";
     return path; 
  }

  while ( !q.empty() && !bfstree.IsVertex(_endVid) ) {
     v1id = q.front();
     if ( IsVertex(v1id,&cv1) ) {
       for (CEI e = cv1->edgelist.begin(); e < cv1->edgelist.end(); e++) {
         v2id = e->vertex2id;
         if ( !bfstree.IsVertex(v2id) && IsVertex(v2id,&cv2) ) {
            q.push_back(v2id);
            v2 = const_cast<VI>(cv2);
            bfstree.AddVertex(v2->data,v2id);
            if ( bfstree.AddEdge(v2id,v1id,e->weight) != OK) {
                cout << "\nIn FindPathBFS: OOPS! edge not added right...";
            }
         }
       }
     } else {
       cout << "\nIn GraphBFS: OOPS! vertex=" << v1id << " not in graph";
     }
     q.pop_front();
  }

  if ( bfstree.IsVertex(_endVid,&cv1) && bfstree.IsVertex(_startVid,&cv2) ) {
     path.insert( path.begin(), pair<VERTEX,WEIGHT>(cv1->data,WEIGHT::InvalidWeight() ) );
     while ( !(path.begin()->first ==  cv2->data) ) {
        CEI e = cv1->edgelist.begin();
        v1id = cv1->vid;
        v2id = e->vertex2id;
        if ( bfstree.IsVertex(v2id,&cv1) ) {
           path.insert( path.begin(), pair<VERTEX,WEIGHT>(cv1->data,e->weight) );
           bfstree.DeleteEdge(v1id,v2id);
        } else {
           cout << "In FindPathBFS: hmm....\n";
        }
     }
  }
  return path;
};

//*************************************************************** 
//  DIJKSTRA'S ALGORITHM
//*************************************************************** 

// Auxillary Data Structures & functions
template<class VERTEX, class WEIGHT>
class
WeightedMultiDiGraph<VERTEX,WEIGHT>::
dkinfo {
public:
  dkinfo() {};
  dkinfo(VID _vid, VID _pvid, double _d) {vid=_vid; predvid=_pvid; dist=_d;}; 
  VID    vid;
  VID    predvid;
  double dist;
};

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
dkinfo_Compare ( const dkinfo& _d1, const dkinfo& _d2) {
  return ( _d1.dist > _d2.dist );
};

///////////////////////////////////////////////////////////////////////
// Dijkstra's Algorithm (follows CLR)
// *NOTE* returns a sssp tree where edge weights are distances from source
//        i.e., the edge weights are cumulative path lengths
///////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DijkstraSSSP(VID _startVid) const {

  CVI cv1;
  VI v1;

  WeightedMultiDiGraph<VERTEX,WEIGHT> sssptree;

  // initialize all distances to be big... and predecessors = NULL
  // priority_queue<dkinfo,dkinfo_Compare> pq;
  vector<dkinfo> pq;
  double  maxdist = v.size() * WEIGHT::MaxWeight();
  for ( cv1 = v.begin(); cv1 != v.end(); cv1++) {
     v1 = const_cast<VI>(cv1);
     if (v1->vid == _startVid) {
        pq.push_back( dkinfo(v1->vid,INVALID_VID,0) );
     } else {
        pq.push_back( dkinfo(v1->vid,INVALID_VID,maxdist) );
     }
  }
  sort( pq.begin(), pq.end(),  ptr_fun(dkinfo_Compare) );

  // loop through and determine shortest paths 
  while ( pq.size() != 0  && (pq.back().dist < maxdist) ) {
     bool relax = false;
     dkinfo u = pq.back();
     if ( sssptree.GetVertexCount() == 0 ) {
       VERTEX tmp = GetData(u.vid);
       sssptree.AddVertex( tmp );
     } else {
       VERTEX tmp = GetData(u.vid);
       sssptree.AddVertex( tmp );
       VERTEX tmp1 = GetData(u.predvid);
       sssptree.AddEdge( tmp1, tmp, u.dist);
     }; 
     pq.pop_back();

     // check all u's successors 
     vector<VID> adj = GetSuccessors(u.vid);
     for (int i = 0; i < adj.size(); i++) {
        for (int j=0; j < pq.size(); j++) {
          if (adj[i] == pq[j].vid) {
            double wt = GetEdgeWeight(u.vid,pq[j].vid).Weight();
            // relax 
            if ( pq[j].dist > u.dist + wt ) { 
              relax = true;
              pq[j].dist = u.dist + wt;
              pq[j].predvid = u.vid;
            }
          }
        } // endfor
     } // endfor
     if (relax) sort( pq.begin(), pq.end(),  ptr_fun(dkinfo_Compare) );
  } // endwhile
  return sssptree;
} 

///////////////////////////////////////////////////////////////////////
// FindPathDijkstra
// *NOTE* returns shortest path where edge weights are original edge weights
//        i.e., the edge weights are *not* the cumulative path weight
///////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
vector< pair<VERTEX,WEIGHT> > 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
FindPathDijkstra (VID _v1id, VID _v2id) const {

  // first, get Dijkstra's SSSP tree
  WeightedMultiDiGraph<VERTEX,WEIGHT> sssptree;
  sssptree =  DijkstraSSSP(_v1id); 

  // now, get bfspath in sssp tree (there's only one path in tree!)
  vector< pair<VERTEX,WEIGHT> > dpath;
  VERTEX tmp1 = GetData(_v1id);
  VERTEX tmp2 = GetData(_v2id);
  dpath = sssptree.FindPathBFS(tmp1, tmp2);

  // now, get "real" edge weights (not the distances in sssptree)
  for (int i=1; i < dpath.size(); i++) {
     WEIGHT tmp = GetEdgeWeight( dpath[i-1].first, dpath[i].first ); 
     dpath[i-1].second = tmp;
  }

  return dpath;
}

///////////////////////////////////////////////////////////////////////
// FindPathDijkstra
// *NOTE* returns shortest path where edge weights are original edge weights
///////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
vector< pair<VERTEX,WEIGHT> >
WeightedMultiDiGraph<VERTEX,WEIGHT>::
FindPathDijkstra (VERTEX& _startV, VERTEX& _endV) const {

  CVI cv1,cv2;
  if ( IsVertex(_startV,&cv1) && IsVertex(_endV,&cv2) ){
     return FindPathDijkstra(cv1->vid,cv2->vid);
  } else {
     cout << "\nIn FindPathDijkstra: start or goal vertex (";
     cout << _startV << ", " << _endV << ") not in graph";
     vector< pair<VERTEX,WEIGHT> > path;
     return path;
  }
};

//*************************************************************** 
//  DFS
//*************************************************************** 
template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DFS () const {

  vector<VID> dfstree;
  deque<VID> tps;
  deque<VID> cycle;

  DfsTpsCyc (0, 0, dfstree, tps, cycle); 

  return dfstree;
};

template<class VERTEX, class WEIGHT>
deque<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
TopologicalSort () {

  vector<VID> dfstree;
  deque<VID> tps;
  deque<VID> cycle;

  DfsTpsCyc (1, 0, dfstree, tps, cycle);
  
  return tps;            
};

template<class VERTEX, class WEIGHT>
deque<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
CycleDetect () {

  vector<VID> dfstree;
  deque<VID> tps;
  deque<VID> cycle;

  typedef deque<VID>::iterator CYC;

      	DfsTpsCyc (0, 1, dfstree, tps, cycle);

	if(!cycle.empty() ) {
	  cout << "Cycle detected in the Graph:";
		for( CYC cyc = cycle.begin(); cyc < cycle.end(); cyc++) {
		  	if ( *cyc == -1 ) cout << "\ncycle: ";
 			else cout << *cyc << " ";   
	        }
	  cout << endl;
	}
	else cout<< "No cycle in the graph." << endl;

  return cycle;            
};


template<class VERTEX, class WEIGHT>
void
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DfsTpsCyc (int tpsflag, int cycflag, vector<VID>&
	   dfstree, deque<VID>& tps, deque<VID>& cycle) {

  typedef vector<VID>::iterator DFSVI;
  typedef deque<VID>::iterator TPSVI;
  typedef deque<VID>::iterator CYC;

  VID *vnode;   //store vertex visited in seqence to record
			//cycle, also used as a stack
  vnode = (VID *) malloc(numVerts * sizeof(VID) );

  int *color;   //0: white, 1: grey, 2: black
	color = (int *) malloc(numVerts * sizeof(int) );

  int ii;
  for( ii=0; ii<numVerts; ii++) color[ii] = 0;    
  for( ii=0; ii<numVerts; ii++) vnode[ii] = (VID) 0;    

  CYC cyc;
  CVI cv, vi, v1;
  VID vid, v1id, v2id;	//vid=x, v1id=parent(x), v2id=adj(x)

  int j=0, kk=0, jj=1;

 for (vi = v.begin(); vi < v.end(); vi++) {
    int k=1;
    vid = vi->vid;
    if( color[vid] == 0 ) {
        dfstree.push_back(vid);
        color[vid] = 1;
	vnode[k] = vid;  

	v1id = 0;
	while( k > 0 ) {
		vid = vnode[k];
		if ( IsVertex(vid, &cv)) ;
		else cout << "\nIn GraphDFS: vid=" << vid << " not in graph";
		v1 = const_cast<VI>(cv);
		CEI e = v1->edgelist.begin(); 
		while ( e < v1->edgelist.end() ) {
			v2id = e->vertex2id;
			if( color[v2id] == 0 ) {
		 		v1id = vid;
				vid = v2id;
        		 	dfstree.push_back(vid);
	        	 	color[vid] = 1;
			 	vnode[++k] = vid;
	       		 	if ( IsVertex(vid, &cv) ) {
					v1 = const_cast<VI>(cv);
					e = v1->edgelist.begin();
				 }  
        			 else 
					cout << "\nIn GraphDFS: v2id=" << v2id << " not in graph";
			}  //if (color..)
			else if( color[v2id] == 1 && cycflag == 1) {
	cout << "back edge = (" << vid << "," << v2id << ")" << endl;
				cycle.push_front(v2id);
				for(kk=k; vnode[kk]!=v2id; kk--) {
					cycle.push_front(vnode[kk]);
				}
				cycle.push_front(v2id);
				cycle.push_front((VID) -1);
				e++, j++;
			} //else if
 			else {
				e++;
       			} //else    
   	 	}  
   	if(tpsflag == 1) tps.push_front(vnode[k]);
    	color[vnode[k]] = 2;
   	k--;
    	} 
   } 
}  
free(vnode);
free(color);
};


  //==================================
  // WeightedMultiDiGraph class Methods: Display, Input, Output 
  //==================================


template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DisplayGraph() const {
      CVI vi;
      int i;
      for (vi = v.begin(), i=0; vi < v.end(); vi++, i++) {
          cout << setw(3) << i << ": ";
          vi->DisplayEdgelist();
      }
};

template<class VERTEX, class WEIGHT>
void
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DisplayVertices() const {
      for (CVI vi = v.begin(); vi != v.end(); vi++) {
          DisplayVertex(vi->vid); 
      } 
};

template<class VERTEX, class WEIGHT>
void
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DisplayVertex(VID _v1id) const {
      CVI v1;
      if ( IsVertex(_v1id,&v1) ) {
         cout << "vertex: id =" << setw(3) << _v1id; 
         cout << ", data = [" << v1->data;
         cout << "]" << endl;
      } else {
         cout << "vertex with id=" << _v1id << " not in graph.";
      }
};

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DisplayVertexAndEdgelist(VID _v1id) const{
      CVI v1;
      if ( IsVertex(_v1id,&v1) ) {
         cout << "vertex: ";
         v1->DisplayEdgelist();
      } else {
         cout << "vertex with id=" << _v1id << " not in graph.";
      }
};

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
WriteGraph(const char* _fname) const {

      ofstream  myofstream(_fname);
      if (!myofstream) {
         cout << "\nInWriteGraph: can't open outfile: " << _fname ; 
      }
      WriteGraph(myofstream);
      myofstream.close();
};

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
WriteGraph(ostream& _myostream) const {

#ifndef __HP_aCC
      _myostream << endl << "#####GRAPHSTART#####";
#else 
      _myostream << "GRAPHSTART";
#endif

      _myostream << endl << numVerts << " " << numEdges << " " << vertIDs; 

      //format: VID VERTEX #edges VID WEIGHT VID WEIGHT ... 
      for (CVI vi = v.begin(); vi != v.end(); vi++) {
          _myostream << endl;
          vi->WriteEdgelist(_myostream);
      } 

#ifndef __HP_aCC
      _myostream << endl << "#####GRAPHSTOP#####";
#else 
      _myostream << endl << "GRAPHSTOP";
#endif

      _myostream << endl; 
};


template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
ReadGraph(const char*  _fname) {

      ifstream  myifstream(_fname);
      if (!myifstream) {
         cout << "\nIn ReadGraph: can't open infile: " << _fname ;
         return;
      }
      ReadGraph(myifstream);
      myifstream.close();
};

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
ReadGraph(istream& _myistream) {
      VID v1id, v2id, maxVID;
      CVI  cv1;
      VI  v1;
      VERTEX data;
      WEIGHT weight;
      int nVerts, nEdges, nedges;
      char tagstring[100];
      //string tagstring;

      _myistream >> tagstring;
      //if ( tagstring != "#####GRAPHSTART" ) {
      if ( !strstr(tagstring,"GRAPHSTART") ) {
         cout << endl << "In ReadGraph: didn't read GRAPHSTART tag right";
         return;
      }

      if (numVerts != 0) {
         EraseGraph(); // empty graph before filling it in
      }

      _myistream >> nVerts >> nEdges >> maxVID;

      for (int i = 0; i < nVerts; i++){
         _myistream >> v1id >> data;             // read and add vertex 
         AddVertex(data,v1id);
         if ( !IsVertex(v1id,&cv1) ) {
            cout << "\nIn ReadGraph: didn't add v1...";
         }

         _myistream >> nedges;               // read and add its edges
         v1 = const_cast<VI>(cv1);
         for (int j = 0; j < nedges; j++){
            _myistream >> v2id >> weight; 
            v1->AddEdge(v2id,weight);
         }
      }
      
      numVerts = nVerts;
      numEdges = nEdges;
      vertIDs = maxVID; // set the maximum VID used so far...
                        // should sort verts & find biggest used...

      _myistream >> tagstring;
      //if ( tagstring != "#####GRAPHSTOP" ) {
      if ( !strstr(tagstring,"GRAPHSTOP") ) {
         cout << endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
         return;
      }
};

  //==================================
  // WeightedMultiDiGraph class Predicates, Comparisons & Operations
  //==================================

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>*
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
my_find_VID_eq(const VID _vid) const {
   CVI vi, startvi; 

   // find the spot to start looking, hopefully at v[_vid]
   if ( v.size() > _vid ) {
     startvi = v.begin() + _vid;
   } else {
     startvi = v.end() - 1; 
   }

   // look back from v[_vid]
   vi = startvi;
   while ( vi >= v.begin()  ) {
      if ( vi->vid == _vid) {
         return const_cast<WtVertexType<VERTEX,WEIGHT>*>( vi );
      } else {
         vi--;
      }
   }

   // look forward from v[_vid]
   vi = startvi;
   while ( vi < v.end()  ) {
      if ( vi->vid == _vid) {
         return const_cast<WtVertexType<VERTEX,WEIGHT>*>( vi );
      } else {
         vi++;
      }
   }

   // if didn't find it return v.end() like STL find
   return const_cast<WtVertexType<VERTEX,WEIGHT>*>( v.end() );
}

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>*
WeightedMultiDiGraph<VERTEX,WEIGHT>::
my_find_VDATA_eq(const VERTEX& _v) const {
   CVI cvi = v.begin();
   VI vi;
   bool found = false;
   vi = const_cast<VI> (cvi);
   while (vi != v.end() && !found) {
      if ( vi->data == _v) {
         found = true;
      } else {
         vi++;
      }
   }
   return const_cast<WtVertexType<VERTEX,WEIGHT>*>(vi);
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
VID_Compare (const Vertex& _v1, const Vertex& _v2){
      return (_v1.vid < _v2.vid ) ; 
};


/*-------------- don't work with sgi/CC, work with sun/g++ -----------------
template<class VERTEX, class WEIGHT>
class 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
VID_eq : public unary_function< WeightedMultiDiGraph<VERTEX,WEIGHT>::Vertex,bool> {
   public:
     explicit VID_eq(const VID vid) : testid (vid) {}
     bool operator() (Vertex v) {
         return v.vid == testid;
     };
   protected:
   private:
     VID testid;
};

template<class VERTEX, class WEIGHT>
class
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
VDATA_eq : public unary_function< WeightedMultiDiGraph<VERTEX,WEIGHT>::Vertex, bool> {
   public:
     explicit VDATA_eq(const VERTEX vt) : testdata (vt) {}
     bool operator() (Vertex v) {
         return v.data == testdata;
     };
   protected:
   private:
     VERTEX testdata;
};

template<class VERTEX, class WEIGHT>
class 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
VID_Compare {
   public:
     int operator() (const Vertex & _v1, const Vertex & _v2) const 
     { return (_v1.GetVID() < _v2.GetVID() ); };
   protected:
   private:
};

--------------------------------------------------------------------------*/



//=====================================================================
// METHODS FOR template WeightedGraph Class
//=====================================================================

  //==================================
  // WeightedGraph class Methods: Constructors and Destructor
  //==================================

template<class VERTEX, class WEIGHT>
WeightedGraph<VERTEX,WEIGHT>::
WeightedGraph(){
};

template<class VERTEX, class WEIGHT>
WeightedGraph<VERTEX,WEIGHT>::
WeightedGraph(int _sz)
 : WeightedMultiDiGraph<VERTEX,WEIGHT>(_sz) 
{
};

template<class VERTEX, class WEIGHT>
WeightedGraph<VERTEX,WEIGHT>::
WeightedGraph(int _sz, int _edgelistsz)
 : WeightedMultiDiGraph<VERTEX,WEIGHT>(_sz,_edgelistsz) 
{
};

// Function removed for Vizmo - bmd 8 - 22 - 00
//template<class VERTEX, class WEIGHT>
//WeightedGraph<VERTEX,WEIGHT>::
//WeightedGraph( WeightedMultiDiGraph<VERTEX,WEIGHT> _base)
// : WeightedMultiDiGraph<VERTEX,WEIGHT>(_base) 
//{
//};

template<class VERTEX, class WEIGHT>
WeightedGraph<VERTEX,WEIGHT>::
~WeightedGraph(){
};
  //==================================
  // WeightedGraph class Methods: Adding & Deleting Vertices
  //==================================

  //==================================
  // WeightedGraph class Methods: Adding & Deleting Edges
  //==================================

template<class VERTEX, class WEIGHT>
int 
WeightedGraph<VERTEX,WEIGHT>:: 
AddEdge(VID _v1id, VID _v2id, WEIGHT _weight) {
    CVI cv1,cv2;
    VI v1,v2;
    if (_v1id != _v2id && IsVertex(_v1id,&cv1) && IsVertex(_v2id,&cv2) ) {
        if ( !IsEdge(_v1id,_v2id) ) {
           v1 = const_cast<VI>(cv1);
           v2 = const_cast<VI>(cv2);
           v1->AddEdge(_v2id,_weight);
           v2->AddEdge(_v1id,_weight);
           numEdges++;
           return OK;
        } else {
           #ifndef QUIETGRAPH
                 cout << "\nIn AddEdge: edge already in graph, not added";
           #endif
           return ERROR;
        }
     } else {
         cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
         return ERROR;
     };
};

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, VID _v2id, pair<WEIGHT,WEIGHT> _wtpair ) {
    CVI cv1,cv2;
    VI v1,v2;
    if (_v1id != _v2id && IsVertex(_v1id,&cv1) && IsVertex(_v2id,&cv2) ) {
        if ( !IsEdge(_v1id,_v2id) ) {
           v1 = const_cast<VI>(cv1);
           v2 = const_cast<VI>(cv2);
           v1->AddEdge(_v2id,_wtpair.first);
           v2->AddEdge(_v1id,_wtpair.second);
           numEdges++;
           return OK;
        } else {
           #ifndef QUIETGRAPH
                 cout << "\nIn AddEdge: edge already in graph, not added";
           #endif
           return ERROR;
        }
     } else {
         cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
         return ERROR;
     };
};


template<class VERTEX, class WEIGHT>
int 
WeightedGraph<VERTEX,WEIGHT>:: 
AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    CVI cv1, cv2;
    VI v1, v2;
    if ( !(_v1 == _v2) && IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
        if ( !IsEdge(_v1,_v2) ) {
           v1 = const_cast<VI>(cv1);
           v2 = const_cast<VI>(cv2);
           v1->AddEdge(v2->vid,_weight);
           v2->AddEdge(v1->vid,_weight);
           numEdges++;
           return OK;
        } else {
           #ifndef QUIETGRAPH
                 cout << "\nIn AddEdge: edge already in graph, not added";
           #endif
           return ERROR;
        }
      } else {
         cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
         return ERROR;
     };
};

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT> _wtpair) {
    CVI cv1, cv2;
    VI v1, v2;
    if ( !(_v1 == _v2) && IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
        if ( !IsEdge(_v1,_v2) ) {
           v1 = const_cast<VI>(cv1);
           v2 = const_cast<VI>(cv2);
           v1->AddEdge(v2->vid,_wtpair.first);
           v2->AddEdge(v1->vid,_wtpair.second);
           numEdges++;
           return OK;
        } else {
           #ifndef QUIETGRAPH
                 cout << "\nIn AddEdge: edge already in graph, not added";
           #endif
           return ERROR;
        }
      } else {
         cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
         return ERROR;
     };
};



template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, EI _ei) {
    CVI cv1, cv2;
    VI v1, v2;
    VID v2id = _ei->vertex2id;
    WEIGHT weight = _ei->weight;

    if (_v1id != v2id && IsVertex(_v1id,&cv1) && IsVertex(v2id,&cv2) ) {
        if ( !IsEdge(_v1id,v2id) ) {
           v1 = const_cast<VI>(cv1);
           v2 = const_cast<VI>(cv2);
           v1->AddEdge(v2id,weight);
           v2->AddEdge(_v1id,weight);
           numEdges++;
           return OK;
        } else {
           #ifndef QUIETGRAPH
                 cout << "\nIn AddEdge: edge already in graph, not added";
           #endif
           return ERROR;
        }
      } else {
         cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
         return ERROR;
     };
};


template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
DeleteEdge(VID _v1id, VID _v2id, int _n) {
   CVI cv1, cv2;
   VI v1, v2;

   if ( IsVertex(_v1id,&cv1) && IsVertex(_v2id,&cv2) && IsEdge(_v1id,_v2id) ) {
      v1 = const_cast<VI>(cv1);
      v2 = const_cast<VI>(cv2);
      int ok1 = v1->DeleteXEdges(_v2id,-1);
      int ok2 = v2->DeleteXEdges(_v1id,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1 
         numEdges--;
         return OK;
      } 
   }
   return ERROR;
};

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
DeleteWtEdge(VID _v1id, VID _v2id, WEIGHT _w, int _n) {
   CVI cv1, cv2;
   VI v1, v2;

   if ( IsVertex(_v1id,&cv1) && IsVertex(_v2id,&cv2) && IsEdge(_v1id,_v2id,_w) ) {
      v1 = const_cast<VI>(cv1);
      v2 = const_cast<VI>(cv2);
      int ok1 = v1->DeleteXEdges(_v2id,-1);
      int ok2 = v2->DeleteXEdges(_v1id,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1 
         numEdges--;
         return OK;
      } 
   }
   return ERROR;
};

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
DeleteEdge(VERTEX& _v1, VERTEX& _v2, int _n) {
   CVI cv1, cv2;
   VI v1, v2;

   if ( IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) && IsEdge(_v1,_v2) ) {
      v1 = const_cast<VI>(cv1);
      v2 = const_cast<VI>(cv2);
      int ok1 = v1->DeleteXEdges(v2->vid,-1);
      int ok2 = v2->DeleteXEdges(v1->vid,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1
         numEdges--; 
         return OK;
      }
   }
   return ERROR;
};

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
DeleteWtEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _w, int _n) {
   CVI cv1, cv2;
   VI v1, v2;

   if ( IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) && IsEdge(_v1,_v2,_w) ) {
      v1 = const_cast<VI>(cv1);
      v2 = const_cast<VI>(cv2);
      int ok1 = v1->DeleteXEdges(v2->vid,-1);
      int ok2 = v2->DeleteXEdges(v1->vid,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1
         numEdges--; 
         return OK;
      }
   }
   return ERROR;
};

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
ChangeEdgeWeight(VID _v1id, VID _v2id, WEIGHT _weight) {

   if ( DeleteEdge(_v1id, _v2id) == OK) {
      return AddEdge(_v1id, _v2id,_weight);
   } else {
      return ERROR;
   }  
};

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
ChangeEdgeWeight(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {

   if ( DeleteEdge(_v1, _v2) == OK) {
      return AddEdge(_v1, _v2,_weight);
   } else {
      return ERROR;
   }  
};


  //==================================
  // WeightedGraph class Methods: Getting Data & Statistics
  //==================================

// only report each edge once
template<class VERTEX, class WEIGHT>
vector< pair< pair<VID,VID>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetEdges() const {
  vector< pair< pair<VID,VID>, WEIGHT> > edges;

  edges.reserve(numEdges);

  for (CVI vi = v.begin(); vi != v.end(); vi++) {
     for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
        if ( vi->vid < ei->vertex2id) {
          pair<VID,VID> newedge(vi->vid, ei->vertex2id);
          pair<pair<VID,VID>,WEIGHT> newedgewt(newedge, ei->weight);
          edges.push_back( newedgewt );
        }
     }
  }
  return edges;
};

// only report each edge once
template<class VERTEX, class WEIGHT>
vector< pair< pair<VERTEX,VERTEX>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetEdgesVData() const {
  vector< pair< pair<VERTEX,VERTEX>, WEIGHT> > edges;

  edges.reserve(numEdges);
  for (CVI vi = v.begin(); vi != v.end(); vi++) {
     for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
        if ( vi->vid < ei->vertex2id) {
          VERTEX v2data = GetData(ei->vertex2id);
          pair<VERTEX,VERTEX> newedge(vi->data, v2data);
          pair<pair<VERTEX,VERTEX>,WEIGHT> newedgewt(newedge, ei->weight);
          edges.push_back( newedgewt );
        }
     }
  }
  return edges;
};


template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
GetVertexDegree(VID _v1id) const {
     CVI v1;
     if ( IsVertex(_v1id,&v1) ) {
         return v1->edgelist.size(); 
     } else {
         cout << "\nGetVertexDegree: vertex "<< _v1id << " not in graph";
         return ERROR;
     };
};

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedGraph<VERTEX,WEIGHT>::
GetAdjacentVertices(VID _v1id) const {
   return GetSuccessors(_v1id);
};

template<class VERTEX, class WEIGHT>
vector< pair<pair<VID,VID>,WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetIncidentEdges(VID _v1id) const {
     vector< pair<pair<VID,VID>,WEIGHT> > iedges;
     CVI v1;
     if ( IsVertex(_v1id,&v1) ) {
         iedges.reserve( v1->edgelist.size() );
         for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
            pair<VID,VID> nextedge(_v1id,ei->vertex2id);
            pair<pair<VID,VID>,WEIGHT> nextedgewt(nextedge,ei->weight);
            iedges.push_back( nextedgewt );
         }
     } else {
         cout << "\nGetIncidentEdges: vertex "<< _v1id << " not in graph";
     };
     return iedges;
};

template<class VERTEX, class WEIGHT>
vector< pair<pair<VERTEX,VERTEX>,WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetIncidentEdgesVData(VID _v1id) const {
     vector< pair<pair<VERTEX,VERTEX>,WEIGHT> > iedges;
     CVI v1;
     if ( IsVertex(_v1id,&v1) ) {
         iedges.reserve( v1->edgelist.size() );
         for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
            pair<VERTEX,VERTEX> nextedge( GetData(_v1id), GetData(ei->vertex2id));
            pair<pair<VERTEX,VERTEX>,WEIGHT> nextedgewt(nextedge,ei->weight);
            iedges.push_back( nextedgewt );
         }
     } else {
         cout << "\nGetIncidentEdgesVData: vertex "<< _v1id << " not in graph";
     };
     return iedges;
};


  //==================================
  // WeightedGraph class Methods: Connected Components Utilities
  //==================================

template<class VERTEX, class WEIGHT>
bool
WeightedGraph<VERTEX,WEIGHT>::
IsSameCC (VID _v1id, VID _v2id) const {

   WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = BFS(_v1id); 

   if ( bfstree.IsVertex(_v1id) && bfstree.IsVertex(_v2id) ) {
      return true;
   } else {
      return false;
   };
};

template<class VERTEX, class WEIGHT>
bool
WeightedGraph<VERTEX,WEIGHT>::
IsSameCC (VERTEX& _v1, VERTEX& _v2) const {

   WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = BFS(_v1);

   if ( bfstree.IsVertex(_v1) && bfstree.IsVertex(_v2) ) {
      return true;
   } else {
      return false;
   }
};

template<class VERTEX, class WEIGHT>
vector<VID> 
WeightedGraph<VERTEX,WEIGHT>::
GetCC ( VID _v1id) const {

   WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = BFS(_v1id);
   vector<VID> ccverts = bfstree.GetVerticesVID();
   //sort( ccverts.begin(), ccverts.end() ); //sort by VID for CC stats
   return ccverts;
}

template<class VERTEX, class WEIGHT>
vector<VERTEX>
WeightedGraph<VERTEX,WEIGHT>::
GetCC ( VERTEX& _v1) const {

   WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = BFS(_v1);
   vector<VERTEX> ccverts = bfstree.GetVerticesData();
   return ccverts;
}

template<class VERTEX, class WEIGHT>
vector< pair<pair<VID,VID>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetCCEdges ( VID _v1id) const {

   vector< pair<pair<VID,VID>,WEIGHT> > ccedges, newedges;

   WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = BFS(_v1id);
   vector<VID> ccverts = bfstree.GetVerticesVID();
   for (int i=0; i < ccverts.size(); i++) {
      newedges = GetIncidentEdges(ccverts[i]);
      for (int k=0; k < newedges.size(); k++){
         ccedges.push_back ( newedges[k] );
      }
   }
   return ccedges;
}

template<class VERTEX, class WEIGHT>
vector< pair<pair<VID,VID>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetCCEdges ( VERTEX&  _v1) const {

   return GetCCEdges ( GetVID(_v1) );
}

template<class VERTEX, class WEIGHT>
vector< pair<pair<VERTEX,VERTEX>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetCCEdgesVData ( VID  _v1id) const {

   vector< pair<pair<VERTEX,VERTEX>,WEIGHT> > ccedges, newedges;

   WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = BFS(_v1id);
   vector<VID> ccverts = bfstree.GetVerticesVID();
   for (int i=0; i < ccverts.size(); i++) {
      newedges = GetIncidentEdgesVData(ccverts[i]);
      for (int k=0; k < newedges.size(); k++){
         ccedges.push_back ( newedges[k] );
      }
   }
   return ccedges;
}

template<class VERTEX, class WEIGHT>
vector< pair<pair<VERTEX,VERTEX>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetCCEdgesVData ( VERTEX&  _v1) const {

   return GetCCEdgesVData ( GetVID(_v1) );
}

// return 2D vector ccedges[i,j] = jth edge of ith CC, edge is VERTEX pair
template<class VERTEX, class WEIGHT>
vector< vector< pair<VERTEX,VERTEX> > >
WeightedGraph<VERTEX,WEIGHT>::
GetEdgesByCCVDataOnly() const {

  vector< vector< pair<VERTEX,VERTEX> > > ccedges;

  vector< pair<int,VID> > cc = GetCCStats();
  for (int i=0; i < cc.size(); i++) {
     vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > thiscc = GetCCEdgesVData(cc[i].second);
     vector< pair<VERTEX,VERTEX> >  edges;
     for (int j=0; j < thiscc.size(); j++) {
       pair<VERTEX,VERTEX> thisedge(thiscc[j].first.first,thiscc[j].first.second);
       edges.push_back ( thisedge );
     }
     ccedges.push_back ( edges );
  }
  return ccedges;
}

template<class VERTEX, class WEIGHT>
vector< pair<int,VID> > 
WeightedGraph<VERTEX,WEIGHT>::
GetCCStats () const {
  vector< pair<int,VID> > ccstats;
  vector<VID> verts = GetVerticesVID();

  while ( verts.size() != 0 ) {
     VID v1id = verts.front();
     vector<VID> CC = GetCC(v1id);
     int CCsize = CC.size();
     ccstats.push_back( pair<int,VID>(CCsize,v1id) );
     sort( CC.begin(), CC.end() ); //sort by VID for set_difference
     set_difference
          ( verts.begin(),verts.end(),CC.begin(),CC.end(),verts.begin() );
     verts.erase(verts.end()-CC.size(), verts.end());
  }
  sort ( ccstats.begin(), ccstats.end(), ptr_fun(CCVID_Compare) );
  return ccstats;
};

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
GetCCcount () const {
  vector< pair<int,VID> > ccstats = GetCCStats();
  return ccstats.size();
};


  //==================================
  // WeightedGraph class Methods: Display, Input, Output 
  //==================================


template<class VERTEX, class WEIGHT>
void
WeightedGraph<VERTEX,WEIGHT>::
DisplayCC ( VID _v1id) const {

   typedef vector<VID>::iterator VI;

   vector<VID> ccverts = GetCC(_v1id);
   cout << "\nCC[" << _v1id << "] = {";
   for (VI vi = ccverts.begin(); vi < ccverts.end(); vi++ ) {
       cout << *vi; 
       if (vi != ccverts.end() -1 ) cout << ", ";
   }
   cout << "}\n";
};

template<class VERTEX, class WEIGHT>
void
WeightedGraph<VERTEX,WEIGHT>::
DisplayEdgesByCCVDataOnly() const {


   vector< vector< pair<VERTEX,VERTEX> > >  ccedges = GetEdgesByCCVDataOnly();

   cout << endl << "Edges in each connected component (vertex data shown)";
   for (int cc=0; cc < ccedges.size(); cc++){
     cout << endl << "CC[" << cc << "]: ";
     for (int e=0; e < ccedges[cc].size(); e++){ 
       cout << " (" << ccedges[cc][e].first << "," << ccedges[cc][e].second << ")"; 
     }
   }

};

template<class VERTEX, class WEIGHT>
void
WeightedGraph<VERTEX,WEIGHT>::
DisplayCCStats(int _maxCCprint) const {

   typedef vector< pair<int,VID> >::const_iterator CCI; 
   int maxCCprint;

   vector< pair<int,VID> > ccstats = GetCCStats();
   if (_maxCCprint == -1) {
      maxCCprint = ccstats.size();
   } else {
      maxCCprint = _maxCCprint;
   }

   int ccnum = 1;
   cout << "\nThere are " << ccstats.size() << " connected components:";
   for (CCI vi = ccstats.begin(); vi != ccstats.end(); vi++) {
       cout << "\nCC[" << ccnum << "]: " << vi->first ; 
       cout << " (vid=" << vi->second << ")";
       ccnum++;
       if (ccnum > maxCCprint) return; 
   }
};

  //==================================
  // WeightedGraph class Predicates, Comparisons & Operations
  //==================================

template<class VERTEX, class WEIGHT>
bool
WeightedGraph<VERTEX,WEIGHT>::
CCVID_Compare (const pair<int,VID>& _cc1, const pair<int,VID>& _cc2) {
      return (_cc1.first > _cc2.first ) ;
};


//=====================================================================
// METHODS FOR template WtVertexType Class
//=====================================================================

  //==================================
  // WtVertexType class Methods: Constructors and Destructor
  //==================================

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
WtVertexType(){
};

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
WtVertexType(VERTEX& _data, VID _id){
     data = _data;
     vid = _id;
};

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
WtVertexType(VERTEX& _data, VID _id, int _edgelistsz){
     data = _data;
     vid = _id;
     edgelist.reserve( _edgelistsz );
};


template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
~WtVertexType(){
};

  //==================================
  // Vertex class Methods: Adding & Deleting Edges
  //==================================

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>:: 
AddEdge(VID _v2id, WEIGHT _weight) {
     WtEdge newEdge(_v2id, _weight);
     edgelist.push_back(newEdge);
};

//delete upto _x edges (v1,v2) of any weight (delete first encountered)
template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
DeleteXEdges(VID _v2id, int _x) {
     int num_to_delete; 
     int num_deleted = 0;
     if (_x == -1) 
        num_to_delete = edgelist.size();
        else num_to_delete = _x;
     //EI ei = find_if(edgelist.begin(), edgelist.end(), EID_eq(_v2id) );
     EI ei = my_find_EID1_eq(_v2id);
     while ( (ei != edgelist.end()) && (num_deleted < num_to_delete) ) {
         edgelist.erase(ei);
         num_deleted++;
         //ei = find_if(ei, edgelist.end(), EID_eq(_v2id) );
         ei = my_find_EID2_eq(ei,edgelist.end(),_v2id);
     }
     return num_deleted;
};

//delete upto _x edges (v1,v2) of specified weight (delete first encountered) 
template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
DeleteXEdges(VID _v2id, WEIGHT _weight, int _x) {
     int num_to_delete;
     int num_deleted = 0;
     if (_x == -1)
        num_to_delete = edgelist.size();
        else num_to_delete = _x;
     //EI ei = find_if(edgelist.begin(), edgelist.end(), EID_eq(_v2id) );
     EI ei = my_find_EID1_eq(_v2id);
     while ( (ei != edgelist.end()) && (num_deleted < num_to_delete) ) {
         if (ei->weight == _weight) {
            edgelist.erase(ei);
            num_deleted++;
            //ei = find_if(ei, edgelist.end(), EID_eq(_v2id) );
            ei = my_find_EID2_eq(ei,edgelist.end(),_v2id);
         } else {
            //ei = find_if(ei+1, edgelist.end(), EID_eq(_v2id) );
            ei = my_find_EID2_eq(ei+1,edgelist.end(),_v2id);
         }
     }
     return num_deleted;
};


  //==================================
  // Vertex class Methods: Finding Edges
  //==================================

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id) const {
     EI ei;
     return ( IsEdge(_v2id, &ei) );
};

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id, const WtEdge** _ei) const {
     //CEI ei = find_if(edgelist.begin(), edgelist.end(), EID_eq(_v2id));
     CEI ei = my_find_EID1_eq(_v2id);
     if (ei != edgelist.end() ) {
         *_ei = ei;
         return true;
     } else {
         return false;
     }
};

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id, WEIGHT _weight, const WtEdge** _ei) const {
     //CEI ei = find_if(edgelist.begin(), edgelist.end(), EIDWT_eq( pair<VID,WEIGHT>(_v2id,_weight) ) );
     CEI ei = my_find_EIDWT_eq( pair<VID,WEIGHT>(_v2id,_weight) );
     if (ei != edgelist.end() ) {
         *_ei = ei;
         return true;
     } else {
         return false;
     }
};

  //==================================
  // Vertex class Methods: Getting Data & Statistics 
  //==================================

template<class VERTEX, class WEIGHT>
VERTEX 
WtVertexType<VERTEX,WEIGHT>::
GetData() const {
    return data;
};

template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
GetVID() const {
    return vid;
};

template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
GetEdgeCount() const {
    return edgelist.size();
};

template<class VERTEX, class WEIGHT>
WEIGHT 
WtVertexType<VERTEX,WEIGHT>::
GetEdgeWeight(VID _v2id) const {
     EI ei;
     CEI cei;
     if (IsEdge(_v2id,&cei)) {
	 ei = const_cast<EI> (cei);
         return ei->weight;
     } else {
         cout << "\nGetEdgeWeight: edge not in graph";
         return WEIGHT::InvalidWeight();
     };
};

  //==================================
  // Vertex class Methods: Display, Input, Output 
  //==================================

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>::
DisplayEdgelist() const {
     cout << "id =" << setw(3) << vid << ", data = ["; 
     cout << data;
     cout << "], edges={";
     for (CEI ei = edgelist.begin(); ei < edgelist.end(); ei++) {
        ei->DisplayEdge();
        if (ei != edgelist.end() - 1) cout << ", ";
     }
     cout << "} \n";
};

template<class VERTEX, class WEIGHT>
void
WtVertexType<VERTEX,WEIGHT>::
WriteEdgelist(ostream& _myostream) const {

      _myostream << vid << " "; 
      _myostream << data << " "; 
      _myostream << edgelist.size() << " "; 

      for (CEI ei = edgelist.begin(); ei != edgelist.end(); ei++) { 
          ei->WriteEdge(_myostream);
          _myostream << " "; 
      }
};

  //==================================
  // Vertex class Predicate Utilities
  //==================================

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>*
WtVertexType<VERTEX,WEIGHT>::
my_find_EID1_eq(const EID _eid) const {
   CEI ei = edgelist.begin();
   bool found = false;
   while (ei != edgelist.end() && !found) {
      if ( ei->vertex2id == _eid) {
         found = true;
      } else {
         ei++;
      }
   }
   return const_cast<WtEdgeType<VERTEX,WEIGHT>*>(ei);
};

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>*
WtVertexType<VERTEX,WEIGHT>::
my_find_EID2_eq(const WtEdgeType<VERTEX,WEIGHT>* _start, 
               const WtEdgeType<VERTEX,WEIGHT>* _end,
               const EID _eid) const {
   CEI ei = _start;
   bool found = false;
   while (ei != _end && !found) {
      if ( ei->vertex2id == _eid) {
         found = true;
      } else {
         ei++;
      }
   }
   return const_cast<WtEdgeType<VERTEX,WEIGHT>*>(ei);
};

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>*
WtVertexType<VERTEX,WEIGHT>::
my_find_EWT_eq(const WEIGHT _wt) const { 
   CEI ei = edgelist.begin();
   bool found = false;
   while (ei != edgelist.end() && !found) {
      if ( ei->weight == _wt) {
         found = true;
      } else {
         ei++;
      }
   }
   return const_cast<WtEdgeType<VERTEX,WEIGHT>*>(ei);
};

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>*
WtVertexType<VERTEX,WEIGHT>::
my_find_EIDWT_eq(const pair<VID,WEIGHT> _wtpair) const {
   CEI ei = edgelist.begin();
   bool found = false;
   while (ei != edgelist.end() && !found) {
      if ( ei->vertex2id==_wtpair.first && ei->weight == _wtpair.second ) {
         found = true;
      } else {
         ei++;
      }
   }
   return const_cast<WtEdgeType<VERTEX,WEIGHT>*>(ei);
};


/*--------------- NMA: these don't work with sgi/CC, but do for sun/g++
template<class VERTEX, class WEIGHT>
class 
WtVertexType<VERTEX,WEIGHT>::
EID_eq : public unary_function< WtEdgeType<VERTEX,WEIGHT>,bool> {
  public:
     explicit EID_eq(const VID i) : testid (i) {}
     bool operator() (WtEdgeType<VERTEX,WEIGHT> e) {
         return e.vertex2id == testid;
     };
     VID testid;
  protected:
  private:
};


template<class VERTEX, class WEIGHT>
class 
WtVertexType<VERTEX,WEIGHT>::
EWT_eq : public unary_function< WtEdgeType<VERTEX,WEIGHT>,bool> {
  public:
     explicit EWT_eq(const WEIGHT w) : testwt (w) {}
     bool operator() (WtEdgeType<VERTEX,WEIGHT> e) {
       return e.weight == testwt;
     };
     WEIGHT testwt;
  protected:
  private:
};

template<class VERTEX, class WEIGHT>
class 
WtVertexType<VERTEX,WEIGHT>::
EIDWT_eq : public unary_function< WtEdgeType<VERTEX,WEIGHT>,bool> {
  public:
     explicit EIDWT_eq(const pair<VID,WEIGHT> eid) : testedge (eid) {}
     bool operator() (WtEdgeType<VERTEX,WEIGHT> e) {
       return ((e.vertex2id==testedge.first) && (e.weight == testedge.second));
     };
     pair<VID,WEIGHT> testedge;
  protected:
  private:
};
*/

//=====================================================================
// METHODS FOR template WtEdge Class
//=====================================================================

  //==================================
  // WtEdge class Methods: Constructors and Destructor
  //==================================

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>:: 
WtEdgeType(){
};

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>:: 
WtEdgeType(VID _v2id, WEIGHT _weight){
     vertex2id = _v2id;
     weight = _weight;
};

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>:: 
~WtEdgeType(){
};

  //==================================
  // WtEdge class Methods: Getting Data & Statistics
  //==================================

  //==================================
  // WtEdge class Methods: Display, Input, Output
  //==================================

template<class VERTEX, class WEIGHT>
void 
WtEdgeType<VERTEX,WEIGHT>:: 
DisplayEdge() const {
     cout << vertex2id << "(" << weight << ")";
}; 

template<class VERTEX, class WEIGHT>
void
WtEdgeType<VERTEX,WEIGHT>::
WriteEdge(ostream& _myostream) const {
      _myostream << vertex2id << " " << weight << " ";
};

#endif

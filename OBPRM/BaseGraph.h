// $Id$

/////////////////////////////////////////////////////////////////////
/**@file  Graph.h
 *
 * General Description
 *     This set of template classes provides an implementation for 
 *     the graph abstract data type. 
 *
 *     The user must provide the parameter type VERTEX, which is
 *     the data that will be stored in each vertex of the graph.
 *     It is assumed the VERTEX class has the following operators
 *     defined for it: << >> == = 
 *
 *     In addition, for weighted graphs, the user must supply
 *     the parameter type WEIGHT, which is the data that will
 *     be stored in each edge of the graph.
 *
 *     The classes in this hierarchy (so far) include:
 *       o AbstractGraph
 *         o BaseGraph (derived from AbstractGraph)
 *
 *     Graphs are represented by adjacency lists. 
 *     The unique vertex IDs are typedef'ed short ints (which
 *     should be suffient for most graphs).
 *
 **IMPORTANT* 
 *     For convenience, options have been included for dealing either with 
 *     vertex ids (VIDs) or vertex data (VERTEX). The ones dealing with 
 *     VERTEX data are only 'safe' if VERTEX data is unique.
 *
 * Created
 * @date 04/02/02  
 * @author Gabriel Tanase; Based on original Graph.g written by Nancy Amato
 */
/////////////////////////////////////////////////////////////////////

#ifndef BaseGraph_h
#define BaseGraph_h

////////////////////////////////////////////////////////////////////////////////////////////
//include standard headers

#include "Defines.h"

#ifndef VID
///ID for every vertex in graph.
typedef int VID;
#endif

////////////////////////////////////////////////////////////////////////////////////////////


#ifndef EID
typedef short EID;
#endif

#ifndef INVALID_VID
#define INVALID_VID    -999
#endif

#ifndef MAX_EDGE_LIST_SIZE
#define MAX_EDGE_LIST_SIZE 6
#endif

//In experimental use for MPI only, don't need to concern about it
#ifndef MAX_VERT_SIZE
#define MAX_VERT_SIZE 100
#endif

///WtEdge is the generic structure for edges in weighted graphs
template<class VERTEX, class WEIGHT> class WtEdgeType;
template<class VERTEX, class WEIGHT> class WtVertexType;

/////////////////////////////////////////////////////////////////////
//
//
//
//
//
//          WtEdgeType<VERTEX,WEIGHT> Definition
//
//
//
//
//
/////////////////////////////////////////////////////////////////////


/**WtEdge is the generic structure for edges in weighted graphs.
  *Its data includes:
  * -# the unique_id of the *second* endpoint of the edge (the 
  *    first endpoint is implicit due to the adjacency list rep) 
  * -# the edge WEIGHT
  *
  * Has friend class WtVertexType<VERTEX,WEIGHT>
  */


/////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
class WtEdgeType {

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Friend info
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    friend class WtVertexType<VERTEX,WEIGHT>;

public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{
       ///Do nothing
       WtEdgeType();
       /**Init data member by given parameters.
         *@param VID the id for the second endpoint
         *@param WEIGHT the weight for this edge.
         */
       WtEdgeType(VID, WEIGHT);
       ///Do nothing
      ~WtEdgeType();
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods. Use these method to read in/write out internal state*/
    //@{
        ///Out put Id of the second endpoint and weight to the standard output
        void DisplayEdge(int wt) const;
        ///Out put Id of the second endpoint and weight to the given output stream
        void WriteEdge(ostream&, int wt) const;
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    /**unique_id of the *second* endpoint of the edge.
      *start vertex (v1) is implicit in adj list rep.
      */
    VID     vertex2id;

    WEIGHT  weight; ///< Weight of this edge

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};


/////////////////////////////////////////////////////////////////////
//
//
//
//
//
//          WtVertexType<VERTEX,WEIGHT> Definition
//
//
//
//
//
/////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////
/**      WtVertexType is the generic structure for vertices in weighted graphs.
  *      Its data includes an edgelist for the vertex, and its methods 
  *      are responsible for adding/removing edges and weights. 
  *
  *      Is friend of class WtEdgeType<VERTEX,WEIGHT>.
  */
template<class VERTEX, class WEIGHT>
class WtVertexType {  

public:

    typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;
    typedef  vector< WtEdge > WtEdge_VECTOR;
    typedef  typename WtEdge_VECTOR::iterator EI;
    typedef  typename WtEdge_VECTOR::const_iterator CEI;
    typedef  typename WtEdge_VECTOR::reverse_iterator REI;
    typedef  typename WtEdge_VECTOR::const_reverse_iterator CREI;

    typedef  vector< pair<VID, WEIGHT> > PAIR_VECTOR;
    typedef  typename PAIR_VECTOR::iterator PEI;
    typedef  typename PAIR_VECTOR::const_iterator CPEI;  
    typedef  typename PAIR_VECTOR::reverse_iterator RPEI;
    typedef  typename PAIR_VECTOR::const_reverse_iterator CRPEI;


      WtVertexType();
      WtVertexType(VERTEX&, VID);       // don't reserve space for edges
      WtVertexType(VERTEX&, VID, int);  // 'reserve' space for edges
      ~WtVertexType();

      void AddEdge(VID, WEIGHT); 
      void AddEdgewCheck(VID, WEIGHT); 
      void AddPredecessorEdge(VID, WEIGHT);
      int  DeleteXEdges(VID, int); 
      int  DeleteXEdges(VID, WEIGHT, int); 
      bool IsEdge(VID) const ; 
      bool IsEdge(VID, CEI*) const;
      bool IsEdge(VID, EI*) ;
      bool IsEdge(VID, WEIGHT, CEI* _ei) const;
      bool IsEdge(VID, WEIGHT, EI* _ei);

      VERTEX  GetData() const; 
      int  GetVID() const;
      int  GetEdgeCount() const;
      WEIGHT  GetEdgeWeight(VID) const; 

      void DisplayEdgelist(int wt) const;
      void WriteEdgelist(ostream&,int wt) const;

  //===================================================================
  /**@name  Utility Stuff*/
  //===================================================================
  //@{

    //typedef typename edge_iterator_ud<vector<WtEdge>::iterator,WtEdge,vector<WtEdge>::difference_type> UEI;
    // Predicates

    /**check if _eid is an edge in the list, edgelist.
      *@return WtEdge which has this ID
      */
    CEI my_find_EID1_eq(const EID _eid) const; 
    EI my_find_EID1_eq(const EID _eid); 

    /**check if _eid is an edge in the list starting from _start to _end.
      *@return WtEdge which has this ID
      */
    CEI my_find_EID2_eqc(CEI _start, CEI _end, const EID _eid) const; 
    EI my_find_EID2_eq(EI _start, EI _end, const EID _eid) ; 

    /**check if any edge in the list, edgelist, has weight, _wt.
      *@return WtEdge which has this weight
      */
    CEI my_find_EWT_eq(const WEIGHT _wt) const; 

    /**check if any edge in the list, edgelist, has weight, _wtpair.second
      *and ID, _wtpair.first
      *@return WtEdge which has this weight
      */
    CEI my_find_EIDWT_eqc(const pair<VID,WEIGHT> _wtpair) const;
    EI my_find_EIDWT_eq(const pair<VID,WEIGHT> _wtpair) ;


  //@}

  // NMA: predicates below don't work with sgi/CC, but do for sun/g++
  //class EID_eq;
  //class EWT_eq;
  //class EIDWT_eq;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  VERTEX data; ///<Vertex Data e.g., cfg, data here...
  VID    vid;  ///< This vertex's unique id (not nec. index)
  vector< WtEdge > edgelist; ///< Adj list rep of graph
  vector< WtEdge > predecessors; ///< call BaseGraph::SetPredecessors() to initialize
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
};


/////////////////////////////////////////////////////////////////////
//
//
//
//
//
//          AbstractGraph<VERTEX> Definition
//
//
//
//
//
/////////////////////////////////////////////////////////////////////

/**   Abstract base class for all graphs.
  *   It is an abstract class (contains pure virtual functions)
  *   and serves as the interface to the derived classes
  *   representing various types of graphs (e.g., undirected,
  *   directed, unweighted, weighted, or multigraphs).
  */

template<class VERTEX> 
class AbstractGraph {

public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{

       ///Constrcutor. Set every thing to zero, NULL, and false.
       AbstractGraph();

       /**Constrcutor with reservation.
         *Set reserveEdgesPerVertex to the given value and 
         *set other data members to zero, NULL, and false. 
         *@param int number of edges per vertex ('reserve' space)
         */
       AbstractGraph(int);

       ///Do nothing
       ~AbstractGraph();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Access Methods*/
    //@{

       ///////////////////////////////////////////////////////////////////////////////////////////
       //
       // Adding & Finding & Deleting Vertices
       //
       ///////////////////////////////////////////////////////////////////////////////////////////

       ///Abstract
       virtual VID  AddVertex(VERTEX&) = 0;
       ///Abstract
       virtual VID  AddVertex(vector<VERTEX>&) = 0;
       ///Abstract
       virtual int  DeleteVertex(VID) = 0;
       ///Abstract
       virtual int  DeleteVertex(VERTEX&) = 0;
       ///Abstract
       virtual void DeleteAllEdges(VID) = 0; // delete all incident edges 

       ///////////////////////////////////////////////////////////////////////////////////////////
       //
       // Finding Vertices & Edges
       //
       ///////////////////////////////////////////////////////////////////////////////////////////

       ///Abstract
       virtual bool IsVertex(VID) const = 0;
       ///Abstract
       //virtual bool IsEdge(VID, VID) const = 0; 
       ///Abstract
       virtual bool IsVertex(VERTEX&)  const = 0;
       ///Abstract
       //virtual bool IsEdge(VERTEX&, VERTEX&) const  = 0; 

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O (Display, Input, Output)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods*/
    //@{

       ///Abstract
       virtual void DisplayGraph() const = 0; 
       ///Abstract
       virtual void DisplayVertexAndEdgelist(VID) const = 0; 

    //@}

	 
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

   VID vertIDs;  ///< used to give each vertex unique identifier     
   int numVerts; ///<How many vertices are in this graph
   int numEdges; ///<How many edges are in this graph   
   int reserveEdgesPerVertex; ///< used to 'reserve' space in edgelist
   int directness;
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
public:
   inline void setDirectness(int d){
     directness=d;
   } 
   inline int isDirected(){
     return directness;
   }
};



/**Derived from AbstractGraph<VERTEX,WEIGHT>.
  *
  *The BaseGraph class:
  * -# is a *directed* weighted graph
  * -# allows multiple vertices with the same VERTEX data
  * -# allows multiple (v1id,v2id) edges with *different* WEIGHTs
  * -# allows multiple (v1id,v2id) edges with *same* WEIGHT
  *
  *The graph is represented by an adjacency list structure.
  *
  */

template<class VERTEX, class WEIGHT>
class BaseGraph : public AbstractGraph<VERTEX> {
public:
  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{

       /**Constructor. Do nothing.
         *@note this constrcutor didn't  reserve any space for verts or edges
         */
       BaseGraph();

       /**Constructor. 'reserve' space for vertices.
         *@param int how many vertices will be reserved.
         */
       BaseGraph(int);

       /**Constrcutor. 'reserve' space for vertices.
         *@param first_int how many vertices will be reserved.
         *@param first_int how many edges will be reserved.
         */
       BaseGraph(int,int);

       /**Destrcutor. Do nothing.
         */
       ~BaseGraph();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Adding & Deleting Vertices
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Adding & Deleting Vertices*/
   //@{
       virtual VID  AddVertex(VERTEX&);
       virtual VID  AddVertex(vector<VERTEX>&);
       int  DeleteVertex(VID);
       int  DeleteVertex(VERTEX&);
       int  EraseGraph(); 
   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Adding & Deleting Edges
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Adding & Deleting Edges*/
   //@{
        
       int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>);
       int  AddEdge(VID, VID, WEIGHT);
       int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>);
       int  AddEdge(VERTEX&, VERTEX&, WEIGHT);  
       int  DeleteEdge(VID, VID, int n=-1); // default, delete all
       int  DeleteEdge(VERTEX&,VERTEX&, int n=-1); //default, delete all
       int  DeleteWtEdge(VID, VID, WEIGHT, int n=-1); // default, delete all
       int  DeleteWtEdge(VERTEX&,VERTEX&,WEIGHT, int n=-1); //default, delete all

       void DeleteAllEdges(VID);
       void DeleteAllEdges(VERTEX&);
       
   //@}
   ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Search methods (Finding Vertices & Edges)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Search methods.
     *Finding Vertices & Edges.
     */
   //@{
       bool IsVertex(VID) const;
       bool IsVertex(VERTEX&) const ;
       //bool IsEdge(VID, VID) const;
       //bool IsEdge(VERTEX&, VERTEX&) const;
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access methods (Getting Data & Statistics, global information)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Adding & Deleting Edges.
     *Getting Data & Statistics, global information.
     */
   //@{

       int GetVerticesVID(vector<VID>&) const;
       int GetVerticesVID(vector<VID>&, VID _vid,int n) const;
       int GetVerticesData(vector<VERTEX>& ) const;
       int GetVerticesData(vector<VERTEX>&, VID _vid,int n) const;

       inline VERTEX  GetData(VID) const;
       VERTEX* GetReferenceofData(VID); 
       int GetData(vector<VERTEX>&, VID _v1id, VID _v2id) const;
       VID     GetVID(VERTEX&) const ;
       void PutData(VID, VERTEX);
       int GetVertexCount() const;
       VID  GetNextVID() const;
   //@}

   //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Typedefs
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

   
  ///Modified for VC
  /**This seems VC is not be able to use vector<>::iterator directly without
    *"using namespace std;". how every this causes bigger problem.
    *therefore, one more typedef is used here
    */
       
   typedef vector< Vertex > VERTEX_VECTOR;
   typedef typename VERTEX_VECTOR::iterator VI;   ///<VI Vertex Iterator
   typedef typename VERTEX_VECTOR::const_iterator CVI;           ///<CVI Constant Vertex Iterator
   typedef typename VERTEX_VECTOR::reverse_iterator RVI;         ///<RVI Reverse Vertex Iterator
   typedef typename VERTEX_VECTOR::const_reverse_iterator CRVI;  ///<CRVI Constant Reverse Vertex Iterator

   typedef vector< WtEdge > WtEdge_VECTOR;
   typedef typename WtEdge_VECTOR::iterator EI;                  ///<EI Edge Iterator
   typedef typename WtEdge_VECTOR::const_iterator CEI;           ///<CEI Constant Edge Iterator
   typedef typename WtEdge_VECTOR::reverse_iterator REI;         ///<REI Reverse Edge Iterator
   typedef typename WtEdge_VECTOR::const_reverse_iterator CREI;  ///<CREI Constant Reverse Edge Iterator


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected: Adding & Deleting Vertices & Edges
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  /**@name Adding & Deleting Vertices & Edges
    */

   //@{

       /**Add a new (isolated) vertex to graph with specifed VID.
         *
         *@param VERTEX& the user specified data that will be stored in this 
         *new vertex
         *@return vertex id (not nec. index) which is given by client.
         *@see AddVertex(VERTEX&)
         *@note this method won't create a new id for this new vertex.
         */
       VID  AddVertex(VERTEX&,VID);

       /**Create a new edge from vid to EI->vertex2id.
         *Acutelly, this edgae, EI, has been created, this method
         *just adds this edge to vid's edge list.
         *@return ERROR if there is no vertex of given VID in graph.
         *OK is every thing is fine.
         */
       int AddEdge(VID, EI);


       /**Delete All edges connection to VID.
         *If there is no vertex of given VID in graph,
         *then nothing will be done.
         */
       virtual void DeleteAllEdgesToV(VID);

       /**Delete All edges connection to v.
         *Here v is any vertex contains user data in the parameter
         *if there are more than one, then the first will be applied.
         *If there is no vertex of given user data in graph,
         *then nothing will be done.
         */
       virtual void DeleteAllEdgesToV(VERTEX&); 

       /**Delete All edges going out from VID.
         *If there is no vertex of given VID in graph,
         *then nothing will be done.
         */
       virtual void DeleteAllEdgesFromV(VID);

       /**Delete All edges going out from v.
         *Here v is any vertex contains user data in the parameter
         *if there are more than one, then the first will be applied.
         *If there is no vertex of given user data in graph,
         *then nothing will be done.
         */
       virtual void DeleteAllEdgesFromV(VERTEX&); 

   //@}
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected: Search methods (Finding Vertices & Edges)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Search methods.
     *Finding Vertices & Edges.
     */
   //@{

       /**Check if given VID is in graph.
         *If the answer is yes then Vertex** will be the pointer
         *pointing to this vertex.
         *
         *@return true if given VID is in graph. Otherwise return false.
         *@see IsVertex(VID)
         */
       bool IsVertex(VID, CVI*) const;
       bool IsVertex(VID, VI*) ;
       /**Check if given v with specified user data is in graph.
         *If the answer is yes then Vertex** will be the pointer
         *pointing to this vertex.
         *
         *@return true if given VID is in graph. Otherwise return false.
         *@see IsVertex(VERTEX&)
         */
       bool IsVertex(VERTEX&, CVI*) const;
       bool IsVertex(VERTEX&, VI*) ;

       /**Check if there is any edge connected from vid1 to vid2.
         *If the answer is yes then Vertex** will be the pointer
         *pointing to the vertex where the edge from, and WtEdge**
         *will be the pointer pointing to this edge.
         *
         *@return false if vid1 or vid2 are not in graph, or
         *there is no edge from vid1 to vid2.
         *@see IsEdge(VID, VID)
         */
       bool IsEdge(VID, VID, VI*, EI*) ;
       /**Check if there is any edge connected from v1 to v2.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *
         *If the answer is yes then Vertex** will be the pointer
         *pointing to the vertex where the edge from, and WtEdge**
         *will be the pointer pointing to this edge.
         *
         *@return false if v1 or v2 are not in graph, or
         *there is no edge from v1 to v2.
         *@see IsEdge(VERTEX&, VERTEX&)
         */
       bool IsEdge(VERTEX&, VERTEX&, VI*, EI*) ;

       /**Check if there is any edge connected from vid1 to vid2 of specified weight.
         *If the answer is yes then Vertex** will be the pointer
         *pointing to the vertex where the edge from, and WtEdge**
         *will be the pointer pointing to this edge.
         *
         *@return false if vid1 or vid2 are not in graph, or
         *there is no edge from vid1 to vid2.
         */
       bool IsEdge(VID, VID, WEIGHT, VI*, EI*);

       /**Check if there is any edge connected from v1 to v2 of specified weight.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *
         *If the answer is yes then Vertex** will be the pointer
         *pointing to the vertex where the edge from, and WtEdge**
         *will be the pointer pointing to this edge.
         *
         *@return false if v1 or v2 are not in graph, or
         *there is no edge from v1 to v2.
         */
       bool IsEdge(VERTEX&, VERTEX&, WEIGHT, VI*, EI*) ;

       /**Find vertex of _vid in graph.
         *If no such vertex, then v.end() will be returned.
         */
       VI my_find_VID_eq(const VID _vid) ;
       CVI my_find_VID_eqc(const VID _vid) const ;
       /**Find vertex of user data _v in graph.
         *If no such vertex, then v.end() will be returned.
         *@note this method compares every vertex in graph.
         */
       VI my_find_VDATA_eq(const VERTEX& _v) ;
       CVI my_find_VDATA_eqc(const VERTEX& _v) const ;

       /**Is _v1's VID smaller than _v2's VID?.
         *@return (_v1.vid < _v2.vid )
         */

#ifdef __HP_aCC
       static bool VID_Compare (const Vertex _v1, const Vertex _v2); 
#else 
       static bool VID_Compare (const Vertex& _v1, const Vertex& _v2); 
#endif

   //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected: DATA
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    vector< Vertex >    v;  ///< vertices (with adj lists)

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data members and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};


//=====================================================================
//=====================================================================
//  METHODS FOR GRAPH RELATED CLASSES
//=====================================================================
//=====================================================================

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  METHODS FOR template AbstractGraph Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////


//==================================
// AbstractGraph class Methods: Constructors and Destructor
//==================================
template<class VERTEX> 
AbstractGraph<VERTEX>::
AbstractGraph() {
    vertIDs=numVerts=numEdges=0;
    reserveEdgesPerVertex = 0;
}

template<class VERTEX> 
AbstractGraph<VERTEX>::
AbstractGraph(int _reserveEdgesPerVertex) {
    vertIDs=numVerts=numEdges=0;
    reserveEdgesPerVertex = _reserveEdgesPerVertex;
}

template<class VERTEX> 
AbstractGraph<VERTEX>::
~AbstractGraph() {
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  METHODS FOR template BaseGraph Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////


//==================================
// BaseGraph class Methods: Constructors and Destructor
//==================================

template<class VERTEX,class WEIGHT> 
BaseGraph<VERTEX,WEIGHT>::
BaseGraph(){
}

template<class VERTEX,class WEIGHT> 
BaseGraph<VERTEX,WEIGHT>::
BaseGraph(int _sz) {
    v.reserve(_sz);
}

template<class VERTEX,class WEIGHT> 
BaseGraph<VERTEX,WEIGHT>::
BaseGraph(int _sz, int _edgelistsz)
: AbstractGraph<VERTEX> (_edgelistsz) 
{
    v.reserve(_sz);
}

template<class VERTEX, class WEIGHT> 
BaseGraph<VERTEX,WEIGHT>:: 
~BaseGraph(){
}


//==================================
// BaseGraph class Methods: Adding & Deleting Vertices
//==================================

template<class VERTEX, class WEIGHT> 
VID 
BaseGraph<VERTEX,WEIGHT>::
AddVertex(VERTEX& _v) {
    VID vid = this->vertIDs++;
    Vertex newVertex(_v,vid,this->reserveEdgesPerVertex);
    v.push_back(newVertex);
    this->numVerts++;
    return (vid); // return vertex id (not nec. index)
}

template<class VERTEX, class WEIGHT>
VID
BaseGraph<VERTEX,WEIGHT>::
AddVertex(vector<VERTEX>& _v) {
    
    if (_v.size()>0) {
        VID vertex_id = AddVertex(_v[0]);
        for (int i=1;i<_v.size();++i)
            AddVertex(_v[i]);
        return (vertex_id); // return vertex id (not nec. index)
    }
    return INVALID_VID;
    
}

template<class VERTEX, class WEIGHT> 
VID 
BaseGraph<VERTEX,WEIGHT>::
AddVertex(VERTEX& _v, VID _vid) {
    VID vid = _vid;
    Vertex newVertex(_v,vid,this->reserveEdgesPerVertex);
    v.push_back(newVertex);
    this->numVerts++;
    return (vid); // return vertex id (not nec. index)
}


template<class VERTEX, class WEIGHT> 
int 
BaseGraph<VERTEX,WEIGHT>::
DeleteVertex(VERTEX& _v1) {
    VI v1;
    if ( IsVertex(_v1,&v1) ) { 
        DeleteAllEdgesToV(v1->vid);
        v.erase(v1);
        this->numVerts--;
        return OK;
    } else {
#ifndef QUIETGRAPH
        cout << "\nDeleteVertex: vertex not in graph";
#endif
        return ERROR;
    }
}

template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteVertex(VID _v1id) {
    VI v1;
    if ( IsVertex(_v1id,&v1) ) {
        DeleteAllEdgesToV(_v1id);
        v.erase(v1);
        this->numVerts--;
        return OK;
    } else {
#ifndef QUIETGRAPH
        cout << "\nDeleteVertex: vertex not in graph";
#endif
        return ERROR;
    }
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
EraseGraph() {
    while ( v.size() != 0 ) 
        v.pop_back();
    this->vertIDs = this->numVerts = this->numEdges = 0;
    return OK;
}


//==================================
// BaseGraph class Methods: Modifying Vertices
//void PutData(VID, VERTEX);
//void SetPredecessors();
//==================================

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>:: 
GetVertexCount() const {
  return v.size();
}

template<class VERTEX, class WEIGHT>
VID
BaseGraph<VERTEX,WEIGHT>:: 
GetNextVID() const {
    return this->vertIDs;
}

template<class VERTEX, class WEIGHT>
void
BaseGraph<VERTEX,WEIGHT>:: 
PutData(VID _vid, VERTEX _v){ 
    VI v1;
    if ( IsVertex(_vid,&v1) ) {
        v1->data = _v;
    }
}

//==================================
// BaseGraph class Methods: Adding & Deleting Edges
//==================================

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, EI _ei) {
    VI v1, v2;
    //CVI cv1, cv2;
    VID v2id = _ei->vertex2id;
    WEIGHT weight = _ei->weight;
    
    if (IsVertex(_v1id,&v1) && IsVertex(v2id,&v2) ) {
      v1->AddEdge(v2id,weight);
      this->numEdges++;
      return OK;
    } else {
#ifndef QUIETGRAPH
      cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
#endif
      return ERROR;
    }
}


template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, VID _v2id, WEIGHT _weight) {
  VI v1;
  if (IsVertex(_v1id,&v1) && IsVertex(_v2id) ) {
    v1->AddEdge(_v2id,_weight);
    this->numEdges++;
    return OK;
  } else {
#ifndef QUIETGRAPH
    cout << endl << "AddEdge: v1 " << _v1id << " and/or v2 " << _v2id << "not in graph" ;
#endif
    return ERROR;
    }
}


template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, VID _v2id, pair<WEIGHT,WEIGHT> _wtpair ) {
  VI v1,v2;
  if (IsVertex(_v1id,&v1) && IsVertex(_v2id,&v2) ) {
    v1->AddEdge(_v2id,_wtpair.first);
    v2->AddEdge(_v1id,_wtpair.second);
    this->numEdges += 2;
    return OK;
  } else {
#ifndef QUIETGRAPH
    cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
#endif
    return ERROR;
  }
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
  VI v1, v2;
  if (IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
    v1->AddEdge(v2->vid,_weight);
    this->numEdges++;
    return OK;
  } else {
#ifndef QUIETGRAPH
    cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
#endif
    return ERROR;
  }
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT> _wtpair) {
  VI v1, v2;
  if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
    v1->AddEdge(v2->vid,_wtpair.first);
    v2->AddEdge(v1->vid,_wtpair.second);
    this->numEdges += 2;
    return OK;
  } else {
#ifndef QUIETGRAPH
    cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
#endif
    return ERROR;
  }
}

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id, VI* _v1ptr, EI* _e12ptr)  {
    VI v1;
    EI e12;
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
    }
}

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id, WEIGHT _weight, VI* _v1ptr, EI* _e12ptr)  {
    VI v1;
    EI e12;
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
    }
}

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2, VI* _v1ptr, EI* _e12ptr)  {
    VI v1,v2;
    EI e12;
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
    }
}

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight, VI* _v1ptr, EI* _e12ptr)  {
    VI v1,v2;
    EI e12;
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
    }
}

template<class VERTEX, class WEIGHT>
void 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesToV(VID _v2id) {
    CVI v2;
    if ( IsVertex(_v2id,&v2) ) {
        for (VI vi = v.begin(); vi < v.end(); vi++) {
            this->numEdges -= vi->DeleteXEdges(_v2id,-1);
        }
    }
}

template<class VERTEX, class WEIGHT>
void 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesFromV(VID _v1id) {
  VI v1;
  if ( IsVertex(_v1id,&v1) ) {
    this->numEdges -= v1->edgelist.size();
    v1->edgelist.erase( v1->edgelist.begin(), v1->edgelist.end() );
  }
}

template<class VERTEX, class WEIGHT>
void 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteAllEdges(VID _vid) {
    DeleteAllEdgesToV(_vid);
    DeleteAllEdgesFromV(_vid);
}

// default: delete all edges (v1,v2), otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteEdge(VID _v1id, VID _v2id, int _n) {
    VI v1;
    if ( IsVertex(_v1id,&v1) ) {
      this->numEdges -= v1->DeleteXEdges(_v2id,_n);
      return OK;
    } else {
      return ERROR;
    }
}

// default: delete all edges (v1,v2) of specified weight, otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteWtEdge(VID _v1id, VID _v2id, WEIGHT _weight, int _n) {
    VI v1;
    if ( IsVertex(_v1id,&v1) ) {
        this->numEdges -= v1->DeleteXEdges(_v2id,_weight,_n);
        return OK;
    } else {
        return ERROR;
    }
}

template<class VERTEX, class WEIGHT>
void 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesToV(VERTEX& _v2) {
    CVI v2;
    if ( IsVertex(_v2,&v2) ) {
        for (VI vi = v.begin(); vi < v.end(); vi++) {
            this->numEdges -= vi->DeleteXEdges(v2->vid,-1);
        }
    }
}

template<class VERTEX, class WEIGHT>
void 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesFromV(VERTEX& _v1) {
  VI v1;
  if ( IsVertex(_v1,&v1) ) {
    this->numEdges -= v1->edgelist.size();
    v1->edgelist.erase( v1->edgelist.begin(), v1->edgelist.end() );
  }
}

template<class VERTEX, class WEIGHT>
void 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteAllEdges(VERTEX& _v) {
    DeleteAllEdgesToV(_v);
    DeleteAllEdgesFromV(_v);
}


// default: delete all edges (v1,v2), otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteEdge(VERTEX& _v1, VERTEX& _v2, int _n) {
    VI v1,v2;
    if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
      this->numEdges -= v1->DeleteXEdges(v2->vid,_n);
      return OK;
    } else {
      return ERROR;
    }
}

// default: delete all edges (v1,v2) of specified weight, otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteWtEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight, int _n) {
    VI v1,v2;
    if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
      this->numEdges -= v1->DeleteXEdges(v2->vid,_weight,_n);
      return OK;
    } else {
      return ERROR;
    }
}

//==================================
// BaseGraph class Methods: Finding Vertices & Edges
//==================================

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VID _v1id) const {
    CVI v1;
    return ( IsVertex(_v1id,&v1) );
}

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VID _v1id, CVI*  _v1ptr)  const {
  //??? constant or nonconstant
    CVI v1 = my_find_VID_eqc(_v1id);
    if (v1 != v.end() ) {
        *_v1ptr = v1;
        return true;        
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VID _v1id, VI*  _v1ptr) {
  //non constant stuff
    VI v1 = my_find_VID_eq(_v1id);
    if (v1 != v.end() ) {
        *_v1ptr = v1;
        return true;        
    } else {
        return false;
    }
}


template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VERTEX& _v1) const {
    CVI v1;
    return ( IsVertex(_v1,&v1) );
}


template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VERTEX& _v1, CVI*  _v1ptr) const{
  CVI v1 = my_find_VDATA_eqc(_v1);
  if (v1 != v.end() ) {
    *_v1ptr = v1;
    return true;
  } else {
    return false;
  }
}

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VERTEX& _v1, VI*  _v1ptr)  {
  //non constant stuff
    VI v1 = my_find_VDATA_eq(_v1);
    if (v1 != v.end() ) {
        *_v1ptr = v1;
        return true;
    } else {
        return false;
    }
}


//==================================
// BaseGraph class Methods: Getting Data & Statistics
//==================================

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
GetVerticesVID(vector<VID>& verts) const {
  verts.clear();
    verts.reserve( v.size() );
    for (CVI vi = v.begin(); vi < v.end(); vi++) {
        verts.push_back(vi->vid);
    }
    sort( verts.begin(),verts.end() );
    return verts.size();
}

template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>::
GetVerticesVID(vector<VID>& verts, VID _vid, int _n) const {
    verts.clear();
    verts.reserve( _n );
    CVI v1, v2;
    int i;
    if ( IsVertex(_vid,&v1) ) {
        for (i = 0, v2 = v1; i < _n && v2 < v.end(); i++, v2++) {
            verts.push_back(v2->vid);
        }
    } else {
#ifndef QUIETGRAPH
        cout << "\nIn GetVerticesVID(VID,int): no vertex VID=" << _vid << " in graph\n";
#endif
    }
    sort( verts.begin(),verts.end() );
    return verts.size();
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
GetVerticesData(vector<VERTEX>& verts) const {
  verts.clear();
    verts.reserve( v.size() );
    for (CVI vi = v.begin(); vi < v.end(); vi++) {
        verts.push_back(vi->data);
    }
    return verts.size();
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
GetVerticesData(vector<VERTEX>& verts, VID _vid, int _n) const {
    verts.clear();
    verts.reserve( _n );
    CVI v1, v2;
    int i;
    if ( IsVertex(_vid,&v1) ) {
        for (i = 0, v2 = v1; i < _n && v2 < v.end(); i++, v2++) {
            verts.push_back(v2->data);
        }
    } else {
#ifndef QUIETGRAPH
        cout << "\nIn GetVerticesData(VID,int): no vertex VID=" << _vid << " in graph\n";
#endif
    }
    return verts.size();
}



template<class VERTEX, class WEIGHT>
VERTEX
BaseGraph<VERTEX,WEIGHT>::
GetData(VID _v1id) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
        return v1->data;
    } else {
      return VERTEX(-1);
/*         return VERTEX::InvalidData();  */
    }
}

template<class VERTEX, class WEIGHT>
VERTEX*
BaseGraph<VERTEX,WEIGHT>::
GetReferenceofData(VID _v1id) {
    VI v1;
    if ( IsVertex(_v1id,&v1) ) {
        return (&v1->data);
    } else {
//	VERTEX vv = VERTEX::InvalidData();
        return NULL; 
    }
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
GetData(vector<VERTEX>& vset, VID _v1id, VID _v2id) const {
    CVI v1, v2;
    vset.clear();
    if ( IsVertex(_v1id,&v1) && IsVertex(_v2id,&v2) ) {
        for (VID i = _v1id; i <= _v2id; i++) { 
            vset.push_back(v1->data);
            v1++;
        }
        return vset.size();
    } else {
        return vset.size(); //in this case return an empty vector
    }
}


template<class VERTEX, class WEIGHT>
VID
BaseGraph<VERTEX,WEIGHT>::
GetVID(VERTEX& _v1) const {
    CVI v1;
    if ( IsVertex(_v1,&v1) ) {
        return v1->vid;
    } else {
        return INVALID_VID;
    }
}

  //==================================
  // BaseGraph class Predicates, Comparisons & Operations
  //==================================

template<class VERTEX, class WEIGHT>
typename vector<WtVertexType<VERTEX,WEIGHT> >::iterator
	      //WtVertexType<VERTEX,WEIGHT>*
BaseGraph<VERTEX,WEIGHT>:: 
my_find_VID_eq(const VID _vid)  {
    VI vi, startvi; 
    
    // find the spot to start looking, hopefully at v[_vid]
    //cout<<"----><"<<v.size()<<" "<<_vid<<endl;
    if(v.size()==0) return v.end();

    if ( v.size() > _vid ) {
        startvi = v.begin() + _vid;
    } else {
        startvi = v.end() - 1; 
    }
    
    // look back from v[_vid]
    vi = startvi;
    while ( vi >= v.begin()  ) {
        if ( vi->vid == _vid) {
            return ( vi );
        } else {
            vi--;
        }
    }
    
    // look forward from v[_vid]
    vi = startvi;
    while ( vi < v.end()  ) {
        if ( vi->vid == _vid) {
            return ( vi );
        } else {
            vi++;
        }
    }
    
    // if didn't find it return v.end() like STL find
    return v.end() ;
}

template<class VERTEX, class WEIGHT>
typename vector<WtVertexType<VERTEX,WEIGHT> >::const_iterator
	      //WtVertexType<VERTEX,WEIGHT>*
BaseGraph<VERTEX,WEIGHT>:: 
my_find_VID_eqc(const VID _vid) const {
    CVI vi, startvi; 
    
    // find the spot to start looking, hopefully at v[_vid]
    //cout<<"----><"<<v.size()<<" "<<_vid<<endl;
    if(v.size()==0) return v.end();

    if ( v.size() > _vid ) {
        startvi = v.begin() + _vid;
    } else {
        startvi = v.end() - 1; 
    }
    
    // look back from v[_vid]
    vi = startvi;
    while ( vi >= v.begin()  ) {
        if ( vi->vid == _vid) {
            return ( vi );
        } else {
            vi--;
        }
    }
    
    // look forward from v[_vid]
    vi = startvi;
    while ( vi < v.end()  ) {
        if ( vi->vid == _vid) {
            return ( vi );
        } else {
            vi++;
        }
    }
    
    // if didn't find it return v.end() like STL find
    return v.end() ;
}

template<class VERTEX, class WEIGHT>
typename vector<WtVertexType<VERTEX,WEIGHT> >::iterator
BaseGraph<VERTEX,WEIGHT>::
my_find_VDATA_eq(const VERTEX& _v)  {
    VI vi = v.begin();
    bool found = false;
    while (vi != v.end() && !found) {
      if ( vi->data == _v) {
	found = true;
      } else {
	vi++;
      }
    }
    return (vi);
}

template<class VERTEX, class WEIGHT>
typename vector<WtVertexType<VERTEX,WEIGHT> >::const_iterator
BaseGraph<VERTEX,WEIGHT>::
my_find_VDATA_eqc(const VERTEX& _v) const  {
    CVI vi = v.begin();
    bool found = false;
    while (vi != v.end() && !found) {
      if ( vi->data == _v) {
	found = true;
      } else {
	vi++;
      }
    }
    return (vi);
}


template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
#ifdef __HP_aCC
VID_Compare (const Vertex _v1, const Vertex _v2){
#else 
VID_Compare (const Vertex& _v1, const Vertex& _v2){
#endif
    return (_v1.vid < _v2.vid ) ; 
}


///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  METHODS FOR template WtVertexType Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//==================================
// WtVertexType class Methods: Constructors and Destructor
//==================================

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
WtVertexType(){
}

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
WtVertexType(VERTEX& _data, VID _id){
    data = _data;
    vid = _id;
}

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
WtVertexType(VERTEX& _data, VID _id, int _edgelistsz){
    data = _data;
    vid = _id;
    edgelist.reserve( _edgelistsz );
}


template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
~WtVertexType(){
}

//==================================
// Vertex class Methods: Adding & Deleting Edges
//==================================

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>:: 
AddEdge(VID _v2id, WEIGHT _weight) {
     WtEdge newEdge(_v2id, _weight);
     edgelist.push_back(newEdge);
}

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>:: 
AddEdgewCheck(VID _v2id, WEIGHT _weight) {
  bool found = false;
  for(EI ei=edgelist.begin(); ei!=edgelist.end(); ei++) {
    if(_v2id == ei->vertex2id) found=true;
  }
  if(!found) {
     WtEdge newEdge(_v2id, _weight);
     edgelist.push_back(newEdge);
  }

}

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>:: 
AddPredecessorEdge(VID _v0id, WEIGHT _weight) {
     WtEdge newEdge(_v0id, _weight);
     predecessors.push_back(newEdge);
}


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
    EI ei = my_find_EID1_eq(_v2id);
    while ( (ei != edgelist.end()) && (num_deleted < num_to_delete) ) {
        edgelist.erase(ei);
        num_deleted++;
        ei = my_find_EID2_eq(ei,edgelist.end(),_v2id);
    }
    return num_deleted;
}

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
    EI ei = my_find_EID1_eq(_v2id);
    while ( (ei != edgelist.end()) && (num_deleted < num_to_delete) ) {
        if (ei->weight == _weight) {
            edgelist.erase(ei);
            num_deleted++;
            ei = my_find_EID2_eq(ei,edgelist.end(),_v2id);
        } else {
            ei = my_find_EID2_eq(ei+1,edgelist.end(),_v2id);
        }
    }
    return num_deleted;
}


//==================================
// Vertex class Methods: Finding Edges
//==================================

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id) const {
    EI ei;
    return ( IsEdge(_v2id, &ei) );
}

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id, CEI* _ei) const {
    CEI ei = my_find_EID1_eq(_v2id);
    if (ei != edgelist.end() ) {
        *_ei = ei;
        return true;
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id, EI* _ei)  {
    EI ei = my_find_EID1_eq(_v2id);
    if (ei != edgelist.end() ) {
        *_ei = ei;
        return true;
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id, WEIGHT _weight, CEI* _ei) const {
    CEI ei = my_find_EIDWT_eqc( pair<VID,WEIGHT>(_v2id,_weight) );
    if (ei != edgelist.end() ) {
        *_ei = ei;
        return true;
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id, WEIGHT _weight, EI* _ei)  {
    EI ei = my_find_EIDWT_eq( pair<VID,WEIGHT>(_v2id,_weight) );
    if (ei != edgelist.end() ) {
        *_ei = ei;
        return true;
    } else {
        return false;
    }
}
//==================================
// Vertex class Methods: Getting Data & Statistics 
//==================================

template<class VERTEX, class WEIGHT>
VERTEX 
WtVertexType<VERTEX,WEIGHT>::
GetData() const  {
    return data;
}

template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
GetVID() const {
    return vid;
}

template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
GetEdgeCount() const {
    return edgelist.size();
}

template<class VERTEX, class WEIGHT>
WEIGHT 
WtVertexType<VERTEX,WEIGHT>::
GetEdgeWeight(VID _v2id) const {
    CEI ei;
    if (IsEdge(_v2id,&ei)) {
        return ei->weight;
    } else {
#ifndef QUIETGRAPH
        cout << "\nGetEdgeWeight: edge not in graph";
#endif
        //return WEIGHT::InvalidWeight();
	//???? this has to be fixed
	return WEIGHT(-1);
    }
}

//==================================
// Vertex class Methods: Display, Input, Output 
//==================================

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>::
DisplayEdgelist(int wt) const {
    cout << "id =" << setw(3) << vid << ", data = ["; 
    cout << data;
    cout << "], edges={";
    for (CEI ei = edgelist.begin(); ei < edgelist.end(); ei++) {
        ei->DisplayEdge(wt);
        if (ei != edgelist.end() - 1) cout << ", ";
    }
    cout << "} \n";
}

template<class VERTEX, class WEIGHT>
void
WtVertexType<VERTEX,WEIGHT>::
WriteEdgelist(ostream& _myostream,int wt) const {
    
    _myostream << vid << " "; 
    _myostream << data << " "; 
    _myostream << edgelist.size() << " "; 
    
    for (CEI ei = edgelist.begin(); ei != edgelist.end(); ei++) { 
        ei->WriteEdge(_myostream,wt);
        _myostream << " "; 
    }
}

//==================================
// Vertex class Predicate Utilities
//==================================

template<class VERTEX, class WEIGHT>
  //WtEdgeType<VERTEX,WEIGHT>*
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::const_iterator
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
    return (ei);
}

template<class VERTEX, class WEIGHT>
  //WtEdgeType<VERTEX,WEIGHT>*
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::iterator
WtVertexType<VERTEX,WEIGHT>::
my_find_EID1_eq(const EID _eid) {
    EI ei = edgelist.begin();
    bool found = false;
    while (ei != edgelist.end() && !found) {
        if ( ei->vertex2id == _eid) {
            found = true;
        } else {
            ei++;
        }
    }
    return (ei);
}

template<class VERTEX, class WEIGHT>
  //WtEdgeType<VERTEX,WEIGHT>*
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::const_iterator
WtVertexType<VERTEX,WEIGHT>::
my_find_EID2_eqc(CEI _start, 
                CEI _end,
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
    return (ei);
}

template<class VERTEX, class WEIGHT>
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::iterator
WtVertexType<VERTEX,WEIGHT>::
my_find_EID2_eq(EI _start, 
                EI _end,
                const EID _eid) {
    EI ei = _start;
    bool found = false;
    while (ei != _end && !found) {
        if ( ei->vertex2id == _eid) {
            found = true;
        } else {
            ei++;
        }
    }
    return (ei);
}

template<class VERTEX, class WEIGHT>
  //WtEdgeType<VERTEX,WEIGHT>*
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::const_iterator
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
    return (ei);
}

template<class VERTEX, class WEIGHT>
  //WtEdgeType<VERTEX,WEIGHT>*
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::const_iterator
WtVertexType<VERTEX,WEIGHT>::
my_find_EIDWT_eqc(const pair<VID,WEIGHT> _wtpair) const  {
   CEI cei = edgelist.begin();
   EI ei = const_cast<EI> (cei);
   
   bool found = false;
   while (ei != edgelist.end() && !found) {
      if ( ei->vertex2id==_wtpair.first && ei->weight == _wtpair.second ) {
         found = true;
      } else {
         ei++;
      }
   }
   return (ei);
}
template<class VERTEX, class WEIGHT>
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::iterator
WtVertexType<VERTEX,WEIGHT>::
my_find_EIDWT_eq(const pair<VID,WEIGHT> _wtpair)  {
   EI ei = edgelist.begin();
   // EI ei = const_cast<EI> (cei);
   
   bool found = false;
   while (ei != edgelist.end() && !found) {
      if ( ei->vertex2id==_wtpair.first && ei->weight == _wtpair.second ) {
         found = true;
      } else {
         ei++;
      }
   }
   return (ei);
}

/*--------------- NMA: these don't work with sgi/CC, but do for sun/g++
template<class VERTEX, class WEIGHT>
class 
WtVertexType<VERTEX,WEIGHT>::
EID_eq : public unary_function< WtEdgeType<VERTEX,WEIGHT>,bool> {
public:
explicit EID_eq(const VID i) : testid (i) {}
bool operator() (WtEdgeType<VERTEX,WEIGHT> e) {
return e.vertex2id == testid;
}
VID testid;
protected:
private:
}

  
    template<class VERTEX, class WEIGHT>
    class 
    WtVertexType<VERTEX,WEIGHT>::
    EWT_eq : public unary_function< WtEdgeType<VERTEX,WEIGHT>,bool> {
    public:
    explicit EWT_eq(const WEIGHT w) : testwt (w) {}
    bool operator() (WtEdgeType<VERTEX,WEIGHT> e) {
    return e.weight == testwt;
    }
    WEIGHT testwt;
    protected:
    private:
    }
    
      template<class VERTEX, class WEIGHT>
      class 
      WtVertexType<VERTEX,WEIGHT>::
      EIDWT_eq : public unary_function< WtEdgeType<VERTEX,WEIGHT>,bool> {
      public:
      explicit EIDWT_eq(const pair<VID,WEIGHT> eid) : testedge (eid) {}
      bool operator() (WtEdgeType<VERTEX,WEIGHT> e) {
      return ((e.vertex2id==testedge.first) && (e.weight == testedge.second));
      }
      pair<VID,WEIGHT> testedge;
      protected:
      private:
      }
*/

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  METHODS FOR template WtEdge Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//==================================
// WtEdge class Methods: Constructors and Destructor
//==================================

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>:: 
WtEdgeType(){
}

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>:: 
WtEdgeType(VID _v2id, WEIGHT _weight){
    vertex2id = _v2id;
    weight = _weight;
}

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>:: 
~WtEdgeType(){
}

//==================================
// WtEdge class Methods: Getting Data & Statistics
//==================================

//==================================
// WtEdge class Methods: Display, Input, Output
//==================================

template<class VERTEX, class WEIGHT>
void 
WtEdgeType<VERTEX,WEIGHT>:: 
DisplayEdge(int wt) const {
    cout << vertex2id;
    if (wt==1) cout << "(" << weight << ")";
} 

template<class VERTEX, class WEIGHT>
void
WtEdgeType<VERTEX,WEIGHT>::
WriteEdge(ostream& _myostream,int wt) const {
    _myostream << vertex2id<<" ";
    if(wt == 1) _myostream << weight << " ";
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  The End
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

#endif

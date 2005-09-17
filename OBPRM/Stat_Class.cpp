#include "Stat_Class.h"
#include "GraphAlgo.h"
/////////////////////////////////////////////////////////////////////
//
//  Stat_Class.c
//
//  General Description
//      This class lets you keep statistics on various aspects of
//      the program (ie. number of collision detection calls,
//      number of calls to each local planner, etc).
//
//  Created
//     1/27/99  Chris Jones
//
/////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
// Static Data Member init

const int Stat_Class::ALL= -1;


// Stat_Class Class Constructor
Stat_Class::
Stat_Class() {
  ClearStats();
};


// Stat_Class Class Deconstructor
Stat_Class::
~Stat_Class() {
};


// Statistic Methods

//----------------------------------------
//  Clear all stats
//----------------------------------------
void
Stat_Class::
ClearStats() {
  int i;

  NumNodes=0;
  NumEdges=0;

  // initialize the size of each connected component to 0
  NumCC=0;
  for(i=0;i<MaxCC;i++)
    SizeCC[i]=0;

  // initialize the number of collision detection calls to 0
  for(i=0;i<MaxCD;i++) {
    NumCollDetCalls[i]=0;
    strcpy(CDNameList[i],"empty");
  }

  // initialize each local planners successful connections, attempts,
  // and collision detection calls to 0
  for(i=0;i<MaxLP;i++) {
    LPConnections[i]=0;
    LPAttempts[i]=0;
    LPCollDetCalls[i]=0;
    strcpy(LPNameList[i],"empty");
  }

  Connections_Attempted = 0;
  Connections_Made = 0;
  Nodes_Attempted = 0;
  Nodes_Generated = 0;
  cc_number = 0;

  avg_min_intracc_dist = 0;
  avg_max_intracc_dist = 0;
  avg_mean_intracc_dist = 0;
  avg_sigma_intracc_dist = 0;

  avg_min_intracc_edge_s = 0;
  avg_max_intracc_edge_s = 0;
  avg_mean_intracc_edge_s = 0;
  avg_sigma_intracc_edge_s = 0;

  avg_max_intracc_dist_to_cm = 0;
  avg_min_intracc_dist_to_cm = 0;
  avg_mean_intracc_dist_to_cm = 0;
  avg_sigma_intracc_dist_to_cm = 0;

  max_intercc_dist = 0.0;
  avg_intercc_dist = 0.0; 
  sigma_intercc_dist = 0.0;
  min_intercc_dist = 100000.0;

  max_cc_size = 0.0;
  min_cc_size = 0.0;
  avg_cc_size = 0.0;
  sigma_cc_size = 0.0;

};

//----------------------------------------
// Increment the number of nodes
//----------------------------------------
int 
Stat_Class::
IncNumNodes() {
  NumNodes++;
  return(NumNodes);
};

//----------------------------------------
// Set the number of nodes to Nodes
//----------------------------------------
int 
Stat_Class::
SetNumNodes( int Nodes ) {
  NumNodes=Nodes;
  return(NumNodes);
};

//----------------------------------------
// Increment the number of edges in the map
//----------------------------------------
int 
Stat_Class::
IncNumEdges() {
  NumEdges++;
  return(NumEdges);
};

//----------------------------------------
// Set the number of edges in the map to Edges
//----------------------------------------
int 
Stat_Class::
SetNumEdges( int Edges ) {
  NumEdges=Edges;
  return(NumEdges);
};

//----------------------------------------
// Increment the number of connected components in the graph
//----------------------------------------
int 
Stat_Class::
IncNumCC() {
  NumCC++;
  return(NumCC);
};

//----------------------------------------
// Set the number of connected components in the graph to CC
//----------------------------------------
int 
Stat_Class::
SetNumCC( int CC ) {
  NumCC=CC;
  return(NumCC);
};

//----------------------------------------
// Increment the size of connected component number CC
//----------------------------------------
int 
Stat_Class::
IncSizeCC( int CC ) {
  if (CC<MaxCC) {
    SizeCC[CC]++;
    return(SizeCC[CC]);
  }
  else
    return(-1);
};

//----------------------------------------
// Set the size of connected component CC to Size
//----------------------------------------
int 
Stat_Class::
SetSizeCC( int CC, int Size ) {
  if (CC<MaxCC) { 
    SizeCC[CC]=Size;
    return(SizeCC[CC]);
  }
  else
    return(-1);
};

//----------------------------------------
// See if a collision detection scheme named CDName
// is already in the list of schemes, if so, return
// its list index
//----------------------------------------
int 
Stat_Class::
FindCD( char *CDName ) {
  int i;

  for(i=0;i<MaxCD;i++)
    if (strcmp(CDNameList[i],CDName) == 0)
      return(i);
    else
      if (strcmp(CDNameList[i],"empty") == 0) {
        strcpy(CDNameList[i],CDName);
        return(i);
      }
  return(-1);
};

//----------------------------------------
// Increment the number of collision detection
// calls for the method by the name of CDName
//----------------------------------------
int
Stat_Class::
IncNumCollDetCalls( char *CDName, std::string *pCallName){
  int CD;

  CD=FindCD( CDName );
  NumCollDetCalls[CD]++;
  
  // If a caller's name was provided
  //  then increment the verification counter
  //  with that name.
  //
  if( pCallName )
  { CollDetCountByName[*pCallName]++; }

  return(NumCollDetCalls[CD]);
};

//----------------------------------------
// See if a local planner scheme named LPName
// is already in the list of schemes, if so, return
// its list index
//----------------------------------------
int
Stat_Class::
FindLP( char *LPName ) {
  int i;

  for(i=0;i<MaxLP;i++)
    if (strcmp(LPNameList[i],LPName) == 0)
      return(i);
    else
      if (strcmp(LPNameList[i],"empty") == 0) {
        strcpy(LPNameList[i],LPName);
        return(i);
      }
  return(-1);
};

//----------------------------------------
// Increment the number of connections made
// by local planning method named LPName
//----------------------------------------
int 
Stat_Class::
IncLPConnections( char *LPName ) {
  int LP;

  LP=FindLP( LPName );
  LPConnections[LP]++;
  return(LPConnections[LP]);
};

//----------------------------------------
// Increment the number of connections made
// by local planning method named LPName by incr
//----------------------------------------
int
Stat_Class::
IncLPConnections( char *LPName ,int incr) {
  int LP;

  LP=FindLP( LPName );
  LPConnections[LP] += incr;
  return(LPConnections[LP]);
};

//----------------------------------------
// Decrement the number of connections made
// by local planning method named LPName
//----------------------------------------
int
Stat_Class::
DecLPConnections( char *LPName ) {
  int LP;
 
  LP=FindLP( LPName );
  LPConnections[LP]--;
  return(LPConnections[LP]);
};

//----------------------------------------
// Increment the number of connections made
// by local planning method named LPName by incr
//----------------------------------------
int
Stat_Class::
DecLPConnections( char *LPName, int decr ) {
  int LP;

  LP=FindLP( LPName );
  LPConnections[LP] -= decr;
  return(LPConnections[LP]);
};


//----------------------------------------
// Set the number of connections made by local
// planning method LPName to Connections
//----------------------------------------
int 
Stat_Class::
SetLPConnections( char *LPName, int Connections ) {
  int LP;

  LP=FindLP( LPName );
  LPConnections[LP]=Connections;
  return(LPConnections[LP]);
};

//----------------------------------------
// Increment the number attempts made by the
// local planning method named LPName by incr
//----------------------------------------
int
Stat_Class::
IncLPAttempts( char *LPName , int incr) {
  int LP;

  LP=FindLP( LPName );
  LPAttempts[LP] += incr;
  return(LPAttempts[LP]);
};

//----------------------------------------
// Increment the number attempts made by the
// local planning method named LPName
//----------------------------------------
int
Stat_Class::
IncLPAttempts( char *LPName ) {
  int LP;

  LP=FindLP( LPName );
  LPAttempts[LP]++;
  return(LPAttempts[LP]);
};

//----------------------------------------
// Decrement the number attempts made by the
// local planning method named LPName
//----------------------------------------
int
Stat_Class::
DecLPAttempts( char *LPName ) {
  int LP;

  LP=FindLP( LPName );
  LPAttempts[LP]--;
  return(LPAttempts[LP]);
};

//----------------------------------------
// Decrement the number attempts made by the
// local planning method named LPName by decr
//----------------------------------------
int
Stat_Class::
DecLPAttempts( char *LPName ,int decr) {
  int LP;

  LP=FindLP( LPName );
  LPAttempts[LP] -= decr;
  return(LPAttempts[LP]);
};

//----------------------------------------
// Set the number of attempts made by local
// planning method LPName to Attempts
//----------------------------------------
int
Stat_Class::
SetLPAttempts( char *LPName, int Attempts ) {
  int LP;

  LP=FindLP( LPName );
  LPAttempts[LP]=Attempts;
  return(LPAttempts[LP]);
};

//----------------------------------------
// Increment the number of collision detection
// calls made by local planning method named
// LPName
//----------------------------------------
int 
Stat_Class::
IncLPCollDetCalls( char *LPName ) {
  int LP;

  LP=FindLP( LPName );
  LPCollDetCalls[LP]++;
  return(LPCollDetCalls[LP]);
};

//----------------------------------------
// Increment the number of collision detection
// calls made by local planning method named
// LPName by incr
//----------------------------------------
int
Stat_Class::
IncLPCollDetCalls( char *LPName, int incr ) {
  int LP;

  LP=FindLP( LPName );
  LPCollDetCalls[LP] += incr;
  return(LPCollDetCalls[LP]);
};

//----------------------------------------
// Decrement the number of collision detection
// calls made by local planning method named
// LPName
//----------------------------------------
int
Stat_Class::
DecLPCollDetCalls( char *LPName ) {
  int LP;
 
  LP=FindLP( LPName );
  LPCollDetCalls[LP]--;
  return(LPCollDetCalls[LP]);
};

//----------------------------------------
// Decrement the number of collision detection
// calls made by local planning method named
// LPName by decr
//----------------------------------------
int
Stat_Class::
DecLPCollDetCalls( char *LPName, int decr ) {
  int LP;

  LP=FindLP( LPName );
  LPCollDetCalls[LP] -= decr;
  return(LPCollDetCalls[LP]);
};


void
Stat_Class::
PrintParams() {
};

void
Stat_Class::
PrintFeatures() {
  cout << "General features:" << endl;
  cout << "\tNodes_Attempted: " << Nodes_Attempted << endl;
  cout << "\tNodes_Generated: " << Nodes_Generated << endl;
  cout << "\tpct_free_nodes: " << ((double)Nodes_Generated)/Nodes_Attempted << endl;
  cout << "\tcc_number: " << cc_number << endl;
  cout << "\tConnections_Attempted: " << Connections_Attempted << endl;
  cout << "\tConnections_Made: " << Connections_Made << endl;
  cout << "\tpct_succesful_connections: " << ((double)Connections_Made)/Connections_Attempted << endl;

  cout << "Intra-cc features:" << endl;
  cout << "\tavg_min_intracc_dist: " << avg_min_intracc_dist << endl;
  cout << "\tavg_max_intracc_dist: " << avg_max_intracc_dist << endl;
  cout << "\tavg_mean_intracc_dist: " << avg_mean_intracc_dist << endl;
  cout << "\tavg_sigma_intracc_dist: " << avg_sigma_intracc_dist << endl;
  cout << "\tavg_min_intracc_edge_s: " << avg_min_intracc_edge_s << endl;
  cout << "\tavg_max_intracc_edge_s: " << avg_max_intracc_edge_s << endl;
  cout << "\tavg_mean_intracc_edge_s: " << avg_mean_intracc_edge_s << endl;
  cout << "\tavg_sigma_intracc_edge_s: " << avg_sigma_intracc_edge_s << endl;

  cout << "\tavg_max_intracc_dist_to_cm: " << avg_max_intracc_dist_to_cm << endl;
  cout << "\tavg_min_intracc_dist_to_cm: " << avg_min_intracc_dist_to_cm << endl;
  cout << "\tavg_mean_intracc_dist_to_cm: " << avg_mean_intracc_dist_to_cm << endl;
  cout << "\tavg_sigma_intracc_dist_to_cm: " << avg_sigma_intracc_dist_to_cm << endl;

  cout << "Inter-cc features:" << endl;
  cout << "\tmax_intercc_dist: " << max_intercc_dist << endl;
  cout << "\tmin_intercc_dist: " << min_intercc_dist << endl;
  cout << "\tavg_intercc_dist: " << avg_intercc_dist << endl;
  cout << "\tsigma_intercc_dist: " << sigma_intercc_dist <<endl<<endl;

  cout << "\tmin_cc_size: " << min_cc_size << endl;
  cout << "\tmax_cc_size: " << max_cc_size << endl;
  cout << "\tavg_cc_size: " << avg_cc_size << endl;
  cout << "\tsigma_cc_size: " << sigma_cc_size << endl;
}

void
Stat_Class::
IncNodes_Generated(){
  Nodes_Generated++;
};

void
Stat_Class::
IncNodes_Attempted(){
  Nodes_Attempted++;
};
void
Stat_Class::
IncConnections_Attempted(){
  Connections_Attempted++;
};
                                                                                
void
Stat_Class::
IncConnections_Made(){
  Connections_Made++;
};
                                                                                


// $Id$

#ifndef StatClass_h
#define StatClass_h

class RoadMap;

// Maximum number of connected components to keep track of
const int MaxCC=    100;

// Maximum number of local planners to keeps stats on
const int MaxLP=    10;

// Maximum number of collision detection algorithms to keep stats on
const int MaxCD=    10;

/**@file Stat_Class.h
   This class lets you keep statistics on various aspects of
   the program (ie. number of collision detection calls,
   number of calls to each local planner, etc).

   @date 1/27/99
   @author Chris Jones 
*/

class Stat_Class {
public:
  Stat_Class();
  ~Stat_Class();

  void ClearStats();
  int IncNumNodes();
  int SetNumNodes( int Nodes );
  int IncNumEdges();
  int SetNumEdges( int Edges );

  int IncNumCC();
  int SetNumCC( int CC );
  int IncSizeCC( int CC );
  int SetSizeCC( int CC, int Size );

  int IncNumCollDetCalls( char *CDName );
  int FindCD( char *CDName );

  int FindLP( char *LPName );
  int IncLPConnections( char *LPName , int);
  int IncLPConnections( char *LPName );
  int DecLPConnections( char *LPName , int);
  int DecLPConnections( char *LPName );
  int SetLPConnections( char *LPName, int Connections );
  int IncLPAttempts( char *LPName ,int );
  int IncLPAttempts( char *LPName );
  int DecLPAttempts( char *LPName ,int );
  int DecLPAttempts( char *LPName );
  int SetLPAttempts( char *LPName, int Attempts );
  int IncLPCollDetCalls( char *LPName );
  int IncLPCollDetCalls( char *LPName ,int);
  int DecLPCollDetCalls( char *LPName ); 
  int DecLPCollDetCalls( char *LPName ,int); 


  static const int ALL;
  void PrintAllStats( Roadmap *rmap, int numCCs=ALL);


  void PrintDataLine(ostream&, Roadmap * , int show_column_headers=0);
  void PrintParams();

protected:
  int NumNodes;
  int NumEdges;
  int NumCC;

  int NumCollDetCalls[MaxCD];
  char CDNameList[MaxCD][50];

  int SizeCC[MaxCC];

  char LPNameList[MaxLP][50];
  int LPConnections[MaxLP];
  int LPAttempts[MaxLP];
  int LPCollDetCalls[MaxLP];
};

#endif


// $Id$

#ifndef StatClass_h
#define StatClass_h

#include "Roadmap.h"

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
  template <class CFG, class WEIGHT>
  void PrintAllStats( Roadmap<CFG, WEIGHT> *rmap, int numCCs=ALL);

  template <class CFG, class WEIGHT>
  void PrintDataLine(ostream&, Roadmap<CFG, WEIGHT>* , int show_column_headers=0);
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

//definitions of templated functions
template <class CFG, class WEIGHT>
void
Stat_Class::
PrintAllStats( Roadmap<CFG, WEIGHT>* rmap, int numCCs) {
  int i;

  cout << endl << endl << "Local Planners:" << endl;
  cout << setw(20) << "Name"
	<<setw(15) << "Connections"
	<<setw(15) << "Attempts"
	<<setw(15) << "Coll Det Calls" << endl;
 
  for(i=0;i<MaxLP;i++)
    if (strcmp(LPNameList[i],"empty")!=0) {
      cout << setw(20) << LPNameList[i];
      cout << setw(15) << LPConnections[i];
      cout << setw(15) << LPAttempts[i];
      cout << setw(15) << LPCollDetCalls[i] << endl;
  }

  cout << endl << endl;
  cout << "Number of Nodes: " << rmap->m_pRoadmap->GetVertexCount() << endl;
  cout << "Number of Edges: " << rmap->m_pRoadmap->GetEdgeCount() << endl;

  cout << "Number of Collision Detection Calls: " << endl;
  for(i=0;i<MaxCD;i++)
    if (strcmp(CDNameList[i],"empty")!=0)
      cout << setw(20) << CDNameList[i] 
	   << setw(15) << NumCollDetCalls[i] << endl;


  #if VERBOSE
  #endif

  cout << endl;

  if (numCCs==ALL)    {DisplayCCStats(*(rmap->m_pRoadmap));      }
  else if (numCCs==0) {DisplayCCStats(*(rmap->m_pRoadmap),0);     }
  else                {DisplayCCStats(*(rmap->m_pRoadmap),numCCs);}

}

template <class CFG, class WEIGHT>
void
Stat_Class::
PrintDataLine(ostream& _myostream, Roadmap<CFG, WEIGHT> *rmap, int show_column_headers) {

   // Default is to NOT print out column headers
   if (show_column_headers){
        _myostream <<"\nV  E #CC 1 2 3 4 5 6 7 8 9 10 #iso CD  LPattSum LPcdSum\n";
   }//endif

   _myostream << rmap->m_pRoadmap->GetVertexCount() << " ";
   _myostream << rmap->m_pRoadmap->GetEdgeCount()   << " ";

   vector< pair<int,VID> > ccstats;
   GetCCStats(*(rmap->m_pRoadmap),ccstats);
   _myostream << ccstats.size() << "  ";
   int i;
   for (i=0;i<10;++i)
      if (ccstats.size() > i)
        _myostream << ccstats[i].first << " ";
      else
        _myostream << 0                << " ";

   int sumIsolatedNodes=0;
   for (i=0;i<ccstats.size();++i)
      if (ccstats[i].first == 1) ++sumIsolatedNodes;
   _myostream << sumIsolatedNodes << " ";

   int sumCDcalls=0;
   for(i=0;i<MaxCD;i++)
     if (strcmp(CDNameList[i],"empty")!=0)
        sumCDcalls += NumCollDetCalls[i];
   _myostream << sumCDcalls << " ";

   int sumAtt=0;
   int sumCD =0;
   for(i=0;i<MaxLP;i++){
     if (strcmp(LPNameList[i],"empty")!=0) {
       sumAtt += LPAttempts[i];
       sumCD  += LPCollDetCalls[i];
     }//endif
   }//endfor
   _myostream << sumAtt << " ";
   _myostream << sumCD  << " ";
   ccstats.clear();
}

#endif


#if !defined(_MAP_GENERATOR_H_)
#define _MAP_GENERATOR_H_

#include "Clock_Class.h"
#include "GenerateMapNodes.h"
#include "ConnectMap.h"

/////////////////////////////////////////////////////////////
//
//    Map Generator
//
/////////////////////////////////////////////////////////////

template <class CFG, class WEIGHT>
class MapGenerator {
 public:
 
  /////////////////////////////////
  // Constructor and Destructor
  /////////////////////////////////
  
  MapGenerator();
  ~MapGenerator();

  //main wrapper, called by main_obprm.cpp
  //if input.seedByChunk, calls GenerateIncrementalMap()
  //otherwise, calls GenerateNormalMap
  void GenerateMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
		     CollisionDetection* cd, 
		     DistanceMetric* dm, vector<CFG>& nodes, 
		     LocalPlanners<CFG,WEIGHT>* lp,
		     Input* input); 
  protected:
  
  virtual void GenerateIncrementalMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
		     CollisionDetection* cd, 
		     DistanceMetric* dm, vector<CFG>& nodes, 
		     LocalPlanners<CFG,WEIGHT>* lp,
		     Input* input,
		     bool addPartialEdge,
		     bool addAllEdges); 

  void GenerateNormalMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
		     CollisionDetection* cd, 
		     DistanceMetric* dm, vector<CFG>& nodes, 
		     LocalPlanners<CFG,WEIGHT>* lp,
		     Input* input,
		     bool addPartialEdge,
		     bool addAllEdges); 
  //Update NextNodeIndex from a map
  //It goes through all methods recorded in the map and update nextNodeIndex for each method
  //it also returns RNGseed
  void SetupFromMap(Input* input, Roadmap<CFG, WEIGHT>* rmap);
		     
  //////////////////////
  //Data
  //////////////////////
 
  public:
  
  GenerateMapNodes<CFG> gn;     //used to generate map nodes
  ConnectMap<CFG, WEIGHT> cm;   //used to connect map nodes
  GenerateMapNodes<CFG> old_gn; //used to parse command line from the existing map
 
};

template <class CFG, class WEIGHT>
MapGenerator<CFG, WEIGHT>::
MapGenerator(){}

template <class CFG, class WEIGHT>
MapGenerator<CFG, WEIGHT>::
~MapGenerator(){}


/*
 Set NextNodeIndex for each method we already used
 Set RNGseed for rmap
 Seed RNG using RNGseed, call OBPRM_srand()
 Update current command line 
 Backup the old map
*/
template <class CFG, class WEIGHT>
void
MapGenerator<CFG, WEIGHT>::
SetupFromMap(Input* input, Roadmap<CFG, WEIGHT>* rmap) 
{
    //get info from the existing map
    ifstream  myifstream(input->inmapFile.GetValue());
    long RNGseed;
    std::string tagstring;
    std::string oldCommandLine;

    getline(myifstream, tagstring);  //skip the first line
    getline(myifstream, tagstring);  //read in the second line 
    if (tagstring.find("PREAMBLESTART") == string::npos ) {
      cout << endl << "In ReadMapFile: didn't read PREAMBLESTART tag right";
      exit (-1);
    }
    getline(myifstream, oldCommandLine);  //read in the third line: old command line
    getline(myifstream, tagstring);       //read in the fourth line to double check
    if (tagstring.find("PREAMBLESTOP") == string::npos ) {
      cout << endl << "In ReadMapFile: didn't read PREAMBLESTOP tag right";
      exit (-1);
    }

    // skip over stuff up to DMSTOP, next line should contain RNGSEEDSTART
    bool moreFile=true;
    while(moreFile) {
      getline(myifstream, tagstring); 
      if (tagstring.find("DMSTOP") != string::npos ) 
        moreFile = false;
    } 
    
    getline(myifstream, tagstring); //Get the line after DMSTOP
    if (tagstring.find("RNGSEEDSTART") == string::npos ) {
      cout << endl << "WARNNING: In ReadMapFile, didn't read RNGSEEDSTART tag right";
      RNGseed = INVALID_RNGSEED;               //set default value for old version map
    } 
    else {
      myifstream >> RNGseed;        //read in RNGseed from the map
      cout<<"Read RNGseed from existing map file and it is: "<<RNGseed<<endl;
    }
    myifstream.close();
   
    
    //update data member in rmap
    rmap->SetRNGseed(RNGseed);
    //reset seed for OBPRM_srand
    if(RNGseed == INVALID_RNGSEED)
      OBPRM_srand(); //seed RNG using the default value
    else
      OBPRM_srand(RNGseed); //seed RNG using this seed

    //set NextNodeIndex for each method used in the existing map
    vector<std::string> gens;  //including "-gNodes"
     
    std::string genStr = "-gNodes";
  
    int current = oldCommandLine.find(genStr, 0);
    int next;
    while (current != std::string::npos) {
      next = oldCommandLine.find('-', current +1);
      if (next == std::string::npos) {
        gens.push_back(oldCommandLine.substr(current,oldCommandLine.size()));
        break;
      }
      else
       gens.push_back(oldCommandLine.substr(current, next-current));
       current = oldCommandLine.find(genStr, next);

    }
     
     int numOldGen = gens.size();
     n_str_param      *GNstrings[numOldGen];
     
     for(int i=0; i<numOldGen; i++) { 
       std::string tmpString = gens[i].substr(8,gens[i].size()) ; //get rid of "-gNodes "
       char *tmp = (char*) tmpString.c_str() ;
       GNstrings[i]=new n_str_param("-gNodes", tmp);
     }
     
     old_gn.ReadCommandLine(GNstrings, gens.size());
     int totalNumNodes = 0;   
     typename vector<NodeGenerationMethod<CFG>*>::iterator itr;
     
     cout<<"Information for nodes in the existing roadmap:"<<endl;
     for ( itr = old_gn.selected.begin(); itr != old_gn.selected.end(); itr++ ) {
       cout<<"  The node generation method was: "<<(*itr)->GetName()<<endl;
       //cout<<"The nextNodeIndex is:"<<(*itr)->GetNextNodeIndex()<<endl;
       cout<<"  Number of nodes were generated: "<<(*itr)->numNodes.GetValue()<<endl;
       totalNumNodes += ((*itr)->numNodes.GetValue());
       (*itr)->IncreaseNextNodeIndex((*itr)->numNodes.GetValue());
       cout<<"  The nextNodeIndex is set to:"<<(*itr)->GetNextNodeIndex()<<endl;
     }

     //update commandLine in the input
     for(int i=0; i<gens.size(); i++) {
      std::string tmpString = gens[i];
      char *tmp = (char*) tmpString.c_str() ;
      strcat(input->commandLine, tmp);
    }

    //backup the existing map
    //file name is: "oldfileName.XXX" where XXX is totalNumNodes
    char outfile[200];
    sprintf(outfile, "%s%s%d", input->inmapFile.GetValue(), ".", totalNumNodes);

    // open infile and outfile
    ifstream  myifstream2(input->inmapFile.GetValue());
    if (!myifstream2) {
      cout << endl << "In SaveCurrentVersion: cannot open infile: " << input->inmapFile.GetValue();
      return;
    }
    ofstream  myofstream(outfile);
    if (!myofstream) {
      cout << endl << "In SaveCurrentVersion: cannot open outfile: " << outfile;
      return;
    }
  
    while ( getline(myifstream2, tagstring) ){
      myofstream << tagstring << endl;
    }
 
}

/*main interface/wrapper to generate a map
  if seedByChunk is set, generate/connect nodes chunk by chunk
    call GenerateIncrementalMap (this is the default option)
  otherwise, use the regular method to generate a map
    call GenerateNormalMap
 */
template <class CFG, class WEIGHT>
void 
MapGenerator<CFG, WEIGHT>::
GenerateMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
	     CollisionDetection* cd, 
	     DistanceMetric* dm, vector<CFG>& nodes, 
	     LocalPlanners<CFG,WEIGHT>* lp,
	     Input* input) {

  if(input->seedByChunk.GetValue()) 
    GenerateIncrementalMap(rmap,Stats,cd,dm,nodes,lp,input, 
                 input->addPartialEdge.GetValue(), input->addAllEdges.GetValue() );
  else
    GenerateNormalMap(rmap,Stats,cd,dm,nodes,lp,input,
                 input->addPartialEdge.GetValue(), input->addAllEdges.GetValue() );

} 


/*Incremental Map Generator
 1. if inmapFile is active,
      read in the existing map
 2. set up nextNodeIndex for node generation methods
 3. generate/connect new nodes chunk by chunk
 4. update command line before we output the final map
 */
template <class CFG, class WEIGHT>
void 
MapGenerator<CFG, WEIGHT>::
GenerateIncrementalMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
            CollisionDetection* cd, 
            DistanceMetric* dm, vector<CFG>& nodes, 
	    LocalPlanners<CFG,WEIGHT>* lp,
	    Input* input,
	    bool addPartialEdge,
	    bool addAllEdges){

  double totalGenTime = 0.0;  //all gen time for this map
  double totalConTime = 0.0;  //all con time for this map
  
  nodes.erase(nodes.begin(),nodes.end());

  /*
  If inmapFile.IsActivated {
    rmap->ReadRoadmapGRAPHONLY(input->inmapFile.GetValue()) to:
    SetupFromMap()
  }
  Generate/connect nodes according to command line
  */
   
  if ( input->inmapFile.IsActivated() ){
     
    //---------------------------
    // Read roadmap nodes
    //---------------------------
    rmap->ReadRoadmapGRAPHONLY(input->inmapFile.GetValue());

    SetupFromMap(input, rmap);
  } 
    
  typename vector<NodeGenerationMethod<CFG>*>::iterator itr;
  for ( itr = gn.selected.begin(); itr != gn.selected.end(); itr++ ) {

    #ifndef QUIET
      cout<<endl<< (*itr)->GetName()<<endl<< flush;
    #endif

    double genTime = 0.0;    //this selected method
    double conTime = 0.0;    //this selected method
    
    int oriNumNodes = (*itr)->numNodes.GetValue();
    int chunkSize =   (*itr)->chunkSize.GetValue();
    int numChunks = oriNumNodes/chunkSize;
    if( oriNumNodes > chunkSize * numChunks)
      numChunks ++;
    vector<CFG> sub_nodes;
    cout<<"the chunk size is: "<<chunkSize<<", and the number of chunks is: "<<numChunks<<endl;

    //generate nodes chunk by chunk and seed for each chunk
    for (int i=0; i<numChunks; i++) {
      Clock_Class genClock;
      Clock_Class conClock;
      char genClockName[100], conClockName[100];      
      sprintf(genClockName, "%s%d%s", "Node generation time in chunk ", i, ": ");
      sprintf(conClockName, "%s%d%s", "Node connection time in chunk ", i, ": ");
      
      (*itr)->numNodes.PutValue (chunkSize);
      sub_nodes.clear();
      int nextNodeIndex = (*itr)->GetNextNodeIndex();
      cout<<endl<<"Chunk "<<i<<endl;
      OBPRM_srand((*itr)->GetName(), (*itr)->GetNextNodeIndex());
      nextNodeIndex += chunkSize;

      genClock.StartClock(genClockName);
      (*itr)->GenerateNodes(rmap->GetEnvironment(), Stats, cd, dm, sub_nodes);  //this is the original GenerateNodes function
      genClock.StopClock();

      //cout<<endl<<"  generate "<<sub_nodes.size()<<endl;
      if (gn.addNodes2Map) {
        rmap->m_pRoadmap->AddVertex(sub_nodes); //add sub_nodes to graph
      }

      //also keep these nodes in nodes
      nodes.insert(nodes.end(), sub_nodes.begin(), sub_nodes.end());

      //do the connection here
      //we should have optimization here
      conClock.StartClock(conClockName);
      cm.ConnectComponents(rmap, Stats, cd, dm, lp,
	       addPartialEdge, addAllEdges);
      conClock.StopClock();

      //set the next node index for this generation method
      (*itr)->SetNextNodeIndex(nextNodeIndex);

      //keep track of gen/con time
      genTime += genClock.GetClock_SEC();
      conTime += conClock.GetClock_SEC();
      #ifndef QUIET
        cout<<"\n "; genClock.PrintName(); cout << " " << flush;
        cout << genClock.GetClock_SEC() << " sec  " << flush;
	cout<<"\n "; conClock.PrintName(); cout << " " << flush;
        cout << conClock.GetClock_SEC() << " sec  \n" << flush;
      #endif

    }
    
    #ifndef QUIET
      cout<<endl<< (*itr)->GetName()<<endl<< flush;
      cout <<" Subtotal node generation time: "<<genTime << " sec  \n" << flush;
      cout <<" Subtotal node connection time: "<<conTime << " sec  \n" << flush;
    #endif

    (*itr)->numNodes.PutValue(oriNumNodes);

    totalGenTime += genTime;
    totalConTime += conTime;

  } //end of for itr = gn.selected

  #ifndef QUIET
    cout <<endl<<"Total node generation time: "<<totalGenTime << " sec  \n" << flush;
    cout <<"Total node connection time: "<<totalConTime << " sec  \n" << flush;
  #endif


} 


//This is the regular method we use: 
// 1. read/generate all nodes, 
// 2. connect them to form a roadmap
template <class CFG, class WEIGHT>
void 
MapGenerator<CFG, WEIGHT>::
GenerateNormalMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
            CollisionDetection* cd, 
            DistanceMetric* dm, vector<CFG>& nodes, 
	    LocalPlanners<CFG,WEIGHT>* lp,
	    Input* input,
	    bool addPartialEdge,
	    bool addAllEdges){

  Clock_Class        NodeGenClock;
  Clock_Class        ConnectionClock;
  nodes.erase(nodes.begin(),nodes.end());

   
  if ( input->inmapFile.IsActivated() ){
    //---------------------------
    // Read roadmap nodes
    //---------------------------
    rmap->ReadRoadmapGRAPHONLY(input->inmapFile.GetValue());
  } else {
    //---------------------------
    // Generate roadmap nodes
    //---------------------------
    NodeGenClock.StartClock("Node Generation");
    vector<CFG> nodes;
    gn.GenerateNodes(rmap,Stats,cd,dm,nodes);
    NodeGenClock.StopClock();
  }


  #ifdef QUIET
  #else
    cout << "\n";
    if ( input->inmapFile.IsActivated() ){
      cout << "Read nodes from the existing map:";
    } else{
      cout << "Node Generation: " << NodeGenClock.GetClock_SEC()
           << " sec (ie, " << NodeGenClock.GetClock_USEC() << " usec),";
    }
    
    cout << " "<<rmap->m_pRoadmap->GetVertexCount()<<" nodes\n"<< flush;
  #endif


  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  cm.ConnectComponents(rmap, Stats, cd, dm, lp,
	       addPartialEdge, addAllEdges);
  ConnectionClock.StopClock();

} 


#endif // !defined(_MAP_GENERATOR_H_)

#ifndef ExpanderStats
#define ExpanderStats

//#include <sys/time.h>
//#include <math.h>

#include "MPStrategyMethod.h"
#include "util.h"
#include "Roadmap.h"
#include "MPProblem.h"


class EdgeExpanderStats : public MPStrategyMethod {
  private:
    vector<string> files;
    string outfile;
    int interval;
    int queries;
    double resolution;
    double epsilon;
    double mu;
    typedef pair<int,vector<float> > expanderStats;

  public:
    int mpl;
    typedef RoadmapGraph<CfgType, WeightType>::VID VID;
    
    EdgeExpanderStats(XMLNodeReader& in_Node, MPProblem* problem) : MPStrategyMethod(in_Node, problem){
      ParseXML(in_Node);
    }

    void ParseXML(XMLNodeReader& in_Node) {
      cout << "edgeExpanderStats::ParseXML()" << endl;
      LOG_DEBUG_MSG("edgeExpanderStats::ParseXML()");
      interval = 100;
      queries=2;
      outfile="outfile.stats";
      outfile = in_Node.stringXMLParameter(string("filename"), true, string(""),string("Filename"));
     
      interval = in_Node.numberXMLParameter(string("interval"), true, 100, 1, 1000000, string("Printout Interval"));
      cout<<"interval="<<interval<<endl;
      mu=.2;

      mu = in_Node.numberXMLParameter("mu", false, double(0.0),double(0.0), double(0.5),"Mu paremeter for expander metrics");
      cout<<"Mu ="<<mu<<endl;
      epsilon = .9;
      epsilon = in_Node.numberXMLParameter("epsilon", false, double(0.0),double(0.0), double(5),"Epsilon paremeter for expander metrics");
      cout<<"Epsilon ="<<epsilon<<endl;
      resolution = .1; 
      resolution = in_Node.numberXMLParameter("resolution", false, double(0.0),double(0.0), double(5),"Resolution");
      cout<<"resolution ="<<resolution<<endl;
      mpl=10;
      mpl = in_Node.numberXMLParameter("max_path_length", false, int(0),int(0), int(1000),"max path length");
      cout<<"max_path_length ="<<mpl<<endl;
      XMLNodeReader::childiterator citr;
      for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr){
        if(citr->getName() == "file"){
          string filename = citr->stringXMLParameter(string("filename"), true, string(""),string("Node Compare Method"));
          cout<<"filename="<<filename<<endl;
          files.push_back(filename);
        } else{
          citr->warnUnknownNode();
        }
      }
    }

    long scaleFree(Roadmap<CfgType,WeightType> &rmp, vector<VID> &vertices){
      int scaleFreeMetric=0;
      for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
        vector<VID> adj_verts;
        
        rmp.m_pRoadmap->get_successors(*iter, adj_verts);
        for(vector<VID>::iterator iter2 = adj_verts.begin(); iter2!= adj_verts.end(); iter2++) {    
	   vector<VID> adj_verts2;
           rmp.m_pRoadmap->get_successors(*iter, adj_verts2);
           scaleFreeMetric+=adj_verts.size()*adj_verts2.size();
	}
	
      }
      return scaleFreeMetric;
    }


    //t,m,M,N = paremeters from randomized algorithm
    //rmp=roadmap
    //vertices = vertices to be included in calculation
    bool ExpanderTester(double t, double m, double M, double N, Roadmap<CfgType,WeightType> &rmp, vector<VID> &vertices){
      //int X;
      //cout<<"in funct"<<endl;
      for(int i=0; i<N; i++){                              //choose N verts
	//cout<<"in outter for i="<<i<<endl;
	int ranIndex = rand()%vertices.size();
	VID v = vertices[ranIndex];
	int X=0;
	vector<VID> visited;
	//VID walk_v = v;
	for(int j=0; j<m; j++){                            //preform m random walks
	  //cout<<"in midd for j="<<j<<endl;
	  VID walk_v = v;
	  for(int k=0; k<t; k++){                          //random walk
	    //cout<<"in inner for k="<<k<<endl;       
	    vector<VID> succ;
	    rmp.m_pRoadmap->get_successors(walk_v, succ);
	    //cout<<"getting rand succ "<<succ.size()<<endl;
	    if(succ.size()>0){
              int ranSuccIndex = rand()%succ.size();
	      //cout<<"setting walk"<<endl;
	      walk_v=succ[ranSuccIndex];
	      //cout<<"walk_v="<<walk_v<<endl;
	    }
	  }  
	  //cout<<"after loop"<<endl;
	
	  //cout<<"before if"<<endl;
	  if(find(visited.begin(), visited.end(), walk_v)==visited.end()){
	    visited.push_back(walk_v);
	  }else{
	    X++;
	  }
	}
	cout<<"x ="<<X<<" m= "<<M<<endl;
	if(X>M)
	  return false;
      }
      return true;
    }

    double vertex_expansion(Roadmap<CfgType,WeightType> &rmp, vector<VID> &vertices, int d){
      double alpha_top=1;
      double alpha_bottom=0;
      /*
      int d=0;
      for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
        vector<VID> succ;
        rmp.m_pRoadmap->get_successors(*iter, succ);
	d=max(d,succ.size());
      }
      */
      cout<<"d="<<d<<endl;
      while(alpha_top-alpha_bottom>resolution){
	cout<<"in while"<<endl;
	double alpha=(alpha_top+alpha_bottom)/2;
	cout<<"alpha"<<alpha<<endl;
	double t = 16*pow((double)d,(int)2)*log(vertices.size())/(double)pow((double)alpha,(int)2);
	cout<<"t="<<t<<endl;
	double m = pow(vertices.size(),.5+mu);
	cout<<"m="<<m<<endl;
        double M = pow(vertices.size(),2*mu)/2+pow(vertices.size(),7*mu/4)/128;
	cout<<"M="<<M<<endl;
	double N =300/epsilon;
	cout<<"N="<<N<<endl;
	cout<<"before"<<endl;
	if(ExpanderTester(t, m, M, N, rmp, vertices)){
	  alpha_bottom=alpha;
	}else{
	  alpha_top=alpha;
	}
	cout<<"alpha="<<alpha_top<<" "<<alpha_bottom<<endl;
      }
      cout<<(alpha_top+alpha_bottom)/2;
      //vector<VID> vertices;
      //rmp.m_pRoadmap->GetVerticesVID(vertices);
      //for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
	//compute stuff here
      //}
      return (alpha_top+alpha_bottom)/2;
    }

    double edge_expansion(Roadmap<CfgType,WeightType> rmp, int d){

      return 0;
    }


    vector<int> hop_graph_vertex(Roadmap<CfgType,WeightType> &rmp, VID v){
      vector<int> hop_graph;
      hop_graph.push_back(0);
      //hop_graph.resize(mpl,0);
      vector<VID> visited;
      visited.push_back(v);
      for(int i=1; i<mpl; i++){
	vector<VID> tmp = visited;
	for(vector<VID>::iterator iter=tmp.begin(); iter<tmp.end(); iter++){ 
	  vector<VID> succ; 
	  rmp.m_pRoadmap->get_successors(*iter, succ); 
	  for(vector<VID>::iterator iter2=succ.begin(); iter2<succ.end(); iter2++){
	    if(find(visited.begin(),visited.end(),*iter2)==visited.end()){
	      visited.push_back(*iter2);
	    }
	  }	  
	}
	hop_graph.push_back(visited.size()-1);  //not counting self
      }
      return hop_graph;
    }

    vector<int> hop_graph(Roadmap<CfgType,WeightType> &rmp,vector<VID> &vertices){
      vector<int> sum;
      sum.resize(mpl,0);
      for(vector<VID>::iterator iter=vertices.begin(); iter<vertices.end(); iter++){
	vector<int> hop_graph_v=hop_graph_vertex(rmp,*iter);
	for(int i=0;i<mpl;i++){
	  sum[i] += hop_graph_v[i];
	}
      }
      for(int i=0;i<mpl;i++){
	sum[i]/=2;
      }
      return sum;
    }


    vector<int> triangleParticipation(Roadmap<CfgType,WeightType> &rmp,vector<VID> &vertices, int d){
      vector<int> participation;
      participation.resize(d*(d-1)/2,0);
      for(vector<VID>::iterator iter=vertices.begin(); iter<vertices.end(); iter++){
	vector<VID> succ;
	succ.reserve(d); 
	rmp.m_pRoadmap->get_successors(*iter, succ);
	int sum=0;
	for(vector<VID>::iterator iter2=succ.begin(); iter2<succ.end(); iter2++){
	  vector<VID> succ_neighbor;
	  succ_neighbor.reserve(d);
	  rmp.m_pRoadmap->get_successors(*iter2, succ_neighbor);
	  for(vector<VID>::iterator iter3=succ_neighbor.begin(); iter3<succ_neighbor.end(); iter3++){
	    if(find(succ.begin(),succ.end(),*iter3)!=succ.end() && *iter2<*iter3) {sum++;}
	  }
	}    
	participation[sum]++;
      }
      int return_size=0;
      for(size_t i=0; i<participation.size(); i++){
        if(participation[i]>0){
	  return_size=i+1;            
        }
      }
      participation.resize(return_size);
      return participation;
    }

    vector<int> hop_graph(Roadmap<CfgType,WeightType> &rmp){
     // vector<VID> vertices(rmp.m_pRoadmap->begin(),rmp.m_pRoadmap->end());
      vector<VID> vertices;
     // vector<VID> vertices(rmp.m_pRoadmap.size());
      return hop_graph(rmp, vertices);
    }


    vector<int> triangleParticipation(Roadmap<CfgType,WeightType> &rmp){
     // vector<VID> vertices(rmp.m_pRoadmap->begin(),rmp.m_pRoadmap->end());
      vector<VID> vertices;
      int d=0;
      for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
        vector<VID> succ;
        rmp.m_pRoadmap->get_successors(*iter, succ);
        d=max(d,(int)succ.size());
      }
      return triangleParticipation(rmp,vertices,d);
    }



    
    void print_expansion_properties(Roadmap<CfgType,WeightType> &rmp, int _interval,ofstream *myofstream){
     // vector<VID> vertices(rmp.m_pRoadmap->begin(),rmp.m_pRoadmap->end());
      vector<VID> vertices;
      int d=0;
      for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
        vector<VID> succ;
        rmp.m_pRoadmap->get_successors(*iter, succ);
	d=max(d,(int)succ.size());
      }
      //rmp.m_pRoadmap->GetVerticesVID(vertices);
      //vector<expanderStats> stats;
      vector<int> intervals;
      vector<long> scaleFreeStats;
      vector<double> vertexExpander;
      vector<vector<int> > hop_graph_stats;
      vector<vector<int> > triangleParticipation_stats;
      while((int)vertices.size()!=queries && (int)vertices.size()>0){
        if(((int)vertices.size()-queries) % _interval == 0){
	  intervals.push_back((int)vertices.size()-queries);
	  long sf= scaleFree(rmp,vertices);
	  scaleFreeStats.push_back(sf);
	  
	  //vertex expansion
	  vector< pair<size_t, VID> > ccstats;
	  stapl::vector_property_map< GRAPH,size_t > cmap;
	  vector <vector <VID> > ccs;
	  get_cc_stats(*(rmp.m_pRoadmap), cmap, ccstats, ccs);
	  vector<VID> cc1;
          //get_cc(*(rmp.m_pRoadmap), cmap, ccstats.front().second, cc1);
	  //vertexExpander.push_back(vertex_expansion(rmp, ccs[0]),d);  //change to do for largest
	  hop_graph_stats.push_back(hop_graph(rmp,vertices));
          triangleParticipation_stats.push_back(triangleParticipation(rmp,vertices,d));
	}
	rmp.m_pRoadmap->delete_vertex(vertices.back());
	vertices.pop_back();
      }
      *myofstream<<"n scale free ";
      for(int i=1; i<mpl; i++){
	*myofstream<<"hop_graph_"<<i<<" ";
      }
      //cout<<"triangle_participation "<<endl;
      vector<int> intervalsTriangle=intervals;
      while(!intervals.empty()){
        *myofstream<<intervals.back()<<" "<<scaleFreeStats.back()<<" "; 
	for(int i=1; i<mpl; i++){
	  *myofstream<<hop_graph_stats.back()[i]<<" ";
	}
	*myofstream<<endl;
	intervals.pop_back();
	scaleFreeStats.pop_back();
	hop_graph_stats.pop_back();
      }
      *myofstream<<"interval triangle_participation "<<endl;
      while(!intervalsTriangle.empty()){
        *myofstream<<intervalsTriangle.back();
	for(size_t i=1; i<triangleParticipation_stats.back().size(); i++){
	  *myofstream<<" "<<triangleParticipation_stats.back()[i];
	}
	*myofstream<<endl;
	intervalsTriangle.pop_back();
	triangleParticipation_stats.pop_back();
      }
    }
    
  virtual void PrintOptions(ostream& out_os) { }

   virtual void Initialize(int in_RegionID){}
   virtual void Run(int in_RegionID){    
      cout<<"*************in operator()***********************"<<endl;
      if(files.size()==0){
         cout<<"no files"<<endl;
      }
      ofstream myofstream(outfile.c_str());

      for(vector<string>::iterator iter = files.begin(); iter!=files.end(); iter++){
         cout<<"loading "<<iter->c_str()<<endl;
         Roadmap<CfgType,WeightType> rmp;
         rmp.ReadRoadmapGRAPHONLY(iter->c_str());
         rmp.SetEnvironment(GetMPProblem()->GetEnvironment());
         cout<<"computing stats for "<<iter->c_str()<<endl;
         myofstream<<iter->c_str()<<endl;
         print_expansion_properties(rmp,interval,&myofstream);

      }
      cout<<"returning"<<endl;
      myofstream.close();

   }
   virtual void Finalize(int in_RegionID){}

};

 


#endif

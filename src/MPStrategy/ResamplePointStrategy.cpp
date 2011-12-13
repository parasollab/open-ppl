#include "ResamplePointStrategy.h"
#include "MPRegion.h"
#include "SamplerMethod.h"
#include "MPStrategy/MPStrategy.h"
#include <algorithm>

class DistanceMetricMethod;

ResamplePointStrategy::
ResamplePointStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
  : MPStrategyMethod(in_Node, in_pProblem) 
{
  ParseXML(in_Node); 
}

ResamplePointStrategy::
~ResamplePointStrategy()
{}

void
ResamplePointStrategy::
PrintOptions(ostream& out_os)
{
   out_os << "ResamplePointStrategy::\n";
   out_os << "\tinput_path_filename = \"" << m_input_path_filename << "\"\n";
   out_os << "\toutput_path_filename = \"" << m_output_path_filename << "\"\n";
   out_os << "\tinput_map_filename = \"" << m_input_map_filename << "\"\n";
   out_os << "\toutput_map_filename = \"" << m_output_map_filename << "\"\n";
   out_os << "\tnum_resamples = " << m_num_resamples << "\n";
}
struct sort_ascend {
   bool operator()(const std::pair<CfgType,double> &left, const std::pair<CfgType,double> &right) {
      return left.second < right.second;
   }
};

struct sort_descend {
   bool operator()(const std::pair<CfgType,double> &left, const std::pair<CfgType,double> &right) {
      return left.second > right.second;
   }
};
void
ResamplePointStrategy::
ParseXML(XMLNodeReader& in_Node) 
{
   m_input_path_filename = in_Node.stringXMLParameter("input_path", true, "", "filename containing path to be resampled");
   m_output_path_filename = in_Node.stringXMLParameter("output_path", false, "", "filename to output resampled path to, default = input_path");
   if(m_output_path_filename == "")
      m_output_path_filename = m_input_path_filename;
   m_input_map_filename = in_Node.stringXMLParameter("input_map", true, "", "filename containing map to be resampled");
   m_output_map_filename = in_Node.stringXMLParameter("output_map", false, "", "filename to output resampled map to, default = input_map");
   if(m_output_map_filename == "")
      m_output_map_filename = m_input_map_filename;
   m_num_resamples = in_Node.numberXMLParameter("resample", false, 5, 1, 5000, "number of samples to generate in each resample attempt, default = 5");
   step_size= in_Node.numberXMLParameter("step_size", false, 0.0, 0.0,2.0, "distance of a resampled node from existing one, default = 0");
   user_value= in_Node.numberXMLParameter("user_value", false, 0.3, 0.0, 2000000.0, "distance of a resampled node from existing one, default = .01,   range .01-2000000");
   type_name = in_Node.stringXMLParameter("type_name", true, "", "type of the CFG task");
   m_vc = in_Node.stringXMLParameter(string("vc_method"), true, string(""), string("CD Library"));
   m_dm = in_Node.stringXMLParameter("dm_method", true, "", "Distance metric");
   m_lp = in_Node.stringXMLParameter("lp_method", true, "", "Local Planner");
   in_Node.warnUnrequestedAttributes();
}


double GetValue(CfgType C, MPProblem* mp, Environment* env, Stat_Class& Stats,
     string m_vc, CDInfo& cdInfo, string m_dm, int x,int bl){
 
    return C.GetSmoothingValue(mp, env, Stats,
     m_vc, cdInfo, m_dm, x,bl );
 
   
}


void findNeighbour(string type_name, Roadmap<CfgType,WeightType>* rdmp, MPProblem* mp,
                   Environment *env, LocalPlanners<CfgType,WeightType>* lp, 
                   shared_ptr <DistanceMetricMethod> dm, Stat_Class& Stats, 
                   CollisionDetection* cd, CDInfo& cdInfo, string m_vc, string m_dm, string m_lp,
                   int x, int bl, CfgType previous, CfgType c, CfgType next, double step_size,
                   double user_value, int max_attempt, int numOfSamples, vector< pair<CfgType,double> > &result) {
     
   typedef RoadmapGraph<CfgType, WeightType>::VID VID;   
   double newConfigurationWeight;
   double oldConfigurationWeight;
   bool firstConnectFlag;
   bool secondConnectFlag;
   VID vid1;
   VID vid2;
   string strVcmethod;
   string callee;
   LPOutput<CfgType,WeightType> lpOutput;
   oldConfigurationWeight= GetValue( c, mp, env, Stats, m_vc, cdInfo, m_dm, x,bl);
   for(int k=0;k<max_attempt && numOfSamples>0;k++) {  
      CfgType r,c2 ;
      r.GetRandomRay(step_size, env, dm );
      c2.add(r,c); 
      newConfigurationWeight=GetValue(c2, mp, env, Stats, m_vc, cdInfo, m_dm, x,bl);
      if((newConfigurationWeight>oldConfigurationWeight && type_name.compare("MAX_CLEARANCE")==0) ||
        (newConfigurationWeight<oldConfigurationWeight && type_name.compare("PROTEIN_ENERGY")==0)) {
        firstConnectFlag = lp->GetLocalPlannerMethod(m_lp)->
                               IsConnected(env, Stats, dm, previous, c2, &lpOutput, 
                               rdmp->GetEnvironment()->GetPositionRes(),
                               rdmp->GetEnvironment()->GetOrientationRes(),true);
				secondConnectFlag = lp->GetLocalPlannerMethod(m_lp)->
                                IsConnected(env, Stats, dm, c2,next, &lpOutput, 
                                rdmp->GetEnvironment()->GetPositionRes(),
                                rdmp->GetEnvironment()->GetOrientationRes(), true);

	 vid1=rdmp->m_pRoadmap->GetVID(previous);
         vid2=rdmp->m_pRoadmap->GetVID(c2);
	 if(vid2 == INVALID_VID) {
            rdmp->m_pRoadmap->AddVertex(c2);// the vertex did not exist 
            vid1=rdmp->m_pRoadmap->GetVID(next);
         } 
	 else              	
	    vid1=rdmp->m_pRoadmap->GetVID(next);
	 if(firstConnectFlag && secondConnectFlag) {
   	    rdmp->m_pRoadmap->AddEdge(previous,c2,lpOutput.edge); 
            rdmp->m_pRoadmap->AddEdge(c2,next,lpOutput.edge);  
            result.push_back( pair<CfgType,double>(c2,newConfigurationWeight));
	    numOfSamples--;
           
         }
             
         }
   
}//for

if(type_name.compare("MAX_CLEARANCE")==0 ) 
  std::sort(result.begin(), result.end(), sort_descend());
else
  std::sort(result.begin(), result.end(), sort_ascend());

}


void
ResamplePointStrategy::
Run(int in_RegionID) 
{
   PrintOptions(cout);
   double currentConfigurationWeight=0.0;
   double temp=0.0;
   int  max_attempts=1000;
   int len; 
   char buff[5000];
   len=type_name.copy(buff,100);
   buff[len]='\0';
   char names[3][5000]={"PROTEIN_ENERGY","MAX_CLEARANCE",""};
   cout<<"type_name,resample,step_size,user_value="<<buff<<","<<m_num_resamples<<","<<step_size<<","<<user_value<<endl;
   
     
   //load input map
   GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap()->ReadRoadmapGRAPHONLY(m_input_map_filename.c_str());
   Roadmap<CfgType,WeightType>* rdmp = GetMPProblem()->GetMPRegion(in_RegionID)->GetRoadmap();

   //load input path
   vector<CfgType> path_cfgs;
   vector<CfgType> opath_cfgs;
   ifstream in_path(m_input_path_filename.c_str());
   cout<<m_input_path_filename.c_str();
   int num_cfgs = 0;
   if(strcmp(buff,names[1])==0 ){//max clearance
     char line[256];
     in_path.getline(line, 256); //skip header line
     in_path.getline(line, 256); //skip header line  

   }
   in_path >> num_cfgs;
   cout<<"num cfgs:"<<num_cfgs<<endl;
   for(int i=0; i<num_cfgs; ++i) {
      CfgType c;
      c.Read(in_path);
      path_cfgs.push_back(c);
    
   }
   in_path.close();
   vector<VID> path_vids;
   double smoothingValues[10000];
   int index =0;
   Stat_Class* pStatClass = GetMPProblem()->GetMPRegion(in_RegionID)->GetStatClass();
   CDInfo cdInfo;
   LocalPlanners<CfgType,WeightType>* lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
   vector<CfgType>::iterator C = path_cfgs.begin();
   Clock_Class ResamplingClock;
   ResamplingClock.StartClock("Resampling started");
   opath_cfgs.push_back(*C);
   CfgType previous;
   temp=GetValue( *C, GetMPProblem(), rdmp->GetEnvironment(), *pStatClass,
    m_vc, cdInfo, m_dm, 5000,false);
   cout<<"oldsmoothingvalues:"<<endl<<temp<<";";
   smoothingValues[index]=temp;
   index++;
   ++C;
   CfgType next;
   for( ;C+1!=path_cfgs.end();++C)  {
    

      currentConfigurationWeight = GetValue( *C, GetMPProblem(), rdmp->GetEnvironment(), *pStatClass,
      m_vc, cdInfo, m_dm, 5000,false);
      cout<<currentConfigurationWeight<<";";
      if((currentConfigurationWeight > user_value && type_name.compare("PROTEIN_ENERGY")==0) ||
        (currentConfigurationWeight < user_value && type_name.compare("MAX_CLEARANCE")==0)) {
        
         vector< pair <CfgType,double> > sampledNeighbors;
         
         --C;
         CfgType previous =*C;
         ++C;
         vector<CfgType>::iterator nxt = C;
         nxt++;
         CfgType current=*C;
         CfgType next=*nxt;
         findNeighbour(type_name, rdmp,GetMPProblem(),rdmp->GetEnvironment(), lp,
                       GetMPProblem()->GetDistanceMetric()->GetDMMethod(m_dm),*pStatClass,
                       GetMPProblem()->GetCollisionDetection(), cdInfo, m_vc, m_dm, m_lp,
                       5000, false, previous, current, next, step_size, user_value,
                       max_attempts, m_num_resamples, sampledNeighbors); 
        
         if(sampledNeighbors.size()>0) {
	    vector<pair<CfgType,double> >::iterator N = sampledNeighbors.begin();
            opath_cfgs.push_back(N->first);
            temp= GetValue( N->first, GetMPProblem(), rdmp->GetEnvironment(), *pStatClass,
      m_vc, cdInfo, m_dm, 5000,false);
            smoothingValues[index]=temp;
            index++;
            *C=N->first;
         }
         else{
           temp= GetValue( *C, GetMPProblem(), rdmp->GetEnvironment(), *pStatClass,
      m_vc, cdInfo, m_dm, 5000,false);
           smoothingValues[index]=temp;
           index++; 
           opath_cfgs.push_back(*C);
        }
   }
      else {
        temp= GetValue( *C, GetMPProblem(), rdmp->GetEnvironment(), *pStatClass,
      m_vc, cdInfo, m_dm, 5000,false);
        smoothingValues[index]=temp;
        index++;
        opath_cfgs.push_back(*C);
           
          }
     
   }//for
   
   temp=GetValue( *C, GetMPProblem(), rdmp->GetEnvironment(), *pStatClass,
   m_vc, cdInfo, m_dm, 5000,false);
   cout<<temp<<endl;
   smoothingValues[index]=temp;
   index++;
   opath_cfgs.push_back(*C);
       
    ResamplingClock.StopPrintClock(); 
   
   cout << endl;
   cout<<"newsmoothingvalues:"<<endl;
   for(int i=0; i<index; ++i)  {
     cout<<smoothingValues[i]<<";";
   }
   cout<<endl;

  

   len=m_output_path_filename.copy(buff,100);
   buff[len]='\0';
   WritePathConfigurations(buff, opath_cfgs, rdmp->GetEnvironment()); 
  //write output map
  ofstream os_map(m_output_map_filename.c_str());
  GetMPProblem()->GetMPRegion(in_RegionID)->WriteRoadmapForVizmo(os_map);
  os_map.close();
}



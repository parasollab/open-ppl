#ifndef MedialAxisSamplers_h
#define MedialAxisSamplers_h

#include "SamplerMethod.h"
class Environment;
class Stat_Class;
class CollisionDetection;
class CDInfo;
class DistanceMetric;

#include <sstream>

template <typename CFG>
class FreeMedialAxisSampler : public SamplerMethod<CFG>
{
   public:
      Environment* env;
      Stat_Class *Stats;
      CollisionDetection* cd;
      ValidityChecker<CFG>* vc;
      CDInfo *cdInfo;
      shared_ptr<DistanceMetricMethod> dm;
      string dmstring;
      MPProblem* mp;
      int clearance, penetration;
      std::string strLabel;
      std::string strVcmethod;
      bool m_debug;

      FreeMedialAxisSampler() : m_debug(false) {}
      FreeMedialAxisSampler(Environment* _env, Stat_Class* _Stats, 
            CollisionDetection* _cd, CDInfo* _cdInfo, 
            shared_ptr<DistanceMetricMethod> _dm, int _c, int _p) :
         env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm),
         clearance(_c), penetration(_p), m_debug(false) {} 
      FreeMedialAxisSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) {
         LOG_DEBUG_MSG("FreeMedialAxisSampler::FreeMedialAxisSampler()");
         ParseXML(in_Node);
         mp = in_pProblem;
         vc = in_pProblem->GetValidityChecker();
         cd = in_pProblem->GetCollisionDetection();
         dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dmstring);
         LOG_DEBUG_MSG("~FreeMedialAxisSampler::FreeMedialAxisSampler()");
      }

      ~FreeMedialAxisSampler() {}

      const char* name() const { return "FreeMedialAxisSampler"; }

      void  ParseXML(XMLNodeReader& in_Node) {
         LOG_DEBUG_MSG("FreeMedialAxisSampler::ParseXML()");
         cout << "FreeMedialAxisSampler ";
 
         strVcmethod = in_Node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
         cout << "strVcmethod = " << strVcmethod << endl;
         
         dmstring = in_Node.stringXMLParameter("dm_method", true, "", "Distance metric");
         
         strLabel = this->ParseLabelXML(in_Node);
         this->SetLabel(strLabel);
         
         clearance = in_Node.numberXMLParameter("clearance", false, 10, 0, 1000, "Clearance Number");
         penetration = in_Node.numberXMLParameter("penetration", false, 5, 0, 1000, "Penetration Number");
         
         m_debug = in_Node.boolXMLParameter("debug", false, false, "debugging flag");
        
         print(cout);

         LOG_DEBUG_MSG("~FreeMedialAxisSampler::ParseXML()");
      }

      void print(ostream& os) const
      {
         os << name() 
            << " (clearance = " << clearance 
            << ", penetration = " << penetration << ")";
      }

      bool sampler(Environment* env,Stat_Class& Stat, const CFG& cfg_in, vector<CFG>& cfg_out, int max_attempts)  
      {
         
            string callee(name());
            callee += "::sampler()";
            bool generated = false;
            int attempts = 0;
            CDInfo cdInfo;
            cdInfo.ret_all_info = true;
            if(m_debug)
            {
              cout << "\n\n***********************************************\n" 
                 << "In MedialAxisSamplers::sampler !! NEW SAMPLE !!\n"
                 << "***********************************************\n"; 
              print(cout);
              cout << flush;
            }
            do {
               Stat.IncNodes_Attempted();
               attempts++;

               CFG tmp = cfg_in;
               CFG tmp2 = tmp;
               if(m_debug)
                 cout << "Attempt: " << attempts << "... About to Call PushToMedialAxis" << endl;

             bool check = tmp.GetMedialAxisCfg(mp, env, Stat, strVcmethod, cd,  cdInfo, dmstring, clearance, penetration);


               if(vc->IsValid(vc->GetVCMethod(strVcmethod), tmp, env, Stat, cdInfo, true, &callee) && check){
                  Stat.IncNodes_Generated();
                  generated = true;
                  cfg_out.push_back(tmp);
               }
               if(m_debug)
                 cout << flush;
            } while (!generated && (attempts < max_attempts));

            return generated;
      
      }

   private:
      template <typename OutputIterator>
         OutputIterator 
         _Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
               OutputIterator result)  
         {
            CFG my_cfg;
            vector<CFG> out1;
            for (int i =0; i< num_nodes; i++) {
               my_cfg.GetRandomCfg(env);
               while(!sampler(env, Stat,my_cfg, out1, max_attempts))
                  my_cfg.GetRandomCfg(env);
            }
            result = copy(out1.begin(), out1.end(), result);
            return result;
         }

      template <typename InputIterator, typename OutputIterator>
         OutputIterator 
         _Sample(Environment* env, Stat_Class& Stat, InputIterator first, InputIterator last, int max_attempts,
               OutputIterator result)  
         {
            while(first != last) {
               vector<CFG> result_cfg;
               if(sampler(env, Stat, *first, result_cfg, max_attempts)) 
                  result = copy(result_cfg.begin(), result_cfg.end(), result);
               first++;
            }
            return result;
         }   

   public:
      //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
      virtual back_insert_iterator<vector<CFG> > 
         Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
               back_insert_iterator<vector<CFG> > result)  
         {
            return _Sample(env, Stat, num_nodes, max_attempts, result);
         }

      virtual back_insert_iterator<vector<CFG> > 
         Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
               back_insert_iterator<vector<CFG> > result)  
         {
            return _Sample(env, Stat, first, last, max_attempts, result);
         }   

      //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = vector<CFG>::iterator
      virtual typename vector<CFG>::iterator 
         Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
               typename vector<CFG>::iterator result)  
         {
            return _Sample(env, Stat, num_nodes, max_attempts, result);
         }

      virtual typename vector<CFG>::iterator 
         Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
               typename vector<CFG>::iterator result)  
         {
            return _Sample(env, Stat, first, last, max_attempts, result);
         }
};

#endif


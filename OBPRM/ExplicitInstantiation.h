#ifndef _ExplicitInstantiation_h_
#define _ExplicitInstationation_h_

// explicit template instation of stati data members
// the is required for the current hp linker
// must be included after CfgType and WeightType are defined

//from BasicOBPRM.h
template<> const int BasicOBPRM<CfgType>::MAX_CONVERGE = 20; 

//from LocalPlanners.h
template<> int LocalPlanners<CfgType,WeightType>::lp_counter = -1;

#if defined USE_CSTK
  template<> cd_predefined LocalPlanners<CfgType,WeightType>::cdtype = CSTK;
#elif defined USE_RAPID
  template<> cd_predefined LocalPlanners<CfgType,WeightType>::cdtype = RAPID;
#elif defined USE_PQP
  template<> cd_predefined LocalPlanners<CfgType,WeightType>::cdtype = PQP;
#elif defined USE_VCLIP
  template<> cd_predefined LocalPlanners<CfgType,WeightType>::cdtype = VCLIP;
#else
  #ifdef NO_CD_USE
    template<> cd_predefined LocalPlanners<CfgType,WeightType>::cdtype = CD_USER1;
  #else
    #error You have to specify at least one collision detection library.
  #endif
#endif 

//from ConnectMap.h
template<> double ConnectMap<CfgType,WeightType>::connectionPosRes = 0.05;
template<> double ConnectMap<CfgType,WeightType>::connectionOriRes = 0.05;


// for Input.cpp
#ifndef CfgTypeIsCfg_free

template<> const int BasicOBPRM<Cfg_free>::MAX_CONVERGE = 20;

#ifndef WeightTypeIsDefaultWeight
template<> int LocalPlanners<Cfg_free,DefaultWeight>::lp_counter = -1;

#if defined USE_CSTK
  template<> cd_predefined LocalPlanners<Cfg_free,DefaultWeight>::cdtype = CSTK;
#elif defined USE_RAPID
  template<> cd_predefined LocalPlanners<Cfg_free,DefaultWeight>::cdtype = RAPID;
#elif defined USE_PQP
  template<> cd_predefined LocalPlanners<Cfg_free,DefaultWeight>::cdtype = PQP;
#elif defined USE_VCLIP
  template<> cd_predefined LocalPlanners<Cfg_free,DefaultWeight>::cdtype = VCLIP;
#else
  #ifdef NO_CD_USE
    template<> cd_predefined LocalPlanners<Cfg_free,DefaultWeight>::cdtype = CD_USER1;
  #else
    #error You have to specify at least one collision detection library.
  #endif
#endif
 
#endif
#endif

#endif

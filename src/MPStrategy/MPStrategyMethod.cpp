#include "MPStrategyMethod.h"
#include <sys/time.h>
#include "MPProblem.h"
#include "MPRegion.h"

MPStrategyMethod::MPStrategyMethod(MPSMContainer& _cont) : m_baseSeed(_cont.m_seed), m_baseFilename(_cont.m_baseFilename) {
}

MPStrategyMethod::MPStrategyMethod(XMLNodeReader& _node, MPProblem* _problem) : MPBaseObject(_node, _problem) {
  if(m_boundary==NULL)
    m_boundary = GetMPProblem()->GetEnvironment()->GetBoundingBox();
  ParseXML(_node);
}

MPStrategyMethod::~MPStrategyMethod() {
}

void MPStrategyMethod::ParseXML(XMLNodeReader& _node){
  struct timeval tv;
  gettimeofday(&tv,NULL);
  m_baseSeed = _node.numberXMLParameter("seed", false, (int)tv.tv_usec, 0, MAX_INT, "Random Seed Value"); 
  m_baseFilename = _node.stringXMLParameter("filename", true, "", "Base output filename");
  SRand(m_baseSeed); 
};

void MPStrategyMethod::operator()(){
  (*this)(GetMPProblem()->CreateMPRegion());
}

void MPStrategyMethod::operator()(int _regionID){
  GetMPProblem()->GetMPRegion(_regionID)->GetStatClass()->SetAuxDest(GetBaseFilename());

  Initialize(_regionID);
  Run(_regionID);
  Finalize(_regionID);
}



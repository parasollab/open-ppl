#ifndef PUSHQUERYTOMA_H_
#define PUSHQUERYTOMA_H_

#include "MPStrategyMethod.h"
#include "Extenders/BasicExtender.h"

template<class MPTraits>
class PushQueryToMA : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    PushQueryToMA(const string& _inQueryFile = "", const string& _outQueryFile = "",
        const MedialAxisUtility<MPTraits>& _medialAxisUtility = MedialAxisUtility<MPTraits>());
    PushQueryToMA(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

  protected:
    string m_inQueryFile, m_outQueryFile;
    MedialAxisUtility<MPTraits> m_medialAxisUtility;

    vector<CfgType> m_query;
};

template<class MPTraits>
PushQueryToMA<MPTraits>::PushQueryToMA(const string& _inQueryFile, const string& _outQueryFile,
    const MedialAxisUtility<MPTraits>& _medialAxisUtility) :
  m_inQueryFile(_inQueryFile), m_outQueryFile(_outQueryFile), m_medialAxisUtility(_medialAxisUtility) {
    this->SetName("PushQueryToMA");
  }

template<class MPTraits>
PushQueryToMA<MPTraits>::PushQueryToMA(MPProblemType* _problem, XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node), m_medialAxisUtility(_problem, _node){
    this->SetName("PushQueryToMA");
    m_inQueryFile = _node.stringXMLParameter("inFilename", true, "", "Query Filename");
    m_outQueryFile = _node.stringXMLParameter("outFilename", true, "", "Query Filename");
    _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
void
PushQueryToMA<MPTraits>::Print(ostream& _os) const {
  _os << "In Query file: " << m_inQueryFile << endl;
  _os << "Out Query file: " << m_outQueryFile << endl;
  m_medialAxisUtility.Print(_os);
}

//////////////////////
//Initialization Phase
/////////////////////
template<class MPTraits>
void
PushQueryToMA<MPTraits>::Initialize(){
  if(this->m_debug) cout<<"\nInitializing PushQueryToMA::"<<endl;

  if(!FileExists(m_inQueryFile))
    throw ParseException(WHERE, "Query file '" + m_inQueryFile + "' does not exist.");

  ifstream ifs(m_inQueryFile.c_str());
  CfgType tmp;
  while(ifs >> tmp)
    m_query.push_back(tmp);

  if(this->m_debug) cout<<"\nEnding Initializing PushQueryToMA"<<endl;
}

////////////////
//Run/Start Phase
////////////////
template<class MPTraits>
void
PushQueryToMA<MPTraits>::Run() {
  if(this->m_debug) cout << "\nRunning PushQueryToMA::" << endl;

  typedef typename vector<CfgType>::iterator CIT;
  for(CIT cit = m_query.begin(); cit != m_query.end(); ++cit) {
    if(!m_medialAxisUtility.PushToMedialAxis(*cit, this->GetMPProblem()->GetEnvironment()->GetBoundary())) {
      cerr << "Cannot push to MA. Exiting." << endl;
      exit(1);
    }
  }
}

/////////////////////
//Finalization phase
////////////////////
template<class MPTraits>
void
PushQueryToMA<MPTraits>::Finalize() {
  if(this->m_debug) cout<<"\nFinalizing PushQueryToMA::"<<endl;

  ofstream ofs(m_outQueryFile.c_str());
  typedef typename vector<CfgType>::iterator CIT;
  for(CIT cit = m_query.begin(); cit != m_query.end(); ++cit)
    ofs << *cit << endl;

  if(this->m_debug) cout<<"\nEnd Finalizing PushQueryToMA"<<endl;
}

#endif

#ifndef PUSH_QUERY_TO_MA_H_
#define PUSH_QUERY_TO_MA_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// TODO
/// @ingroup MotionPlanningStrategies
/// @internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PushQueryToMA : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    PushQueryToMA(const string& _inQueryFile = "",
        const string& _outQueryFile = "",
        const MedialAxisUtility<MPTraits>& _medialAxisUtility =
        MedialAxisUtility<MPTraits>());
    PushQueryToMA(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

  protected:
    string m_inQueryFile, m_outQueryFile;
    MedialAxisUtility<MPTraits> m_medialAxisUtility;

    vector<CfgType> m_query;
};

template <typename MPTraits>
PushQueryToMA<MPTraits>::
PushQueryToMA(const string& _inQueryFile, const string& _outQueryFile,
    const MedialAxisUtility<MPTraits>& _medialAxisUtility) :
  m_inQueryFile(_inQueryFile), m_outQueryFile(_outQueryFile),
  m_medialAxisUtility(_medialAxisUtility) {
    this->SetName("PushQueryToMA");
  }

template <typename MPTraits>
PushQueryToMA<MPTraits>::
PushQueryToMA(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node),
  m_medialAxisUtility(_node) {
    this->SetName("PushQueryToMA");
    m_inQueryFile = _node.Read("inFilename", true, "", "Query Filename");
    m_outQueryFile = _node.Read("outFilename", true, "", "Query Filename");
  }

template <typename MPTraits>
void
PushQueryToMA<MPTraits>::
Print(ostream& _os) const {
  _os << "In Query file: " << m_inQueryFile << endl;
  _os << "Out Query file: " << m_outQueryFile << endl;
  m_medialAxisUtility.Print(_os);
}

template <typename MPTraits>
void
PushQueryToMA<MPTraits>::
Initialize() {
  /// @TODO Remove dependence on query file.
  if(!FileExists(m_inQueryFile))
    throw ParseException(WHERE,
        "Query file '" + m_inQueryFile + "' does not exist.");

  ifstream ifs(m_inQueryFile.c_str());
  CfgType tmp(this->GetTask()->GetRobot());
  while(ifs >> tmp)
    m_query.push_back(tmp);
}

template <typename MPTraits>
void
PushQueryToMA<MPTraits>::
Iterate() {
  for(auto& cfg : m_query)
    if(!m_medialAxisUtility.PushToMedialAxis(
          cfg, this->GetEnvironment()->GetBoundary()))
      throw RunTimeException(WHERE, "Cannot push to MA.");
}

template <typename MPTraits>
void
PushQueryToMA<MPTraits>::
Finalize() {
  ofstream ofs(m_outQueryFile.c_str());
  for(auto&  cfg : m_query)
    ofs << cfg << endl;
}

#endif

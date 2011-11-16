#ifndef MPUTILS_H_
#define MPUTILS_H_

#include "IOUtils.h"
#include "RoadmapGraph.h"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// MPBaseObject
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

class MPProblem;
class MPBaseObject {
  public: 
    MPBaseObject():m_problem(NULL), m_label(""), m_name(""), m_debug(false){};
    MPBaseObject(MPProblem* _problem, string _label="", string _name="", bool _debug=false) :
      m_problem(_problem), m_label(_label), m_name(_name), m_debug(_debug) {};
    MPBaseObject(XMLNodeReader& _node, MPProblem* _problem, string _name="") : 
      m_problem(_problem), m_label(""), m_name(_name), m_debug(false) { 
        ParseXML(_node); 
      };

    virtual ~MPBaseObject() {}

    virtual void ParseXML(XMLNodeReader& _node) {
      m_label = _node.stringXMLParameter("Label", false, "", "Label Identifier");
      m_debug = _node.boolXMLParameter("debug", false, false, "Run-time debug on(true)/off(false)");
    };

    MPProblem* GetMPProblem() {return m_problem;}
    void SetMPProblem(MPProblem* _m){m_problem = _m;}
    virtual void PrintOptions(ostream& _os) {};
    string GetLabel() const {return m_label;};
    string GetName() const {return m_name;}
    void SetName(string _s) {m_name = _s;}

  private:
    MPProblem* m_problem;
    string m_label;
  protected:
    string m_name;
    bool m_debug;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// GetCentroid
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template<class CFG, class WEIGHT>
CFG
GetCentroid(RoadmapGraph<CFG, WEIGHT>* graph, vector<typename RoadmapGraph<CFG, WEIGHT>::VID>& cc){
  CFG center;
  for(size_t i = 0; i < cc.size(); i++) {

    CFG cfg = (*(graph->find_vertex(cc[i]))).property();
    center.add(center, cfg);
  }
  center.divide(center, cc.size());
  return center;
};

#endif


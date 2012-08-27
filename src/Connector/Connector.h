#ifndef CONNECTOR_H_
#define CONNECTOR_H_

// Standard Headers
#include <sstream>

// Roadmap Headers
#include "Roadmap.h"
#include "MPProblem.h"

// Connector Headers
#include "ConnectionMethod.h"
#include "ConnectCCs.h"
#include "NeighborhoodConnection.h"
#include "PreferentialAttachment.h"
#include "OptimalConnection.h"
#include "OptimalRewire.h"
#include "ClosestVE.h"
//#include "RRTcomponents.h"
//#include "RayTracer.h"

// Utility Headers
#include "MetricUtils.h"

//#############################################################################
// A collection of connection methods
//#############################################################################
template <class CFG, class WEIGHT>
class Connector :  private ElementSet< ConnectionMethod<CFG,WEIGHT> >, public MPBaseObject{

  private:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

  public:
    typedef ElementSet<ConnectionMethod<CFG, WEIGHT> > ConnectionSet;
    typedef typename ConnectionSet::MethodPointer ConnectionPointer;

    ////////////////////////////////////////////////////////////////////////////////////
    /* CONSTRUCTORS AND DESTRUCTORS */
    ////////////////////////////////////////////////////////////////////////////////////
    Connector() : ConnectionSet(pmpl_detail::ConnectorMethodList()){}

    Connector(XMLNodeReader& _node, MPProblem* _problem) 
      : ConnectionSet(pmpl_detail::ConnectorMethodList()), MPBaseObject(_node, _problem){
        ConnectionSet::ParseXML(_node, _problem);
        PrintOptions(cout);
      }

    ~Connector(){}

    ////////////////////////////////////////////////////////////////////////////////////
    /* ACCESS METHODS */
    ////////////////////////////////////////////////////////////////////////////////////
    ConnectionPointer GetMethod(const string& _label){
      return this->GetElement(_label);
    }

    void AddMethod(string const& _label, ConnectionPointer _cp){
      this->AddElement(_label, _cp);
    }

    virtual void SetMPProblem(MPProblem* _mp){
      MPBaseObject::SetMPProblem(_mp);
      ConnectionSet::SetMPProblem(_mp);
    }

    ////////////////////////////////////////////////////////////////////////////////////
    /* DEBUG METHODS */
    ////////////////////////////////////////////////////////////////////////////////////
    void PrintOptions(ostream& _os){
      _os << "  Connection Methods" << endl;
      typename map<string, ConnectionPointer>::const_iterator Conn;
      for(Conn = this->ElementsBegin(); Conn != this->ElementsEnd(); ++Conn){
        _os << "  " << Conn->first << "::\t";
        Conn->second->PrintOptions(_os);
      }
    }
};

#endif /*_Connector_h_*/


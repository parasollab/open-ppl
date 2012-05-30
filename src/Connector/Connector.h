#ifndef _Connector_h_
#define _Connector_h_

// Standard Headers
#include <sstream>

// Roadmap Headers
#include "Roadmap.h"
#include "MPRegion.h"
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
class Connector :  private ElementSet< ConnectionMethod<CFG,WEIGHT> >,
  public MPBaseObject{

    private:
      typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

    public:
      typedef typename ElementSet<ConnectionMethod<CFG,WEIGHT> >::MethodPointer ConnectionPointer;

      ////////////////////////////////////////////////////////////////////////////////////
      /* CONSTRUCTORS AND DESTRUCTORS */
      ////////////////////////////////////////////////////////////////////////////////////
      Connector() : ElementSet< ConnectionMethod<CFG,WEIGHT> >(pmpl_detail::ConnectorMethodList()){}

      Connector(XMLNodeReader& inNode, MPProblem* in_pProblem) 
        : ElementSet< ConnectionMethod<CFG,WEIGHT> >(pmpl_detail::ConnectorMethodList()), MPBaseObject(inNode, in_pProblem){
          ParseXML(inNode);
        }

      ~Connector(){}

      ////////////////////////////////////////////////////////////////////////////////////
      /* ACCESS METHODS */
      ////////////////////////////////////////////////////////////////////////////////////
      ConnectionPointer GetMethod(const string& _label){
        return ElementSet<ConnectionMethod<CFG,WEIGHT> >::GetElement(_label);
      }
    
      void AddMethod(string const& _label, ConnectionPointer _cp){
        ElementSet<ConnectionMethod<CFG, WEIGHT> >::AddElement(_label, _cp);
      }

      virtual void SetMPProblem(MPProblem* _mp){
        MPBaseObject::SetMPProblem(_mp);
        ElementSet<ConnectionMethod<CFG, WEIGHT> >::SetMPProblem(_mp);
      }

      ////////////////////////////////////////////////////////////////////////////////////
      /* DEBUG METHODS */
      ////////////////////////////////////////////////////////////////////////////////////
      void PrintOptions(ostream& _os);

    protected:
      void ParseXML(XMLNodeReader& _node);
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void Connector<CFG,WEIGHT>::PrintOptions(ostream& _os){
  _os << "  Connection Methods" << endl;
  typename map<string, ConnectionPointer>::const_iterator Conn;
  for(Conn = ElementSet<ConnectionMethod<CFG,WEIGHT> >::ElementsBegin(); 
      Conn != ElementSet<ConnectionMethod<CFG,WEIGHT> >::ElementsEnd(); 
      ++Conn){
    _os << "  " << Conn->first << "::\t";
    Conn->second->PrintOptions(_os);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void Connector<CFG,WEIGHT>::ParseXML(XMLNodeReader& inNode) {
  XMLNodeReader::childiterator citr;
  //Iterate over child nodes
  for(citr = inNode.children_begin(); citr!= inNode.children_end(); ++citr) {
    if (!ElementSet<ConnectionMethod<CFG,WEIGHT> >::AddElement(citr->getName(), *citr, GetMPProblem())){
      citr->warnUnknownNode();
      exit(-1);
    }
  }
  PrintOptions(cout);
}

#endif /*_Connector_h_*/


///////////////////////////////////////////////////////////////////////////////
///\file XmlWrapper.h
/// This file provides a set of classes that wrap up
/// READONLY interaction with TinyXML.  
///
/// \author Roger Pearce
/// \date 2008.01.28
///////////////////////////////////////////////////////////////////////////////
#ifndef XML_WRAPPER_H
#define XML_WRAPPER_H

#ifndef TIXML_USE_STL
#define TIXML_USE_STL
#endif //ifndef TIXML_USE_STL
#include <cctype>
#include <string>
#include <vector>
#include <algorithm>


#include "tinyxml.h"

/// \class XMLNode
/// This is a wrapper class for XML handeling.  It is READONLY
/// and supports only trivial XML parsing.  Wrapping up TinyXML, 
/// it only supports TiXmlElements; this is because this is our
/// only need for intput XML files.
///
/// If for some reason you really really need the underlying TiXMLNode,
/// then make your class a friend of this one.  Adding the neccessary
/// functionality here is a better solution!
///
/// \todo Testing for inproper int,float,double is still not good
/// things like int=3.5 or double=0.6q are accepted but truncated.
///
/// Questions:  Should this ever terminate on fail?  or return -1?
class XMLNodeReader {
  /////////////////////////////////////////////////////////////////////////////
  //  Public Methods
  /////////////////////////////////////////////////////////////////////////////
public:
  typedef std::vector<XMLNodeReader>::iterator childiterator;
  
  /// Constructor for XMLNodeReader
  /// \param in_pnode the TiXmlNode this class wraps 
  explicit XMLNodeReader(TiXmlNode* in_pnode, const std::string& in_filename );

  /// This constructor takes as input a XML document and searches for a toplevel
  /// node by the name of in_desiredNode
  ///
  /// \param in_fileName The XML filename to parse
  /// \param in_desiredNode The top-level XML Node to return
  explicit XMLNodeReader(const std::string& in_filename, TiXmlDocument& in_doc, 
                            const std::string& in_desiredNode);


  //Accessing Methods//
  std::string getName() const;

  /// Returns true if XMLNode has a specific child
  ///
  /// \param in_childName Name of child queried
  bool hasChild(const std::string& in_childName) const;

  ///WARNING: will terminate with msg if child doesn't exist, 
  ///so its better to check outside!
  XMLNodeReader getFirstChild(const std::string& in_childName) ;


  childiterator children_begin();

  childiterator children_end();


  template<typename T>
  T numberXMLParameter (const std::string& in_strName,
                        bool in_req,
                        const T& in_default,
                        const T& in_min, 
                        const T& in_max, 
                        const std::string& in_strDesc) {
    verifyElement();
    m_vec_req_attributes.push_back(in_strName);
    T to_return;  
    //was QueryValueAttribute<T>
    int qr_return = m_pnode->ToElement()->QueryValueAttribute(in_strName,&to_return);
    if(qr_return == TIXML_WRONG_TYPE) {
      printAttrWrongType(in_strName,in_strDesc);
      exit(-1);
    } else if(qr_return == TIXML_NO_ATTRIBUTE && in_req == true) {
      printAttrMissing(in_strName, in_strDesc);
      exit(-1);
    } else if(qr_return == TIXML_NO_ATTRIBUTE && in_req == false) {
      to_return = in_default; //defaulting value b/c not found in XML file
    } else if(qr_return == TIXML_SUCCESS) {
      if(to_return < in_min || to_return > in_max) {
        printAttrInvalidBounds(in_strName,in_strDesc,in_min,in_max);
        exit(-1);
      }
    } else {
       std::cerr << "num_XML_param::numberXMLParameter Logic Error!" << std::endl;
    }

    return to_return;
  }

  std::string stringXMLParameter (const std::string& in_strName,
                                  bool in_req,
                                  const std::string& in_default,
                                  const std::string& in_strDesc);

  bool hasXMLParameter(const std::string& in_strName) ;

  bool boolXMLParameter (const std::string& in_strName, bool in_req,
                         bool in_default, const std::string& in_strDesc);
  /// Will output a warning to the user that XML Node 
  /// contains extra attributes.  This could be a sign
  /// of a user typeof on an optional attribute.
  void warnUnrequestedAttributes();

  void warnUnknownNode() ;
  ///Verify this node is what is expected, will terminate on fail
  void verifyName(const std::string& in_strName);

  std::ostream& operator<< (std::ostream& _out) {
    _out << *(this->m_pnode);
    return _out;
  }

  bool operator==(const XMLNodeReader& in_Node) const
  {
    return (m_pnode == in_Node.m_pnode) &&
           (m_bChildVecBuilt == in_Node.m_bChildVecBuilt) &&
           (m_vec_children == in_Node.m_vec_children) &&
           (m_vec_req_attributes == in_Node.m_vec_req_attributes);
  }

  /////////////////////////////////////////////////////////////////////////////
  //  Private Methods
  /////////////////////////////////////////////////////////////////////////////
  private:
  /// Returns true if the XMLNode is an element.  Used for error checking,
  /// Elements are may carry attributes.
  bool isElement() const {
    return (m_pnode->Type() == TiXmlNode::ELEMENT) ? true : false;
  }

  void verifyElement() const ;

  void printAttrWrongType(const std::string& in_strName, const std::string& in_strDesc) const ;
  void printAttrMissing(const std::string& in_strName, const std::string& in_strDesc) const;
  template<typename T>
  void printAttrInvalidBounds(const std::string& in_strName, const std::string& in_strDesc, 
                              const T& in_min, const T& in_max) const {
    std::cerr << "*************************************************************" << std::endl;
    std::cerr << "XML Error:: Invalid Attribute Range" << std::endl;
    std::cerr << "XML file:: " << m_xmlfilename << std::endl;
    std::cerr << "Parent Node: " << getName() << ", Attribute: "; 
    std::cerr << in_strName << std::endl;
    std::cerr << "Attribute Range: ["<< in_min << ", " << in_max << "]" << std::endl;
    std::cerr << "Attribute Description: " << in_strDesc << std::endl;
    std::cerr << *m_pnode << std::endl;
    std::cerr << "*************************************************************" << std::endl;
  }

  void printMissingRequestedChild(const std::string& in_childName);
  /// Builds the internal vector of children, used when iterating over
  /// the children.  This vector is only build with ELEMENT type nodes
  void buildChildVector();

  friend std::ostream& operator<< (std::ostream& _out, const XMLNodeReader& _node);

  bool hasRequestedAttr(std::string in_strName);
  /////////////////////////////////////////////////////////////////////////////
  //  Protected Data
  /////////////////////////////////////////////////////////////////////////////
  protected:
  

  /////////////////////////////////////////////////////////////////////////////
  //  Private Data
  /////////////////////////////////////////////////////////////////////////////
  private:
  TiXmlNode* m_pnode;
  bool m_bChildVecBuilt;
  std::vector<XMLNodeReader> m_vec_children; //made lazily!
  //std::vector<std::string> m_vec_req_children;
  std::vector<std::string> m_vec_req_attributes;
  std::string m_xmlfilename;
};






#endif //XML_WRAPPER_H
